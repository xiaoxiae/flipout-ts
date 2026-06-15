// Ported from geometry-central:
//   include/geometrycentral/surface/flip_geodesics.h
//   src/surface/flip_geodesics.cpp
//
// L4 — FlipEdgeNetwork: the "FlipOut" algorithm for finding geodesic paths
// in triangle meshes by edge flips
//   "You Can Find Geodesic Paths in Triangle Meshes by Just Flipping Edges"
//   Sharp & Crane, SIGGRAPH Asia 2020.
//
// The path is a doubly-linked list of `PathSegment`s, each of which holds an
// intrinsic halfedge. Each interior path vertex defines two wedges (left and
// right of the path). When the smaller of the two wedge angles is below
// `π − EPS_ANGLE`, the path is *not* locally straight at that vertex — we
// flip out the edge "blocking" the wedge (gc's `locallyShortenAt`) and
// splice the flipped edges into the path.
//
// Public surface (a TS-cased subset of gc's class):
//   - constructor(intrinsic, initialPath)
//   - flipOut(maxIterations)
//   - pathLength()
//   - pathHalfedges()
//   - extractPolyline()
//   - isMarkedVertex(v) / setMarkedVertex(v, marked)
//   - straightenAroundMarkedVertices (flag, default true)
//
// Naming & control flow follow gc verbatim: `measureSideAngles`,
// `locallyShortestTestWithType`, `locallyShortenAt`, `iterativeShorten`,
// `addToWedgeAngleQueue`, `replacePathSegment`, `wedgeAngleQueue`. See the
// comments next to each method for the gc function it mirrors.
//
// Marked vertices: gc supports tagging vertices as "control points" so the
// straightening pass treats them as fixed — flips that would straighten across
// a marked vertex are skipped. This is needed by the bezier subdivision
// algorithm. See `straightenAroundMarkedVertices` (gc default `true`, i.e. the
// flag has no effect — marked vertices are still passed through). Setting it
// `false` enforces the gating; gc's `wedgeIsClear` does the equivalent check
// in the multi-path case.

import type { Vec3 } from '../math/vec3.js';
import type {
  SignpostIntrinsicTriangulation,
  SurfacePoint,
} from '../intrinsic/signpost-intrinsic-triangulation.js';
import { shortestEdgePath } from './dijkstra.js';

/** gc's `SegmentAngleType`. `Shortest` means the wedge is locally straight. */
export const enum SegmentAngleType {
  Shortest = 0,
  LeftTurn = 1,
  RightTurn = 2,
}

/** A single segment in the path linked list — gc's `pathHeInfo[id]` tuple. */
interface PathSegment {
  /** Intrinsic halfedge for this segment. May be re-anchored (twin) by flips. */
  he: number;
  /** Previous segment id, or -1 if this is the head of an open path. */
  prevId: number;
  /** Next segment id, or -1 if this is the tail of an open path. */
  nextId: number;
}

/** Priority queue entry — gc's `WeightedAngle = (angle, type, segment)`. */
interface WedgeQueueEntry {
  angle: number;
  type: SegmentAngleType;
  segId: number;
  /** Generation used to detect stale entries (`pathHeInfo.find` lookup in gc). */
  gen: number;
}

/** Internal: a tiny min-heap keyed by (angle, then insertion order). */
class WedgeHeap {
  private heap: WedgeQueueEntry[] = [];

  size(): number {
    return this.heap.length;
  }
  push(e: WedgeQueueEntry): void {
    this.heap.push(e);
    this.siftUp(this.heap.length - 1);
  }
  pop(): WedgeQueueEntry | undefined {
    if (this.heap.length === 0) return undefined;
    const top = this.heap[0]!;
    const last = this.heap.pop()!;
    if (this.heap.length > 0) {
      this.heap[0] = last;
      this.siftDown(0);
    }
    return top;
  }
  private less(a: WedgeQueueEntry, b: WedgeQueueEntry): boolean {
    // Match gc's tiebreak: (angle ASC, type ASC, segId ASC). gc uses
    // `std::greater<tuple<double, SegmentAngleType, FlipPathSegment>>` whose
    // tuple compare cascades, with `FlipPathSegment::operator<` keying by
    // segment id (since all entries here share a single path*).
    if (a.angle !== b.angle) return a.angle < b.angle;
    if (a.type !== b.type) return a.type < b.type;
    return a.segId < b.segId;
  }
  private siftUp(i: number): void {
    const h = this.heap;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (!this.less(h[i]!, h[parent]!)) break;
      [h[parent], h[i]] = [h[i]!, h[parent]!];
      i = parent;
    }
  }
  private siftDown(i: number): void {
    const h = this.heap;
    const n = h.length;
    while (true) {
      const l = 2 * i + 1;
      const r = 2 * i + 2;
      let smallest = i;
      if (l < n && this.less(h[l]!, h[smallest]!)) smallest = l;
      if (r < n && this.less(h[r]!, h[smallest]!)) smallest = r;
      if (smallest === i) break;
      [h[smallest], h[i]] = [h[i]!, h[smallest]!];
      i = smallest;
    }
  }
}

/**
 * Thrown by {@link FlipEdgeNetwork.bezierSubdivide} when the path is not
 * simple at a control vertex (the curve passes through a vertex more than
 * once). gc has the same precondition. Callers that want to fall back to a
 * piecewise scheme should catch this rather than matching on the message text.
 */
export class BezierNonSimpleError extends Error {
  constructor(message = 'bezierSubdivide: multiple paths at vertex') {
    super(message);
    this.name = 'BezierNonSimpleError';
  }
}

/**
 * FlipOut implementation. Holds an intrinsic triangulation and a path on it
 * (here we support a single open path — the common shortest-path case). Use
 * {@link flipOut} to drive the path to a geodesic.
 *
 * Mirrors gc's `FlipEdgeNetwork`. The iteration loop, wedge classification,
 * and locally-shorten move are direct ports.
 */
export class FlipEdgeNetwork {
  /** The signpost intrinsic triangulation (mutated by flips). */
  readonly intrinsic: SignpostIntrinsicTriangulation;

  /** EPS for "is wedge locally straight?" — gc's `EPS_ANGLE` (default 1e-5). */
  EPS_ANGLE = 1e-5;

  /** All segments, indexed by id. ids are never reused — gc's `pathHeInfo`. */
  private readonly segments = new Map<number, PathSegment>();

  /** Monotonic id counter — gc's `nextUniquePathSegmentInd`. */
  private nextSegId = 0;

  /** The path's head (open path: prevId === -1). */
  private headId = -1;
  /** The path's tail (open path: nextId === -1). */
  private tailId = -1;

  /** Generation counter used to invalidate stale queue entries. */
  private gen = 0;

  /** Priority queue of wedge candidates — gc's `wedgeAngleQueue`. */
  private readonly wedgeAngleQueue = new WedgeHeap();

  /** Counters — gc's `nFlips` / `nShortenIters`. */
  nFlips = 0;
  nShortenIters = 0;

  /**
   * gc's `straightenAroundMarkedVertices`. When `true` (gc default), marked
   * vertices are still allowed to be straightened through — the flag has no
   * effect on a path with no obstructed wedges. When `false`, the
   * straightening pass skips any wedge whose junction vertex is marked,
   * effectively pinning the path through those control points.
   */
  straightenAroundMarkedVertices = true;

  /**
   * gc's `isMarkedVertex` (`VertexData<bool>` over the *intrinsic* mesh).
   * Stored as a Uint8Array; lazily grown when the intrinsic mesh gains
   * vertices via `insertVertex` (used by bezier subdivision).
   */
  private markedVertexBits: Uint8Array;

  /**
   * gc's `pathsAtEdge` simplified to a refcount per edge: the number of
   * path segments currently using each intrinsic edge (in either direction).
   * Used by the `wedgeIsClear` check to decide whether a candidate flip
   * would cross an edge that's already on the path elsewhere — important
   * in the bezier case where multiple sub-geodesics can meet at a non-mark
   * interior vertex and produce a non-simple curve.
   *
   * For our single-path layout this is at most 2 in pathological cases; we
   * just track the count and use `count > 0` as the predicate.
   */
  private edgeRefCounts: Int32Array;

  /**
   * Construct a network from an initial path of intrinsic halfedge indices.
   * Mirrors gc's `FlipEdgeNetwork(...)` + `FlipEdgePath(...)` for the
   * single-open-path case (closed loops + multi-paths are not supported).
   */
  constructor(intrinsic: SignpostIntrinsicTriangulation, initialPath: number[]) {
    this.intrinsic = intrinsic;
    this.markedVertexBits = new Uint8Array(intrinsic.intrinsicMesh.nVertices);
    this.edgeRefCounts = new Int32Array(intrinsic.intrinsicMesh.nEdges);
    if (initialPath.length === 0) {
      throw new Error('FlipEdgeNetwork: initial path must be non-empty');
    }
    // Validate: consecutive halfedges must be tip-to-tail connected.
    const m = intrinsic.intrinsicMesh;
    for (let i = 0; i + 1 < initialPath.length; i++) {
      const heA = initialPath[i]!;
      const heB = initialPath[i + 1]!;
      if (m.tipVertex(heA) !== m.vertex(heB)) {
        throw new Error(
          `FlipEdgeNetwork: initial path not connected at index ${i}: ` +
            `tip(he[${i}]=${heA})=${m.tipVertex(heA)} != tail(he[${i + 1}]=${heB})=${m.vertex(heB)}`,
        );
      }
    }

    // Build the linked list. gc allocates each new id, links to prev, etc.
    let prevId = -1;
    for (const he of initialPath) {
      const id = this.nextSegId++;
      this.segments.set(id, { he, prevId, nextId: -1 });
      this.incrementEdgeRef(m.edge(he));
      if (prevId !== -1) {
        const prev = this.segments.get(prevId)!;
        prev.nextId = id;
      } else {
        this.headId = id;
      }
      prevId = id;
    }
    this.tailId = prevId;

    // Seed the wedge queue with every interior junction.
    this.addAllWedgesToAngleQueue();
  }

  // ============================================================================
  // Diagnostic / accessor surface.
  // ============================================================================

  /** Total intrinsic length — sum of `edgeLengths[edge(seg.he)]` over the path. */
  pathLength(): number {
    let len = 0;
    for (const seg of this.iterPath()) {
      len += this.intrinsic.edgeLengths[this.intrinsic.intrinsicMesh.edge(seg.he)]!;
    }
    return len;
  }

  /** List of intrinsic halfedge indices currently in the path, in order. */
  pathHalfedges(): number[] {
    const out: number[] = [];
    for (const seg of this.iterPath()) out.push(seg.he);
    return out;
  }

  /** Vertices visited by the path, in order (length = #segments + 1). */
  pathVertices(): number[] {
    const m = this.intrinsic.intrinsicMesh;
    const segs = [...this.iterPath()];
    if (segs.length === 0) return [];
    const out: number[] = [m.vertex(segs[0]!.he)];
    for (const seg of segs) out.push(m.tipVertex(seg.he));
    return out;
  }

  // ============================================================================
  // Marked vertices — gc's `isMarkedVertex` accessors.
  //
  // The bit-array auto-grows when the intrinsic mesh gains vertices (via
  // `insertVertex`); reads past the current capacity return `false`.
  // ============================================================================

  /** Returns whether intrinsic vertex `v` is marked. */
  isMarkedVertex(v: number): boolean {
    return v < this.markedVertexBits.length && this.markedVertexBits[v] !== 0;
  }

  /** Mark or unmark intrinsic vertex `v`. */
  setMarkedVertex(v: number, marked: boolean): void {
    this.ensureMarkedCapacity(v + 1);
    this.markedVertexBits[v] = marked ? 1 : 0;
  }

  private ensureMarkedCapacity(n: number): void {
    if (n <= this.markedVertexBits.length) return;
    // Grow at least to the intrinsic mesh's current vertex count, with some
    // slack to amortize repeated growth during subdivision.
    const target = Math.max(n, this.intrinsic.intrinsicMesh.nVertices, this.markedVertexBits.length * 2);
    const grown = new Uint8Array(target);
    grown.set(this.markedVertexBits);
    this.markedVertexBits = grown;
  }

  // ============================================================================
  // edge ref-count maintenance — gc's `pathsAtEdge` reduced to a per-edge count.
  // ============================================================================

  private edgeInPath(e: number): boolean {
    return e < this.edgeRefCounts.length && this.edgeRefCounts[e]! > 0;
  }

  private incrementEdgeRef(e: number): void {
    this.ensureEdgeRefCapacity(e + 1);
    this.edgeRefCounts[e]! += 1;
  }

  private decrementEdgeRef(e: number): void {
    if (e >= this.edgeRefCounts.length) return;
    const c = this.edgeRefCounts[e]!;
    if (c <= 0) return;
    this.edgeRefCounts[e] = c - 1;
  }

  private ensureEdgeRefCapacity(n: number): void {
    if (n <= this.edgeRefCounts.length) return;
    const target = Math.max(n, this.intrinsic.intrinsicMesh.nEdges, this.edgeRefCounts.length * 2);
    const grown = new Int32Array(target);
    grown.set(this.edgeRefCounts);
    this.edgeRefCounts = grown;
  }

  // ============================================================================
  // Iteration — walk segments from head to tail (open path only).
  // ============================================================================

  *iterPath(): Generator<PathSegment, void, void> {
    let id = this.headId;
    while (id !== -1) {
      const seg = this.segments.get(id);
      if (seg === undefined) return;
      yield seg;
      id = seg.nextId;
    }
  }

  // ============================================================================
  // Wedge angle measurement — direct ports of gc's helpers.
  // ============================================================================

  /**
   * gc's `measureSideAngles(hePrev, heNext)` — `(leftAngle, rightAngle)`
   * around the junction vertex `v = vertex(heNext) = tipVertex(hePrev)`.
   *
   *     angleIn  = signpost[hePrev.twin()]    // direction the path arrived
   *     angleOut = signpost[heNext]           // direction it leaves
   *     s        = vertexAngleSums[v]
   *
   *   right = (angleIn  < angleOut) ? angleOut - angleIn  : (s - angleIn)  + angleOut   (interior)
   *   left  = (angleOut < angleIn ) ? angleIn  - angleOut : (s - angleOut) + angleIn    (interior)
   *
   * Boundary case: when the wedge would wrap around the missing exterior
   * (i.e. one of the second branches above is taken), the wedge angle is
   * +Infinity — the path can never short-circuit through the boundary.
   */
  private measureSideAngles(hePrev: number, heNext: number): { left: number; right: number } {
    const im = this.intrinsic.intrinsicMesh;
    const v = im.vertex(heNext);
    const s = this.intrinsic.vertexAngleSums[v]!;
    const angleIn = this.intrinsic.halfedgeSignposts[im.twin(hePrev)]!;
    const angleOut = this.intrinsic.halfedgeSignposts[heNext]!;
    const isBoundary = im.isBoundaryVertex(v);

    let rightAngle: number;
    if (angleIn < angleOut) {
      rightAngle = angleOut - angleIn;
    } else if (isBoundary) {
      rightAngle = Number.POSITIVE_INFINITY;
    } else {
      rightAngle = (s - angleIn) + angleOut;
    }

    let leftAngle: number;
    if (angleOut < angleIn) {
      leftAngle = angleIn - angleOut;
    } else if (isBoundary) {
      leftAngle = Number.POSITIVE_INFINITY;
    } else {
      leftAngle = (s - angleOut) + angleIn;
    }

    return { left: leftAngle, right: rightAngle };
  }

  /**
   * gc's `locallyShortestTestWithType(hePrev, heNext)`: returns
   * `(type, minAngle)` — `Shortest` if both wedges are >= π − EPS_ANGLE
   * (locally straight), otherwise the side whose wedge is smaller.
   */
  private locallyShortestTestWithType(
    hePrev: number,
    heNext: number,
  ): { type: SegmentAngleType; angle: number } {
    const { left, right } = this.measureSideAngles(hePrev, heNext);
    const minAngle = Math.min(left, right);
    if (left < right) {
      if (left > Math.PI - this.EPS_ANGLE) return { type: SegmentAngleType.Shortest, angle: minAngle };
      return { type: SegmentAngleType.LeftTurn, angle: minAngle };
    } else {
      if (right > Math.PI - this.EPS_ANGLE) return { type: SegmentAngleType.Shortest, angle: minAngle };
      return { type: SegmentAngleType.RightTurn, angle: minAngle };
    }
  }

  /**
   * gc's `locallyShortestTestWithBoth(hePrev, heNext)`: classifies BOTH
   * sides — used by `addToWedgeAngleQueue` to enqueue both potential moves.
   */
  private locallyShortestTestWithBoth(
    hePrev: number,
    heNext: number,
  ): {
    minType: SegmentAngleType;
    minAngle: number;
    maxType: SegmentAngleType;
    maxAngle: number;
  } {
    const { left, right } = this.measureSideAngles(hePrev, heNext);
    let minType: SegmentAngleType, minAngle: number, maxType: SegmentAngleType, maxAngle: number;
    if (left < right) {
      minAngle = left;
      maxAngle = right;
      minType = left > Math.PI - this.EPS_ANGLE ? SegmentAngleType.Shortest : SegmentAngleType.LeftTurn;
      maxType = right > Math.PI - this.EPS_ANGLE ? SegmentAngleType.Shortest : SegmentAngleType.RightTurn;
    } else {
      minAngle = right;
      maxAngle = left;
      minType = right > Math.PI - this.EPS_ANGLE ? SegmentAngleType.Shortest : SegmentAngleType.RightTurn;
      maxType = left > Math.PI - this.EPS_ANGLE ? SegmentAngleType.Shortest : SegmentAngleType.LeftTurn;
    }
    return { minType, minAngle, maxType, maxAngle };
  }

  /** Smallest of all wedge angles in the path (for diagnostics / convergence). */
  minWedgeAngle(): number {
    let minAngle = Number.POSITIVE_INFINITY;
    for (const seg of this.iterPath()) {
      if (seg.prevId === -1) continue;
      const prev = this.segments.get(seg.prevId)!;
      const { angle } = this.locallyShortestTestWithType(prev.he, seg.he);
      if (angle < minAngle) minAngle = angle;
    }
    return minAngle;
  }

  // ============================================================================
  // Wedge angle queue — gc's `addToWedgeAngleQueue` / `addAllWedgesToAngleQueue`.
  // ============================================================================

  /**
   * gc's `addToWedgeAngleQueue`: enqueue the smaller wedge of the junction
   * at the *tail* (==`vertex(seg.he)`) of segment `seg`. If both sides are
   * straight, nothing is enqueued. Both sides may be enqueued (smaller
   * first) so the smaller side is processed first.
   */
  private addToWedgeAngleQueue(segId: number): void {
    const seg = this.segments.get(segId);
    if (seg === undefined) return;
    if (seg.prevId === -1) return; // first halfedge of an open path — no junction

    const prev = this.segments.get(seg.prevId)!;
    const result = this.locallyShortestTestWithBoth(prev.he, seg.he);

    if (result.minType !== SegmentAngleType.Shortest) {
      this.wedgeAngleQueue.push({ angle: result.minAngle, type: result.minType, segId, gen: this.gen });
    }
    if (result.maxType !== SegmentAngleType.Shortest) {
      this.wedgeAngleQueue.push({ angle: result.maxAngle, type: result.maxType, segId, gen: this.gen });
    }
  }

  private addAllWedgesToAngleQueue(): void {
    for (const seg of this.iterPath()) {
      if (seg.prevId === -1) continue;
      this.addToWedgeAngleQueue(this.segIdOf(seg));
    }
  }

  /** Reverse-lookup: find segment id by reference. Linear scan, only used for diagnostics. */
  private segIdOf(seg: PathSegment): number {
    for (const [id, s] of this.segments) {
      if (s === seg) return id;
    }
    throw new Error('FlipEdgeNetwork: segment not found in segments map');
  }

  // ============================================================================
  // The locally-shorten move — direct port of gc's `locallyShortenAt`.
  //
  // For a wedge of type LeftTurn at the junction between `hePrev` and `heNext`:
  //
  //   - Walk inside the wedge starting from the next halfedge after `sPrev`.
  //   - Repeatedly try to flip whatever edge our walk is on.
  //   - If the flip succeeds, step backwards (`sCurr.twin().next().twin()`)
  //     to re-process the previous edge (whose angle has now changed).
  //   - If it fails (boundary, would create duplicate edge, or numerical),
  //     advance forward (`sCurr.twin().next()`).
  //   - Stop when we reach `sNext`. Then read out the new path along the
  //     sequence of `sCurr.next().twin()` halfedges.
  //
  // RightTurn is the same, with `sPrev/sNext = heNext.twin() / hePrev.twin()`
  // (i.e. flip the orientation so the inner walk is again CW).
  // ============================================================================

  private locallyShortenAt(segId: number, angleType: SegmentAngleType): void {
    if (angleType === SegmentAngleType.Shortest) return;
    const seg = this.segments.get(segId);
    if (seg === undefined) return;
    if (seg.prevId === -1) return;

    const prev = this.segments.get(seg.prevId)!;
    const heNext = seg.he;
    const hePrev = prev.he;

    this.nShortenIters++;

    const initLength =
      this.intrinsic.edgeLengths[this.intrinsic.intrinsicMesh.edge(hePrev)]! +
      this.intrinsic.edgeLengths[this.intrinsic.intrinsicMesh.edge(heNext)]!;

    const m = this.intrinsic.intrinsicMesh;

    // Set up `sPrev`/`sNext` so the inner walk traces CW around the wedge.
    let sPrev: number;
    let sNext: number;
    let reversed: boolean;
    if (angleType === SegmentAngleType.LeftTurn) {
      sPrev = hePrev;
      sNext = heNext;
      reversed = false;
    } else {
      sPrev = m.twin(heNext);
      sNext = m.twin(hePrev);
      reversed = true;
    }

    // Inner flip loop — walks `sCurr` from `sPrev.next()` to `sNext`, flipping
    // every "in-the-way" edge. After a successful flip, step back one edge
    // (`sCurr.twin().next().twin()`) so we re-test the predecessor whose
    // angle just changed.
    {
      let sCurr = m.next(sPrev);
      const sPrevTwin = m.twin(sPrev);
      let safety = 0;
      const safetyCap = m.nHalfedges * 4 + 16;
      while (sCurr !== sNext) {
        if (++safety > safetyCap) {
          // Numerical pathology — bail out without modifying the path.
          return;
        }
        if (sCurr === sPrevTwin) {
          sCurr = m.next(m.twin(sCurr));
          continue;
        }
        const currEdge = m.edge(sCurr);
        const flipped = this.intrinsic.flipEdge(currEdge);
        if (flipped) {
          this.nFlips++;
          // After flipping we move `sCurr` to `sCurr.twin().next().twin()` —
          // the same physical step gc does. Note `currEdge`'s halfedge slot
          // is preserved by the flip, only its endpoints change.
          sCurr = m.twin(m.next(m.twin(sCurr)));
        } else {
          sCurr = m.next(m.twin(sCurr));
        }
      }
    }

    // Read out the new path: walk `sCurr` from `sPrev.next()` to `sNext`
    // and collect `sCurr.next().twin()` (the post-flip "outside boundary"
    // edge of the wedge — this becomes a path edge after replacement).
    const newPath: number[] = [];
    let newPathLength = 0;
    {
      let sCurr = m.next(sPrev);
      let safety = 0;
      const safetyCap = m.nHalfedges * 4 + 16;
      while (true) {
        if (++safety > safetyCap) return; // pathological
        const heAdd = m.twin(m.next(sCurr));
        newPath.push(heAdd);
        newPathLength += this.intrinsic.edgeLengths[m.edge(heAdd)]!;
        if (sCurr === sNext) break;
        sCurr = m.next(m.twin(sCurr));
      }
    }

    // Defensive: gc skips the splice if the new path is *longer* (rare FP edge case).
    if (newPathLength > initLength) return;

    // Restore orientation if we operated in reversed mode.
    if (reversed) {
      newPath.reverse();
      for (let i = 0; i < newPath.length; i++) newPath[i] = m.twin(newPath[i]!);
    }

    this.replacePathSegment(segId, newPath);
  }

  /**
   * gc's `FlipEdgePath::replacePathSegment` for the open-path case: replace
   * the two segments `(prev, curr)` with the list `newHalfedges`. Updates
   * the linked list, schedules new wedges in the queue, and invalidates
   * stale queue entries via `gen++`.
   */
  private replacePathSegment(currId: number, newHalfedges: number[]): void {
    const curr = this.segments.get(currId);
    if (curr === undefined || curr.prevId === -1) return;
    const prev = this.segments.get(curr.prevId)!;
    const m = this.intrinsic.intrinsicMesh;

    const prevPrevId = prev.prevId;
    const nextNextId = curr.nextId;

    // Remove the two old segments.
    this.decrementEdgeRef(m.edge(prev.he));
    this.decrementEdgeRef(m.edge(curr.he));
    this.segments.delete(curr.prevId);
    this.segments.delete(currId);

    // Insert new segments between prevPrev and nextNext.
    let runningPrevId = prevPrevId;
    let firstAddedId = -1;
    for (const newHe of newHalfedges) {
      const newId = this.nextSegId++;
      this.segments.set(newId, { he: newHe, prevId: runningPrevId, nextId: -1 });
      this.incrementEdgeRef(m.edge(newHe));
      if (runningPrevId !== -1) {
        this.segments.get(runningPrevId)!.nextId = newId;
      }
      if (firstAddedId === -1) firstAddedId = newId;
      runningPrevId = newId;
    }

    if (runningPrevId !== -1) {
      this.segments.get(runningPrevId)!.nextId = nextNextId;
    }
    if (nextNextId !== -1) {
      this.segments.get(nextNextId)!.prevId = runningPrevId;
    }

    // Update head/tail pointers.
    if (prevPrevId === -1) {
      // The new front replaces the previous head.
      this.headId = firstAddedId !== -1 ? firstAddedId : nextNextId;
    }
    if (nextNextId === -1) {
      // The new back is the new tail.
      this.tailId = runningPrevId !== -1 ? runningPrevId : prevPrevId;
    }

    // Bump the generation so any pre-existing queue entries referencing
    // either of the two deleted segments become "stale" (gc tracks this
    // via `pathHeInfo.find`; we use a simple gen counter).
    this.gen++;

    // Schedule wedge checks at every junction touched by the new path.
    // For each new segment, enqueue its wedge.
    let id = firstAddedId;
    while (id !== -1) {
      this.addToWedgeAngleQueue(id);
      const s = this.segments.get(id);
      if (s === undefined) break;
      if (id === runningPrevId) break;
      id = s.nextId;
    }
    // Also enqueue the wedge AFTER the last new segment (its prev pointer
    // may have changed and its angle is now different).
    if (nextNextId !== -1) {
      this.addToWedgeAngleQueue(nextNextId);
    }
  }

  // ============================================================================
  // Main driver — gc's `iterativeShorten`.
  // ============================================================================

  /**
   * Run FlipOut until convergence (queue empty) or `maxIterations` shorten
   * moves have been performed. gc's `iterativeShorten`.
   */
  /**
   * gc's `wedgeIsClear`. Returns true iff the wedge at the junction between
   * `hePrev` and `heNext` may be straightened. Two reasons it can be blocked:
   *
   *   1. The junction vertex is marked AND `straightenAroundMarkedVertices`
   *      is off (i.e. we're treating it as a fixed control point).
   *   2. Some intrinsic edge inside the wedge fan is already on the path
   *      elsewhere — straightening would force the path to cross itself.
   *      This only matters when the path is non-simple, which happens
   *      naturally during bezier subdivision when two adjacent sub-geodesics
   *      share an interior vertex.
   */
  private wedgeIsClear(hePrev: number, heNext: number, type: SegmentAngleType): boolean {
    if (type === SegmentAngleType.Shortest) return true;
    const m = this.intrinsic.intrinsicMesh;
    const middleVert = m.vertex(heNext);

    if (!this.straightenAroundMarkedVertices && this.isMarkedVertex(middleVert)) {
      return false;
    }

    // Orbit incident edges in the wedge fan; none of them may already be in
    // the path. The fan walk mirrors gc's `next/twin/next` chains.
    if (type === SegmentAngleType.LeftTurn) {
      let heCurr = m.next(hePrev);
      while (heCurr !== heNext) {
        if (this.edgeInPath(m.edge(heCurr))) return false;
        heCurr = m.next(m.twin(heCurr));
      }
    } else {
      // RightTurn. gc starts at `hePrev.twin().next().next().twin()` and
      // steps `heCurr.next().next().twin()` until reaching heNext.
      let heCurr = m.twin(m.next(m.next(m.twin(hePrev))));
      while (heCurr !== heNext) {
        if (this.edgeInPath(m.edge(heCurr))) return false;
        heCurr = m.twin(m.next(m.next(heCurr)));
      }
    }
    return true;
  }

  flipOut(maxIterations = 100000): { iterations: number; converged: boolean } {
    let iterations = 0;

    while (this.wedgeAngleQueue.size() > 0) {
      const top = this.wedgeAngleQueue.pop()!;

      const seg = this.segments.get(top.segId);
      if (seg === undefined) continue;
      if (seg.prevId === -1) continue;

      const prev = this.segments.get(seg.prevId)!;
      const { type: currType, angle: currAngle } = this.locallyShortestTestWithType(prev.he, seg.he);
      if (currType === SegmentAngleType.Shortest) continue;
      // gc drops stale entries silently using strict-equality on the angle:
      //   - `locallyShortestTestWithType` returns the *smaller* side, and
      //     queue entries enqueued for the larger side never match.
      //   - When a neighboring flip mutates this wedge, that flip's own
      //     `addToWedgeAngleQueue` call inserts a fresh entry with the
      //     up-to-date angle. We don't re-enqueue here.
      // Strict `!==` mirrors gc; a tolerance would accept stale entries
      // gc would discard, producing extra flips and (under the marked-vertex
      // gate) bezier paths that diverge from gc's reference.
      if (currType !== top.type) continue;
      if (currAngle !== top.angle) continue;

      if (!this.wedgeIsClear(prev.he, seg.he, top.type)) continue;

      // About to perform a real flip — enforce the iteration cap here so
      // queues full of stale entries don't trigger spurious converged=false
      // when called with maxIterations exactly equal to the natural iter
      // count. Re-enqueue the popped entry so a follow-up flipOut() resumes.
      if (iterations >= maxIterations) {
        this.addToWedgeAngleQueue(top.segId);
        return { iterations, converged: false };
      }

      // Perform the locally-shorten move.
      this.locallyShortenAt(top.segId, top.type);
      iterations++;
    }
    return { iterations, converged: true };
  }

  // ============================================================================
  // Bezier subdivision — direct ports of gc's `bezierSubdivide` and
  // `bezierSubdivideRecursive` from `flip_geodesics.cpp`. Implements the
  // de-Casteljau-style scheme of "Modeling on triangulations with geodesic
  // curves" (Morera, Velho & de Carvalho 2008), but with FlipOut as the
  // straightening oracle.
  //
  // Preconditions (matching gc):
  //   - the network is a single open path forming a connected curve
  //   - every control vertex is marked (`isMarkedVertex == true`)
  //   - the path is simple (no repeated vertex)
  // ============================================================================

  /**
   * gc's `FlipEdgeNetwork::bezierSubdivide`. Performs `nRounds` of
   * de-Casteljau-style subdivision on the current path; the path converges to
   * an exact geodesic Bezier curve as `nRounds → ∞`.
   *
   * Mutates the underlying intrinsic triangulation by inserting midpoint
   * vertices on edges (via `insertVertex_edge`).
   *
   * gc-divergence note: in 14 of 17 fixture cases this port matches gc to
   * machine epsilon; in 3 (teapot/spot with many near-degenerate geodesic
   * ties) it produces a valid-but-different Bezier curve, drifting up to ~1%
   * in length. Root cause is V8's `Math.acos` returning bit patterns that
   * differ from glibc's `std::acos` by 1 ulp on certain inputs; the drift
   * cascades into priority-queue tie-breaking. See `CLAUDE.md` ("Bezier
   * subdivision") and `test/unit/flipout/bezier-fixtures.test.ts` for the
   * full investigation.
   */
  bezierSubdivide(nRounds: number, options: { maxIterations?: number } = {}): void {
    if (!Number.isInteger(nRounds) || nRounds < 0) {
      throw new RangeError(`bezierSubdivide: nRounds must be a non-negative integer, got ${nRounds}`);
    }
    const maxIter = options.maxIterations ?? 1_000_000;

    // gc disables `straightenAroundMarkedVertices` for the duration of the
    // call so iterativeShorten respects control points.
    const oldStraighten = this.straightenAroundMarkedVertices;
    this.straightenAroundMarkedVertices = false;

    // Ensure the curve is straight to start with.
    this.flipOut(maxIter);

    const im = this.intrinsic.intrinsicMesh;
    const head = this.segments.get(this.headId);
    const tail = this.segments.get(this.tailId);
    if (head === undefined || tail === undefined) {
      this.straightenAroundMarkedVertices = oldStraighten;
      throw new Error('bezierSubdivide: empty path');
    }
    const firstControlCall = im.vertex(head.he);
    const lastControlCall = im.tipVertex(tail.he);

    try {
      this.bezierSubdivideRecursive(nRounds, firstControlCall, lastControlCall, maxIter);
    } finally {
      this.straightenAroundMarkedVertices = oldStraighten;
    }
  }

  /**
   * gc's `FlipEdgeNetwork::bezierSubdivideRecursive`. Each call inserts one
   * new control point at the midpoint of `firstControlCall .. lastControlCall`
   * (as defined by the iterative-refinement procedure described in gc) and
   * recurses on both halves until `nRoundsRemaining` reaches zero.
   *
   * Region boundaries are tracked by *vertices*, not segments, since segments
   * are constantly rewritten by intervening straightening passes.
   */
  private bezierSubdivideRecursive(
    nRoundsRemaining: number,
    firstControlCall: number,
    lastControlCall: number,
    maxIter: number,
  ): void {
    if (nRoundsRemaining === 0) return;
    const im = this.intrinsic.intrinsicMesh;

    // Tolerance for "tSplit close enough to 1 → snap to existing vertex".
    const useVertexEPS = 1e-4;

    let firstControlActive = firstControlCall;
    let lastControlActive = lastControlCall;
    let newMidpoint = -1;

    while (true) {
      const firstSegId = this.findSegmentAfterId(firstControlActive);
      const lastSegId = this.findSegmentBeforeId(lastControlActive);
      if (firstSegId === -1 || lastSegId === -1) {
        throw new Error('bezierSubdivide: could not find first/last segment');
      }

      // Walk the path between firstSeg and lastSeg, partitioning into regions
      // delimited by marked vertices (= "control regions").
      interface Region {
        firstSegId: number;
        lastSegId: number;
        length: number;
      }
      const regions: Region[] = [];
      {
        let currId = firstSegId;
        let regionStartId = firstSegId;
        let length = 0;
        while (true) {
          const curr = this.segments.get(currId)!;
          length += this.intrinsic.edgeLengths[im.edge(curr.he)]!;
          const tip = im.tipVertex(curr.he);
          if (this.isMarkedVertex(tip)) {
            regions.push({ firstSegId: regionStartId, lastSegId: currId, length });
            if (currId === lastSegId) break;
            currId = curr.nextId;
            regionStartId = currId;
            length = 0;
          } else {
            currId = curr.nextId;
          }
          if (currId === -1) {
            throw new Error('bezierSubdivide: walked off path before reaching lastSeg');
          }
        }
      }

      const isLast = regions.length === 1;

      // Unmark all old interior control points (except on the last iteration —
      // we want to keep at least the bracketing controls of the recursive
      // call alive).
      if (!isLast) {
        for (const region of regions) {
          let pId = region.firstSegId;
          while (true) {
            if (pId !== firstSegId) {
              const p = this.segments.get(pId)!;
              const vertBefore = im.vertex(p.he);
              this.setMarkedVertex(vertBefore, false);
              const sId = this.findSegmentAfterId(vertBefore);
              if (sId !== -1) this.addToWedgeAngleQueue(sId);
            }
            if (pId === region.lastSegId) break;
            pId = this.segments.get(pId)!.nextId;
          }
        }
      }

      // For each region, find the segment containing the midpoint by
      // arc-length, split (or snap), mark the new vertex.
      const newControlPoints: number[] = [];
      for (const region of regions) {
        const halfLen = region.length * 0.5;
        let runningLen = 0;
        let pId = region.firstSegId;
        while (true) {
          const p = this.segments.get(pId)!;
          const eLen = this.intrinsic.edgeLengths[im.edge(p.he)]!;
          const nextLen = runningLen + eLen;
          if ((1 + useVertexEPS) * nextLen > halfLen) break;
          if (pId === region.lastSegId) {
            throw new Error("bezierSubdivide: couldn't find split segment");
          }
          runningLen = nextLen;
          pId = p.nextId;
        }

        const splitId = pId;
        const split = this.segments.get(splitId)!;
        const splitELen = this.intrinsic.edgeLengths[im.edge(split.he)]!;
        const tSplit = (halfLen - runningLen) / splitELen;

        let newControlP: number;
        if (tSplit > 1 - useVertexEPS) {
          // Snap to the existing tip vertex — happens often on regular grids.
          newControlP = im.tipVertex(split.he);
        } else {
          newControlP = this.splitSegmentEdge(splitId, tSplit);
        }
        this.setMarkedVertex(newControlP, true);
        newControlPoints.push(newControlP);
      }

      // Shrink the active region to the span between the new endpoints.
      firstControlActive = newControlPoints[0]!;
      lastControlActive = newControlPoints[newControlPoints.length - 1]!;

      // Straighten to geodesic before the next refinement pass.
      this.flipOut(maxIter);

      if (isLast) {
        newMidpoint = newControlPoints[0]!;
        break;
      }
    }

    // Recurse on both halves.
    this.bezierSubdivideRecursive(nRoundsRemaining - 1, firstControlCall, newMidpoint, maxIter);
    this.bezierSubdivideRecursive(nRoundsRemaining - 1, newMidpoint, lastControlCall, maxIter);
  }

  /**
   * gc's `findSegmentAfter(v)`: the segment in the path whose tail is `v`.
   * Throws if multiple segments are incident on the same side (path is not
   * simple at `v`). Returns `-1` if no segment matches.
   */
  private findSegmentAfterId(v: number): number {
    const im = this.intrinsic.intrinsicMesh;
    let id = this.headId;
    let found = -1;
    while (id !== -1) {
      const seg = this.segments.get(id)!;
      if (im.vertex(seg.he) === v) {
        if (found !== -1) throw new BezierNonSimpleError();
        found = id;
      }
      id = seg.nextId;
    }
    return found;
  }

  /** gc's `findSegmentBefore(v)`: the segment whose tip is `v`. */
  private findSegmentBeforeId(v: number): number {
    const im = this.intrinsic.intrinsicMesh;
    let id = this.headId;
    let found = -1;
    while (id !== -1) {
      const seg = this.segments.get(id)!;
      if (im.tipVertex(seg.he) === v) {
        if (found !== -1) throw new BezierNonSimpleError();
        found = id;
      }
      id = seg.nextId;
    }
    return found;
  }

  /**
   * gc's `FlipPathSegment::splitEdge(tSplit)` + `updatePathAfterEdgeSplit`.
   * Splits the edge under segment `segId` at parameter `tSplit ∈ (0, 1)`
   * along the segment's halfedge orientation, inserting a new intrinsic
   * vertex and replacing the segment with two consecutive segments.
   * Returns the new vertex's index.
   *
   * The two new wedges (at the original tail and at the new vertex) are
   * re-enqueued for straightening — the geometry of the new edges is
   * straight by construction, but numerical drift in signposts can shift
   * the angle by ~ε.
   */
  private splitSegmentEdge(segId: number, tSplit: number): number {
    const seg = this.segments.get(segId)!;
    const im = this.intrinsic.intrinsicMesh;
    const origHe = seg.he;
    const origTail = im.vertex(origHe);
    const origTip = im.tipVertex(origHe);
    const origEdge = im.edge(origHe);
    // `insertVertex_edge` interprets t along the edge's canonical halfedge.
    // If our segment runs along the canonical halfedge, t passes through;
    // otherwise we flip it.
    const origForward = origHe === im.edgeHalfedge(origEdge);
    const insertT = origForward ? tSplit : 1 - tSplit;

    const newV = this.intrinsic.insertVertex_edge(origEdge, insertT);

    // Find the two halves of the original edge as outgoing halfedges of
    // newV — they're the ones whose tip is one of the original endpoints.
    let frontHe = -1; // newV → origTip
    let backHe = -1; // newV → origTail
    for (const he of im.outgoingHalfedges(newV)) {
      if (im.tipVertex(he) === origTip) frontHe = he;
      else if (im.tipVertex(he) === origTail) backHe = he;
    }
    if (frontHe === -1 || backHe === -1) {
      throw new Error('splitSegmentEdge: front/back halfedges of new vertex not found');
    }

    // Wire the two new segments in path orientation.
    //   first segment (kept id):  origTail → newV  = twin(backHe)
    //   second segment (new id):  newV → origTip   = frontHe
    // The original edge ref was held by `seg`; release it and acquire two
    // new refs (one per new edge half).
    this.decrementEdgeRef(origEdge);
    seg.he = im.twin(backHe);
    this.incrementEdgeRef(im.edge(seg.he));

    const newSegId = this.nextSegId++;
    const newSeg: PathSegment = { he: frontHe, prevId: segId, nextId: seg.nextId };
    this.segments.set(newSegId, newSeg);
    this.incrementEdgeRef(im.edge(frontHe));

    if (seg.nextId !== -1) {
      const oldNext = this.segments.get(seg.nextId)!;
      oldNext.prevId = newSegId;
    } else {
      this.tailId = newSegId;
    }
    seg.nextId = newSegId;

    this.addToWedgeAngleQueue(segId);
    this.addToWedgeAngleQueue(newSegId);

    return newV;
  }

  // ============================================================================
  // Polyline extraction.
  //
  // Mirrors gc's `getPathPolyline` -> `traceIntrinsicHalfedgeAlongInput` for
  // each path halfedge. We use L3's `tracePolylineFromVertex` (added for L4).
  // ============================================================================

  /**
   * Extract a 3D polyline by tracing each path segment independently across
   * the input mesh and concatenating the per-segment polylines (de-duplicating
   * shared endpoints). Mirrors gc's `getPathPolyline` /
   * `traceIntrinsicHalfedgeAlongInput`.
   *
   * Each segment's intrinsic halfedge is traced from its TAIL's input-mesh
   * location, in the tail's input tangent frame, for the segment's intrinsic
   * length:
   *
   *   - Original input vertices trace from the vertex itself; the rescaled
   *     signpost is already an input tangent angle.
   *   - Vertices inserted during FlipOut / bezier subdivision trace from the
   *     `SurfacePoint` resolved by `resolveNewVertex`, using the per-vertex
   *     frame offset via `insertedTraceAngle` (the raw signpost frame is not
   *     input-aligned once the mesh has been flipped).
   *
   * Because every vertex carries a correct input-mesh location, segments with
   * both endpoints inserted (which arise during deep bezier subdivision) trace
   * just like any other — no straight-chord fallback is needed.
   */
  extractPolyline(): Vec3[] {
    const im = this.intrinsic.intrinsicMesh;
    const inputNV = this.intrinsic.inputGeometry.mesh.nVertices;

    const parts: Vec3[][] = [];
    for (const seg of this.iterPath()) {
      const he = seg.he;
      const tail = im.vertex(he);
      const len = this.intrinsic.edgeLengths[im.edge(he)]!;

      if (tail < inputNV) {
        const angle =
          this.intrinsic.halfedgeSignposts[he]! /
          this.intrinsic.vertexAngleScaling(tail);
        parts.push(this.intrinsic.tracePolylineFromVertex(tail, angle, len));
      } else {
        const loc = this.intrinsic.insertedVertexLocations.get(tail);
        if (loc === undefined) {
          throw new Error(`extractPolyline: inserted vertex ${tail} has no input location`);
        }
        const angle = this.intrinsic.insertedTraceAngle(he);
        parts.push(this.intrinsic.tracePolylineFromSurfacePoint(loc, angle, len));
      }
    }
    return concatPolylines(parts);
  }
}

/**
 * High-level convenience function: build an initial Dijkstra path from
 * `vSrc` to `vDst` on the input mesh's edge graph, run FlipOut, and
 * return polyline + length + iteration counts.
 *
 * Mirrors gc's `FlipEdgeNetwork::constructFromDijkstraPath` followed by
 * `iterativeShorten` and `getPathPolyline3D`.
 */
export function flipOutPath(
  intrinsic: SignpostIntrinsicTriangulation,
  vSrc: number,
  vDst: number,
  options: { maxIterations?: number } = {},
): { polyline: Vec3[]; length: number; iterations: number; converged: boolean } {
  const initial = shortestEdgePath(intrinsic, vSrc, vDst);
  if (initial === null) {
    throw new Error(`flipOutPath: no path from vertex ${vSrc} to vertex ${vDst}`);
  }
  const network = new FlipEdgeNetwork(intrinsic, initial);
  const { iterations, converged } = network.flipOut(options.maxIterations ?? 100000);
  const polyline = network.extractPolyline();
  const length = network.pathLength();
  return { polyline, length, iterations, converged };
}

// ===========================================================================
// flipOutPathFromSurfacePoints — extension over `flipOutPath` that accepts
// arbitrary surface points (vertex / edge / face). Source and destination
// points that are not at existing vertices are inserted into the intrinsic
// triangulation via L3's `insertVertex_*` before running the standard
// FlipOut algorithm.
//
// The intrinsic triangulation is mutated in place by both insertion and
// flipping. Callers should construct a fresh `SignpostIntrinsicTriangulation`
// per path query (same constraint as `flipOutPath`).
// ===========================================================================

/** Snap policy: if a barycentric / edge t lands within this epsilon of an
 *  existing vertex, snap to that vertex instead of inserting. */
export const SNAP_EPS = 1e-9;

/**
 * Resolve a `SurfacePoint` to an intrinsic vertex index, inserting a new
 * intrinsic vertex if needed. Snaps to an existing vertex when the point
 * is within {@link SNAP_EPS} of one (max bary coord > 1 - eps for face,
 * t < eps or t > 1 - eps for edge).
 */
function resolveSurfacePoint(
  intrinsic: SignpostIntrinsicTriangulation,
  point: SurfacePoint,
): number {
  if (point.kind === 'vertex') {
    return point.vertex;
  }
  if (point.kind === 'edge') {
    return intrinsic.insertVertex_edge(point.edge, point.t);
  }
  // face
  return intrinsic.insertVertex_face(point.face, point.bary);
}

/**
 * Trace one path segment's geodesic across the input mesh to produce a 3D
 * polyline. Handles three cases:
 *
 *   1. Both endpoints are *original* input-mesh vertices — uses the
 *      existing `tracePolylineFromVertex` from the tail.
 *   2. The tail is an inserted vertex but the tip is original — traces
 *      *backwards* from the tip along `twin(seg.he)`, then reverses the
 *      output. The first point (after reversal) is the inserted vertex's
 *      stored 3D position; we replace it with the exact stored value to
 *      avoid FP drift between the trace endpoint and the stored position.
 *   3. The tip is an inserted vertex but the tail is original — traces
 *      forward from the tail; the trace's endpoint is the inserted
 *      vertex's stored position; we replace it with the exact stored value.
 *
 * Case "both inserted" is impossible for a path of length ≥ 2 (we only
 * ever insert at most two vertices and they're never adjacent unless the
 * path itself has length 1, which we handle explicitly in the caller).
 */
function traceSegmentPolyline(
  intrinsic: SignpostIntrinsicTriangulation,
  segHe: number,
  insertedSrcV: number,
  insertedDstV: number,
): Vec3[] {
  const im = intrinsic.intrinsicMesh;
  const tail = im.vertex(segHe);
  const tip = im.tipVertex(segHe);
  const insertedSet = new Set<number>();
  if (insertedSrcV >= 0) insertedSet.add(insertedSrcV);
  if (insertedDstV >= 0) insertedSet.add(insertedDstV);
  const tailIsInserted = insertedSet.has(tail);
  const tipIsInserted = insertedSet.has(tip);

  const len = intrinsic.edgeLengths[im.edge(segHe)]!;

  if (!tailIsInserted && !tipIsInserted) {
    // Standard case — both endpoints are original. Trace from tail.
    const rawAngle = intrinsic.halfedgeSignposts[segHe]!;
    const rescaled = rawAngle / intrinsic.vertexAngleScaling(tail);
    return intrinsic.tracePolylineFromVertex(tail, rescaled, len);
  }

  if (!tailIsInserted && tipIsInserted) {
    // Trace forward from tail; final endpoint is the inserted dst — use
    // its stored 3D position to land exactly there.
    const rawAngle = intrinsic.halfedgeSignposts[segHe]!;
    const rescaled = rawAngle / intrinsic.vertexAngleScaling(tail);
    const out = intrinsic.tracePolylineFromVertex(tail, rescaled, len);
    const tipLoc = intrinsic.insertedVertexLocations.get(tip);
    if (tipLoc !== undefined && out.length > 0) {
      out[out.length - 1] = intrinsic.surfacePointPosition(tipLoc);
    }
    return out;
  }

  if (tailIsInserted && !tipIsInserted) {
    // Trace from tip backward along twin, then reverse.
    const heTwin = im.twin(segHe);
    const rawAngle = intrinsic.halfedgeSignposts[heTwin]!;
    const rescaled = rawAngle / intrinsic.vertexAngleScaling(tip);
    const out = intrinsic.tracePolylineFromVertex(tip, rescaled, len);
    out.reverse();
    const tailLoc = intrinsic.insertedVertexLocations.get(tail);
    if (tailLoc !== undefined && out.length > 0) {
      out[0] = intrinsic.surfacePointPosition(tailLoc);
    }
    return out;
  }

  // Both endpoints inserted. Trace from the tail's stored SurfacePoint
  // along the intrinsic halfedge for the full edge length, walking the
  // input mesh. The tangent direction at the tail is the rescaled signpost
  // angle of `segHe`. The trace's first/last point may drift by FP noise
  // from the stored positions; pin both ends to the stored values.
  const tailLoc = intrinsic.insertedVertexLocations.get(tail);
  const tipLoc = intrinsic.insertedVertexLocations.get(tip);
  if (tailLoc === undefined || tipLoc === undefined) {
    // Defensive: shouldn't happen since both vertices were just inserted.
    const pa = tailLoc ? intrinsic.surfacePointPosition(tailLoc) : [0, 0, 0];
    const pb = tipLoc ? intrinsic.surfacePointPosition(tipLoc) : [0, 0, 0];
    return [pa as Vec3, pb as Vec3];
  }
  const angle = intrinsic.insertedTraceAngle(segHe);
  const out = intrinsic.tracePolylineFromSurfacePoint(tailLoc, angle, len);
  if (out.length > 0) out[0] = intrinsic.surfacePointPosition(tailLoc);
  if (out.length > 1) out[out.length - 1] = intrinsic.surfacePointPosition(tipLoc);
  return out;
}

/**
 * Stitch per-segment 3D polylines into a single deduplicated 3D polyline.
 * Drops the first point of each segment if it duplicates the previous
 * segment's last point (within FP tolerance).
 */
function concatPolylines(parts: Vec3[][]): Vec3[] {
  const out: Vec3[] = [];
  for (const part of parts) {
    if (part.length === 0) continue;
    if (out.length === 0) {
      out.push(...part);
    } else {
      const last = out[out.length - 1]!;
      const first = part[0]!;
      const same =
        Math.abs(last[0] - first[0]) < 1e-9 &&
        Math.abs(last[1] - first[1]) < 1e-9 &&
        Math.abs(last[2] - first[2]) < 1e-9;
      const startIdx = same ? 1 : 0;
      for (let i = startIdx; i < part.length; i++) out.push(part[i]!);
    }
  }
  return out;
}

/**
 * High-level convenience: same as {@link flipOutPath} but accepts arbitrary
 * surface points (vertex / edge / face) for the source and destination.
 *
 * Steps:
 *   1. Resolve `src` / `dst` to intrinsic vertex indices, inserting new
 *      intrinsic vertices via L3's `insertVertex_face` / `insertVertex_edge`
 *      if needed. Snapping to an existing vertex happens within {@link SNAP_EPS}.
 *   2. Build a Dijkstra path on the (now updated) intrinsic edge graph.
 *   3. Run FlipOut.
 *   4. Extract a 3D polyline that begins at `src`'s stored 3D position
 *      and ends at `dst`'s stored 3D position, with intermediate
 *      face-crossing points along the geodesic.
 *
 * Returns `{ polyline, length, iterations, converged }` exactly like
 * {@link flipOutPath}.
 */
export function flipOutPathFromSurfacePoints(
  intrinsic: SignpostIntrinsicTriangulation,
  src: SurfacePoint,
  dst: SurfacePoint,
  options: { maxIterations?: number } = {},
): { polyline: Vec3[]; length: number; iterations: number; converged: boolean } {
  // Track which (if any) vertices were freshly inserted so the polyline
  // tracer knows when to use stored 3D positions instead of input-mesh
  // vertex positions.
  const wasInsertedSrc = src.kind !== 'vertex';
  const wasInsertedDst = dst.kind !== 'vertex';

  const vSrc = resolveSurfacePoint(intrinsic, src);
  const vDst = resolveSurfacePoint(intrinsic, dst);

  // After resolution, snapping may have put us on the same vertex — refuse.
  if (vSrc === vDst) {
    throw new Error(
      `flipOutPathFromSurfacePoints: source and destination resolved to the same vertex (${vSrc})`,
    );
  }

  const insertedSrcV = wasInsertedSrc && vSrc >= intrinsic.inputGeometry.mesh.nVertices ? vSrc : -1;
  const insertedDstV = wasInsertedDst && vDst >= intrinsic.inputGeometry.mesh.nVertices ? vDst : -1;

  const initial = shortestEdgePath(intrinsic, vSrc, vDst);
  if (initial === null) {
    throw new Error(
      `flipOutPathFromSurfacePoints: no path from vertex ${vSrc} to vertex ${vDst}`,
    );
  }
  const network = new FlipEdgeNetwork(intrinsic, initial);
  const { iterations, converged } = network.flipOut(options.maxIterations ?? 100000);

  // Per-segment polyline with custom handling of inserted endpoints.
  const segHEs = network.pathHalfedges();
  const parts: Vec3[][] = segHEs.map((he) =>
    traceSegmentPolyline(intrinsic, he, insertedSrcV, insertedDstV),
  );
  const polyline = concatPolylines(parts);
  const length = network.pathLength();
  return { polyline, length, iterations, converged };
}

/**
 * gc's `FlipEdgeNetwork::constructFromPiecewiseDijkstraPath`. Builds an
 * initial path by concatenating per-segment Dijkstra paths between
 * consecutive control vertices, then constructs a `FlipEdgeNetwork`. When
 * `markInterior` is true, every control vertex is marked — the caller can
 * subsequently set `straightenAroundMarkedVertices = false` to pin the path
 * through them (used by bezier subdivision).
 *
 * Returns `null` if any segment is disconnected or has zero length
 * (`vA === vB`), matching gc's null-return contract.
 *
 * Closed loops (gc's `closed` argument) are not supported here — the
 * downstream `FlipEdgeNetwork` handles only open paths.
 */
export function flipEdgeNetworkFromControlPath(
  intrinsic: SignpostIntrinsicTriangulation,
  controlVertices: readonly number[],
  options: { markInterior?: boolean } = {},
): FlipEdgeNetwork | null {
  const { markInterior = false } = options;
  if (controlVertices.length < 2) {
    throw new Error(
      `flipEdgeNetworkFromControlPath: need ≥2 control vertices, got ${controlVertices.length}`,
    );
  }

  const m = intrinsic.intrinsicMesh;
  const halfedges: number[] = [];
  for (let i = 0; i + 1 < controlVertices.length; i++) {
    const vA = controlVertices[i]!;
    const vB = controlVertices[i + 1]!;
    const seg = shortestEdgePath(intrinsic, vA, vB);
    if (seg === null || seg.length === 0) return null;
    for (const he of seg) halfedges.push(he);
  }
  if (halfedges.length === 0) return null;

  // gc's back-and-forth cleanup. Only safe when not marking the interior
  // joins, since collapsing a `[he, he.twin()]` pair removes the visit to
  // their shared middle vertex — which would have been a control point.
  let pathHe: number[];
  if (markInterior) {
    pathHe = halfedges;
  } else {
    pathHe = [];
    for (const he of halfedges) {
      const last = pathHe.length > 0 ? pathHe[pathHe.length - 1]! : -1;
      if (last !== -1 && last === m.twin(he)) {
        pathHe.pop();
      } else {
        pathHe.push(he);
      }
    }
    if (pathHe.length === 0) return null;
  }

  const net = new FlipEdgeNetwork(intrinsic, pathHe);
  if (markInterior) {
    for (const v of controlVertices) net.setMarkedVertex(v, true);
  }
  return net;
}

/**
 * Like {@link flipEdgeNetworkFromControlPath} but accepts arbitrary
 * {@link SurfacePoint}s (vertex / edge / face) as control points. Each
 * non-vertex point is inserted into the intrinsic triangulation as a new
 * vertex (via the same `resolveSurfacePoint` path as
 * {@link flipOutPathFromSurfacePoints}); consecutive points that resolve to
 * the same vertex (e.g. two clicks that snap together) are collapsed.
 *
 * Mutates the intrinsic triangulation by inserting vertices — build a fresh
 * `SignpostIntrinsicTriangulation` per query. Returns `null` if fewer than two
 * distinct control vertices remain or a Dijkstra segment is disconnected.
 */
export function flipEdgeNetworkFromSurfacePointControlPath(
  intrinsic: SignpostIntrinsicTriangulation,
  controlPoints: readonly SurfacePoint[],
  options: { markInterior?: boolean } = {},
): FlipEdgeNetwork | null {
  if (controlPoints.length < 2) {
    throw new Error(
      `flipEdgeNetworkFromSurfacePointControlPath: need ≥2 control points, got ${controlPoints.length}`,
    );
  }
  const controlVertices: number[] = [];
  for (const p of controlPoints) {
    const v = resolveSurfacePoint(intrinsic, p);
    if (controlVertices.length === 0 || controlVertices[controlVertices.length - 1] !== v) {
      controlVertices.push(v);
    }
  }
  if (controlVertices.length < 2) return null;
  return flipEdgeNetworkFromControlPath(intrinsic, controlVertices, options);
}
