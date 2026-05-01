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
//
// Naming & control flow follow gc verbatim: `measureSideAngles`,
// `locallyShortestTestWithType`, `locallyShortenAt`, `iterativeShorten`,
// `addToWedgeAngleQueue`, `replacePathSegment`, `wedgeAngleQueue`. See the
// comments next to each method for the gc function it mirrors.

import type { Vec3 } from '../math/vec3.js';
import type { SignpostIntrinsicTriangulation } from '../intrinsic/signpost-intrinsic-triangulation.js';
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
    return a.angle < b.angle;
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
   * Construct a network from an initial path of intrinsic halfedge indices.
   * Mirrors gc's `FlipEdgeNetwork(...)` + `FlipEdgePath(...)` for the
   * single-open-path case (closed loops + multi-paths are not supported).
   */
  constructor(intrinsic: SignpostIntrinsicTriangulation, initialPath: number[]) {
    this.intrinsic = intrinsic;
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

    const prevPrevId = prev.prevId;
    const nextNextId = curr.nextId;

    // Remove the two old segments.
    this.segments.delete(curr.prevId);
    this.segments.delete(currId);

    // Insert new segments between prevPrev and nextNext.
    let runningPrevId = prevPrevId;
    let firstAddedId = -1;
    for (const newHe of newHalfedges) {
      const newId = this.nextSegId++;
      this.segments.set(newId, { he: newHe, prevId: runningPrevId, nextId: -1 });
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
  flipOut(maxIterations = 100000): { iterations: number; converged: boolean } {
    let iterations = 0;

    while (this.wedgeAngleQueue.size() > 0) {
      if (iterations >= maxIterations) return { iterations, converged: false };
      const top = this.wedgeAngleQueue.pop()!;

      const seg = this.segments.get(top.segId);
      if (seg === undefined) continue; // segment was deleted (stale)
      if (seg.prevId === -1) continue;

      // Re-check the angle now: it may have changed since enqueue.
      const prev = this.segments.get(seg.prevId)!;
      const { type: currType, angle: currAngle } = this.locallyShortestTestWithType(prev.he, seg.he);
      if (currType === SegmentAngleType.Shortest) continue; // already straight
      if (currType !== top.type) {
        // The minority side's type changed (typically because the other side
        // became smaller after a neighbouring flip). Re-enqueue with current
        // type; we'll reprocess later.
        this.addToWedgeAngleQueue(top.segId);
        continue;
      }
      if (Math.abs(currAngle - top.angle) > 1e-12 * Math.max(1, top.angle)) {
        // Angle drifted (numerical or due to a neighbouring flip). Re-enqueue
        // with the up-to-date angle and skip this stale entry.
        this.addToWedgeAngleQueue(top.segId);
        continue;
      }

      // Perform the locally-shorten move.
      this.locallyShortenAt(top.segId, top.type);
      iterations++;
    }
    return { iterations, converged: true };
  }

  // ============================================================================
  // Polyline extraction.
  //
  // Mirrors gc's `getPathPolyline` -> `traceIntrinsicHalfedgeAlongInput` for
  // each path halfedge. We use L3's `tracePolylineFromVertex` (added for L4).
  // ============================================================================

  /**
   * Extract a 3D polyline by tracing each path segment across the input
   * mesh and concatenating the per-segment polylines (de-duplicating
   * shared endpoints).
   */
  extractPolyline(): Vec3[] {
    const out: Vec3[] = [];
    const im = this.intrinsic.intrinsicMesh;

    for (const seg of this.iterPath()) {
      const tail = im.vertex(seg.he);
      const len = this.intrinsic.edgeLengths[im.edge(seg.he)]!;
      // Convert raw signpost angle to rescaled tangent angle (the form
      // `tracePolylineFromVertex` expects).
      const rawAngle = this.intrinsic.halfedgeSignposts[seg.he]!;
      const rescaledAngle = rawAngle / this.intrinsic.vertexAngleScaling(tail);
      const seg3D = this.intrinsic.tracePolylineFromVertex(tail, rescaledAngle, len);

      if (seg3D.length === 0) continue;
      if (out.length === 0) {
        out.push(...seg3D);
      } else {
        // Drop the first point of `seg3D` if it coincides with the last
        // point of `out` (shared endpoint between consecutive traces).
        const last = out[out.length - 1]!;
        const first = seg3D[0]!;
        const samePoint =
          Math.abs(last[0] - first[0]) < 1e-9 &&
          Math.abs(last[1] - first[1]) < 1e-9 &&
          Math.abs(last[2] - first[2]) < 1e-9;
        if (samePoint) {
          for (let i = 1; i < seg3D.length; i++) out.push(seg3D[i]!);
        } else {
          out.push(...seg3D);
        }
      }
    }
    return out;
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
