// Ported from geometry-central:
//   include/geometrycentral/surface/surface_mesh.h
//   include/geometrycentral/surface/surface_mesh.ipp
//   include/geometrycentral/surface/manifold_surface_mesh.h
//   src/surface/surface_mesh.cpp                       (flip(Edge), helpers)
//   src/surface/manifold_surface_mesh.cpp              (constructor from polygons)
//
// L1 — manifold half-edge mesh.
//
// Conventions
// -----------
// We follow geometry-central's *implicit-twin* convention used by
// `ManifoldSurfaceMesh`:
//
//   twin(he) = he ^ 1
//   edge(he) = he >> 1
//   firstHalfedge(e) = 2 * e
//
// `vertex(he)` returns the **tail** of `he` (i.e. the vertex that `he` emanates
// *from*). This matches `Halfedge::vertex() == tailVertex()` in
// geometry-central's `halfedge_element_types.ipp`.
//
//   tailVertex(he) === vertex(he)         // origin
//   tipVertex(he)  === vertex(next(he))   // destination
//
// Around each face, halfedges are ordered CCW via `next`. So for a triangle
// face `(a,b,c)` walked CCW, `next(he_ab) = he_bc`, `next(he_bc) = he_ca`,
// `next(he_ca) = he_ab`, and the three tails are `a, b, c` respectively.
//
// Boundary handling
// -----------------
// FlipOut and the intrinsic-triangulation layers built on top of this typically
// operate on closed surfaces. We diverge from geometry-central's "boundary loop
// face" trick: for every halfedge on the boundary (i.e. with no incident
// interior face on its left), `face(he) === INVALID_INDEX === -1` and
// `next(he)` is wired up around the boundary loop so iteration still
// terminates. The twin of a boundary halfedge is its companion boundary
// halfedge and points the opposite direction along the same edge.
//
// Storage
// -------
// All connectivity is stored in `Int32Array`s keyed by halfedge / vertex /
// face index. We use `Int32Array` (not `Uint32Array`) so that `-1` round-trips
// cleanly as the invalid sentinel without bit-fiddling.

import type { Vec3 } from '../math/vec3.js';

/** Sentinel for "no such index". Stored as -1 in `Int32Array`s. */
export const INVALID_INDEX = -1;

/** Indexed-face list element type. */
export type Triangle = readonly [number, number, number];

/**
 * Manifold half-edge mesh.
 *
 * The mesh is built once from an indexed triangle face list; the only public
 * mutation is `flipEdge(e)`. Iteration helpers are generators over numeric
 * indices and never allocate beyond the iteration state itself.
 */
export class SurfaceMesh {
  // ---------------------------------------------------------------------------
  // Connectivity arrays. Held by reference and grown in place by the
  // vertex-insertion mutations (`splitEdgeTriangular`, `splitFace`). The
  // edge-flip mutation only mutates entries — it never grows the arrays.
  // ---------------------------------------------------------------------------

  /** Tail (origin) vertex of each halfedge. Length: `nHalfedges`. */
  private heVertexArr: Int32Array;
  /** Next halfedge in face cycle (CCW). Length: `nHalfedges`. */
  private heNextArr: Int32Array;
  /** Face on the left of each halfedge, or `INVALID_INDEX` on boundary. */
  private heFaceArr: Int32Array;
  /** Some outgoing halfedge for each vertex. Length: `nVertices`. */
  private vHalfedgeArr: Int32Array;
  /** Some halfedge bordering each face. Length: `nFaces`. */
  private fHalfedgeArr: Int32Array;

  private _nVertices: number;
  private _nFaces: number;
  private _nHalfedges: number;

  private constructor(
    heVertexArr: Int32Array,
    heNextArr: Int32Array,
    heFaceArr: Int32Array,
    vHalfedgeArr: Int32Array,
    fHalfedgeArr: Int32Array,
    nVertices: number,
    nFaces: number,
    nHalfedges: number,
  ) {
    this.heVertexArr = heVertexArr;
    this.heNextArr = heNextArr;
    this.heFaceArr = heFaceArr;
    this.vHalfedgeArr = vHalfedgeArr;
    this.fHalfedgeArr = fHalfedgeArr;
    this._nVertices = nVertices;
    this._nFaces = nFaces;
    this._nHalfedges = nHalfedges;
  }

  // ---------------------------------------------------------------------------
  // Element counts. χ = V - E + F.
  // ---------------------------------------------------------------------------

  /** Number of vertices. */
  get nVertices(): number {
    return this._nVertices;
  }

  /** Number of triangular faces (does *not* include boundary loops). */
  get nFaces(): number {
    return this._nFaces;
  }

  /** Number of halfedges (interior + boundary). */
  get nHalfedges(): number {
    return this._nHalfedges;
  }

  /** Number of undirected edges. With implicit twin pairing: `nHalfedges / 2`. */
  get nEdges(): number {
    return this._nHalfedges >> 1;
  }

  /** Euler characteristic `V - E + F`. */
  get eulerCharacteristic(): number {
    return this._nVertices - this.nEdges + this._nFaces;
  }

  // ---------------------------------------------------------------------------
  // Implicit-twin / edge accessors. These are inlined hot loops below; we also
  // expose them so callers can peek at the layout without going through the
  // iteration helpers.
  // ---------------------------------------------------------------------------

  /** Twin of `he`. `twin(twin(he)) === he` always. */
  twin(he: number): number {
    return he ^ 1;
  }

  /** Edge index for `he`. Pairs `(2k, 2k+1)` share edge `k`. */
  edge(he: number): number {
    return he >> 1;
  }

  /** Canonical halfedge of edge `e` (the even-indexed one). */
  edgeHalfedge(e: number): number {
    return e * 2;
  }

  /** Tail vertex of `he` (origin; geometry-central's `vertex()` / `tailVertex()`). */
  vertex(he: number): number {
    const v = this.heVertexArr[he];
    if (v === undefined) {
      throw new RangeError(`halfedge ${he} out of range [0, ${this._nHalfedges})`);
    }
    return v;
  }

  /** Alias: tail (origin) vertex. */
  tailVertex(he: number): number {
    return this.vertex(he);
  }

  /** Tip (destination) vertex of `he`. Equal to `vertex(next(he))` in a face. */
  tipVertex(he: number): number {
    return this.vertex(this.twin(he));
  }

  /** Next halfedge around the face on `he`'s left. */
  next(he: number): number {
    const n = this.heNextArr[he];
    if (n === undefined) {
      throw new RangeError(`halfedge ${he} out of range [0, ${this._nHalfedges})`);
    }
    return n;
  }

  /** Face on the left of `he`. `INVALID_INDEX` for boundary halfedges. */
  face(he: number): number {
    const f = this.heFaceArr[he];
    if (f === undefined) {
      throw new RangeError(`halfedge ${he} out of range [0, ${this._nHalfedges})`);
    }
    return f;
  }

  /** Some outgoing halfedge of vertex `v`. `vertex(vertexHalfedge(v)) === v`. */
  vertexHalfedge(v: number): number {
    const h = this.vHalfedgeArr[v];
    if (h === undefined) {
      throw new RangeError(`vertex ${v} out of range [0, ${this._nVertices})`);
    }
    return h;
  }

  /** Some halfedge of face `f`. `face(faceHalfedge(f)) === f`. */
  faceHalfedge(f: number): number {
    const h = this.fHalfedgeArr[f];
    if (h === undefined) {
      throw new RangeError(`face ${f} out of range [0, ${this._nFaces})`);
    }
    return h;
  }

  /** Whether halfedge `he` is on the boundary (no incident interior face). */
  isBoundaryHalfedge(he: number): boolean {
    return this.face(he) === INVALID_INDEX;
  }

  /** Whether edge `e` is on the boundary (one of its halfedges is exterior). */
  isBoundaryEdge(e: number): boolean {
    const h = this.edgeHalfedge(e);
    return this.isBoundaryHalfedge(h) || this.isBoundaryHalfedge(this.twin(h));
  }

  /** Whether vertex `v` lies on the mesh boundary. */
  isBoundaryVertex(v: number): boolean {
    for (const he of this.outgoingHalfedges(v)) {
      if (this.isBoundaryHalfedge(he)) return true;
    }
    return false;
  }

  // ---------------------------------------------------------------------------
  // Iteration helpers.
  //
  // All return numeric indices (halfedge / vertex / face / edge). Generators
  // are implemented imperatively (no array allocation in the hot loop).
  //
  // Iteration direction matches geometry-central's signpost-construction
  // walk in `signpost_intrinsic_triangulation.cpp` (the constructor steps via
  // `currHe.next().next().twin()`). For triangle meshes that simplifies to
  //   step(he) = twin(next(next(he)))   ≡  prev(he).twin()
  // which advances CCW around `tailVertex(he)` to the next outgoing halfedge.
  // The reverse step (the immediate CW predecessor in this iteration order,
  // a.k.a. gc's `cwHe = he.twin().next()`) is `next(twin(he))`. This pairing
  // lets gc's signpost formulas translate verbatim into L3.
  //
  // For boundary vertices `vertexHalfedge(v)` is the FIRST INTERIOR outgoing
  // halfedge in this CCW arc (matching gc's "v.halfedge() begins a ccw arc
  // along the interior" convention — see comment near `flipEdgeIfPossible`
  // in `signpost_intrinsic_triangulation.cpp`). Iteration ends when the
  // single boundary outgoing halfedge is reached (the CCW step from a
  // boundary halfedge would leave the vertex fan, so we stop there).
  // ---------------------------------------------------------------------------

  /**
   * Outgoing halfedges of `v`, walking CCW around the vertex via
   * `twin(next(next(currHe)))` (≡ `prev(currHe).twin()` for triangles).
   * Always emits at least one halfedge.
   *
   * Boundary handling: for a boundary vertex the iteration yields all
   * interior outgoing halfedges (in CCW order starting from the first
   * interior), then the single boundary outgoing halfedge, then stops.
   */
  *outgoingHalfedges(v: number): Generator<number, void, void> {
    const first = this.vertexHalfedge(v);
    let curr = first;
    do {
      yield curr;
      // CCW step from a boundary halfedge leaves the vertex fan (it walks
      // around the boundary loop instead). Stop here — every vertex has at
      // most one boundary outgoing halfedge, and gc's iteration terminates
      // at it implicitly by virtue of the constructor's `break` clause.
      if (this.heFaceArr[curr] === INVALID_INDEX) return;
      curr = this.heNextArr[this.heNextArr[curr]!]! ^ 1;
    } while (curr !== first);
  }

  /**
   * Incoming halfedges of `v`. Equivalent to `twin(outgoingHalfedge(v))`,
   * so iterates in gc's CCW order.
   */
  *incomingHalfedges(v: number): Generator<number, void, void> {
    for (const he of this.outgoingHalfedges(v)) {
      yield this.twin(he);
    }
  }

  /**
   * Neighbours of `v`, in gc's CCW order. Each neighbour appears once on a
   * manifold mesh.
   */
  *verticesAroundVertex(v: number): Generator<number, void, void> {
    for (const he of this.outgoingHalfedges(v)) {
      yield this.tipVertex(he);
    }
  }

  /**
   * Edges incident to `v`, in gc's CCW order. Each edge appears once on a
   * manifold mesh.
   */
  *edgesAroundVertex(v: number): Generator<number, void, void> {
    for (const he of this.outgoingHalfedges(v)) {
      yield this.edge(he);
    }
  }

  /**
   * Halfedges of face `f`, in the CCW order induced by `next`. Always yields
   * three for triangle meshes built via `fromFaces`.
   */
  *halfedgesAroundFace(f: number): Generator<number, void, void> {
    const first = this.faceHalfedge(f);
    let curr = first;
    do {
      yield curr;
      curr = this.heNextArr[curr]!;
    } while (curr !== first);
  }

  /** Three vertex indices of a triangular face, in CCW order. */
  *verticesOfFace(f: number): Generator<number, void, void> {
    for (const he of this.halfedgesAroundFace(f)) {
      yield this.vertex(he);
    }
  }

  /** Degree (valence) of `v`: number of incident edges. */
  vertexDegree(v: number): number {
    let k = 0;
    for (const _ of this.outgoingHalfedges(v)) k++;
    return k;
  }

  /** Degree of `f`: number of halfedges around the face. 3 for triangles. */
  faceDegree(f: number): number {
    let k = 0;
    for (const _ of this.halfedgesAroundFace(f)) k++;
    return k;
  }

  // ---------------------------------------------------------------------------
  // Construction.
  //
  // Mirrors `ManifoldSurfaceMesh::ManifoldSurfaceMesh(polygons)` from
  // `manifold_surface_mesh.cpp`. We restrict to triangles since FlipOut's
  // domain is triangle meshes; supporting general polygons would be a small
  // extension but adds boundary/edge bookkeeping we don't need yet.
  // ---------------------------------------------------------------------------

  /**
   * Build a `SurfaceMesh` from an indexed triangle list and a vertex count.
   *
   * Throws on:
   *   - face containing a self-edge (e.g. `[v, v, w]`)
   *   - vertex index out of `[0, vertexCount)`
   *   - edge with more than two incident faces (non-manifold edge)
   *   - vertex with disconnected face fans (non-manifold vertex / "hourglass")
   *   - boundary that visits a vertex more than once
   *
   * Disconnected components are allowed (each component must itself be
   * manifold).
   *
   * @param faces    indexed triangle list, each `[a, b, c]` is a face wound CCW
   * @param vertexCount number of vertices (max index in `faces` must be < this)
   */
  static fromFaces(faces: ArrayLike<Triangle>, vertexCount: number): SurfaceMesh {
    const nFaces = faces.length;
    if (vertexCount < 0 || !Number.isInteger(vertexCount)) {
      throw new RangeError(`vertexCount must be a non-negative integer, got ${vertexCount}`);
    }

    // Validate face indices and detect self-edges.
    for (let iFace = 0; iFace < nFaces; iFace++) {
      const tri = faces[iFace]!;
      const [a, b, c] = tri;
      if (a < 0 || a >= vertexCount || b < 0 || b >= vertexCount || c < 0 || c >= vertexCount) {
        throw new RangeError(
          `face ${iFace} ${JSON.stringify(tri)} has vertex index outside [0, ${vertexCount})`,
        );
      }
      if (a === b || b === c || c === a) {
        throw new Error(
          `face ${iFace} ${JSON.stringify(tri)} contains a self-edge (repeated vertex)`,
        );
      }
    }

    // Pass 1: assign each directed edge (tail, tip) a halfedge index and pair
    // it with its twin under the implicit `^1` rule. We use a Map keyed by
    // `tail * vertexCount + tip` (cheap perfect hash for the pair).
    //
    // Twin pairing rule (matches ManifoldSurfaceMesh constructor): when we see
    // a directed pair (a, b) for the first time and its mirror (b, a) hasn't
    // been seen, allocate a fresh edge — both halfedges of that edge get
    // assigned. If we later see (a, b) again, that's a non-manifold edge:
    // error.
    const directedToHe = new Map<number, number>();
    const heVertexArr: number[] = []; // grown to nHalfedges

    let nHalfedgesSoFar = 0;

    /** Return halfedge index for directed pair (tail, tip), allocating if new. */
    const getOrAllocHe = (tail: number, tip: number, faceIdx: number): number => {
      const key = tail * vertexCount + tip;
      const existing = directedToHe.get(key);
      if (existing !== undefined) {
        // Already seen this directed pair → another face is using the same
        // (a, b) orientation, which means the two faces have inconsistent
        // winding *and* would make this a non-manifold (>2 faces per edge)
        // or duplicate-face error.
        throw new Error(
          `non-manifold or inconsistently-wound input: directed edge (${tail} -> ${tip}) ` +
            `appears in face ${faceIdx} and was already claimed by an earlier face`,
        );
      }
      const twinKey = tip * vertexCount + tail;
      const twinHe = directedToHe.get(twinKey);
      let myHe: number;
      if (twinHe !== undefined) {
        // Twin exists; pair up by `^1`.
        myHe = twinHe ^ 1;
        // Defensive: the slot should not yet have been claimed.
        if (directedToHe.has(key)) {
          throw new Error(`internal: halfedge slot for (${tail}, ${tip}) already claimed`);
        }
      } else {
        // Fresh edge: allocate two consecutive halfedge slots.
        myHe = nHalfedgesSoFar;
        nHalfedgesSoFar += 2;
        heVertexArr.push(tail, tip); // myHe and myHe^1
      }
      directedToHe.set(key, myHe);
      return myHe;
    };

    // Walk each face, allocating halfedges. We also collect the per-face
    // halfedge triple so we can wire `next` and `face` arrays in pass 2.
    const faceHalfedges: Int32Array = new Int32Array(nFaces * 3);
    for (let iFace = 0; iFace < nFaces; iFace++) {
      const [a, b, c] = faces[iFace]!;
      faceHalfedges[iFace * 3 + 0] = getOrAllocHe(a, b, iFace);
      faceHalfedges[iFace * 3 + 1] = getOrAllocHe(b, c, iFace);
      faceHalfedges[iFace * 3 + 2] = getOrAllocHe(c, a, iFace);
    }

    const nHalfedges = nHalfedgesSoFar;

    // Allocate connectivity arrays.
    const heNextArr = new Int32Array(nHalfedges);
    const heFaceArr = new Int32Array(nHalfedges);
    const vHalfedgeArr = new Int32Array(vertexCount);
    const fHalfedgeArr = new Int32Array(nFaces);
    heFaceArr.fill(INVALID_INDEX);
    heNextArr.fill(INVALID_INDEX);
    vHalfedgeArr.fill(INVALID_INDEX);
    fHalfedgeArr.fill(INVALID_INDEX);

    // Pass 2: wire interior `next`, set face pointers, set vertex pointers.
    for (let iFace = 0; iFace < nFaces; iFace++) {
      const h0 = faceHalfedges[iFace * 3 + 0]!;
      const h1 = faceHalfedges[iFace * 3 + 1]!;
      const h2 = faceHalfedges[iFace * 3 + 2]!;

      heNextArr[h0] = h1;
      heNextArr[h1] = h2;
      heNextArr[h2] = h0;
      heFaceArr[h0] = iFace;
      heFaceArr[h1] = iFace;
      heFaceArr[h2] = iFace;
      fHalfedgeArr[iFace] = h0;

      // vHalfedge: any outgoing halfedge for the tail vertex of each. We may
      // overwrite — the boundary fix-up below ensures boundary vertices end up
      // with a boundary-interior halfedge, matching geometry-central.
      vHalfedgeArr[heVertexArr[h0]!] = h0;
      vHalfedgeArr[heVertexArr[h1]!] = h1;
      vHalfedgeArr[heVertexArr[h2]!] = h2;
    }

    // Verify every halfedge actually has a vertex (i.e. allocation didn't
    // leave a phantom). Should be impossible; here as a contract check.
    for (let i = 0; i < nHalfedges; i++) {
      if (heVertexArr[i] === undefined) {
        throw new Error(`internal: heVertex[${i}] was never set`);
      }
    }

    // Pass 3: stitch up boundary halfedges. A halfedge has no face iff its
    // twin's directed pair was never claimed by any face. Each such halfedge
    // is part of a 1D boundary loop running around a hole in the surface.
    //
    // Boundary halfedges obey the standard halfedge invariant: for boundary
    // `b`, `tail(next(b)) === tip(b)`. On a manifold boundary, each boundary
    // vertex `v` is the tail of *exactly one* boundary halfedge (and the tip
    // of exactly one other). So once we know `tip(b)` we know `next(b)`
    // uniquely: it's the boundary outgoing halfedge from `tip(b)`.
    //
    // Strategy: build a `nextBoundaryFromTail[v] = b` map by scanning all
    // boundary halfedges once. If we find two boundary halfedges with the
    // same tail, the input has a vertex appearing in more than one boundary
    // loop (or a non-manifold boundary) and we throw.

    const nextBoundaryFromTail = new Int32Array(vertexCount);
    nextBoundaryFromTail.fill(INVALID_INDEX);

    for (let iHe = 0; iHe < nHalfedges; iHe++) {
      if (heFaceArr[iHe] !== INVALID_INDEX) continue;
      const tail = heVertexArr[iHe]!;
      if (nextBoundaryFromTail[tail] !== INVALID_INDEX) {
        throw new Error(
          `non-manifold boundary: vertex ${tail} is the tail of more than one boundary ` +
            `halfedge (boundary loops touch the vertex more than once)`,
        );
      }
      nextBoundaryFromTail[tail] = iHe;
    }

    for (let iHe = 0; iHe < nHalfedges; iHe++) {
      if (heFaceArr[iHe] !== INVALID_INDEX) continue;
      const tip = heVertexArr[iHe ^ 1]!;
      const nxt = nextBoundaryFromTail[tip]!;
      if (nxt === INVALID_INDEX) {
        throw new Error(
          `internal: boundary halfedge ${iHe} has tip ${tip} but no boundary halfedge starts there`,
        );
      }
      heNextArr[iHe] = nxt;
    }

    // Boundary fix-up: ensure `vHalfedgeArr[v]` for boundary vertex `v` is
    // the FIRST INTERIOR outgoing halfedge in the CCW arc starting from the
    // boundary — matching geometry-central's "v.halfedge() begins a ccw arc
    // along the interior" convention.
    //
    // For each boundary halfedge `b` with tail = u and tip = w, `b` is the
    // boundary halfedge entering vertex `w` along the boundary loop (gc's
    // `b_in` for `w`). Its twin is the interior halfedge of the same edge,
    // outgoing from `w`, and lives in the first interior face encountered
    // when sweeping CCW from the boundary at `w`. So `vHalfedge[w] = twin(b)`
    // gives the desired anchor.
    for (let iHe = 0; iHe < nHalfedges; iHe++) {
      if (heFaceArr[iHe] !== INVALID_INDEX) continue;
      const tip = heVertexArr[iHe ^ 1]!;
      vHalfedgeArr[tip] = iHe ^ 1;
    }

    // Pass 4: manifoldness check. Walk the outgoing halfedges around each
    // vertex (using the same CCW step the iterator uses,
    // `twin(next(next(curr)))`); we should visit every halfedge exactly
    // once when summed across all vertices. Boundary vertices terminate
    // their walk at the (single) boundary halfedge.
    const halfedgeSeen = new Uint8Array(nHalfedges);
    for (let iV = 0; iV < vertexCount; iV++) {
      const start = vHalfedgeArr[iV];
      if (start === undefined || start === INVALID_INDEX) {
        // Vertex not referenced by any face — check if it's actually unused.
        // Geometry-central rejects unreferenced vertices but we accept them
        // as long as `vertexCount` was supplied explicitly.
        continue;
      }
      let curr = start;
      let safety = 0;
      do {
        if (halfedgeSeen[curr]) {
          throw new Error(
            `non-manifold input: revisited halfedge ${curr} while orbiting vertex ${iV}`,
          );
        }
        halfedgeSeen[curr] = 1;
        if (heVertexArr[curr] !== iV) {
          throw new Error(
            `internal: outgoing-halfedge orbit at vertex ${iV} produced halfedge ${curr} ` +
              `with tail ${heVertexArr[curr]}`,
          );
        }
        // Boundary halfedge: end of CCW arc at `iV`.
        if (heFaceArr[curr] === INVALID_INDEX) break;
        curr = heNextArr[heNextArr[curr]!]! ^ 1;
        if (++safety > nHalfedges) {
          throw new Error(`internal: outgoing-halfedge orbit at vertex ${iV} did not terminate`);
        }
      } while (curr !== start);
    }
    for (let iHe = 0; iHe < nHalfedges; iHe++) {
      if (!halfedgeSeen[iHe]) {
        throw new Error(
          `non-manifold input: halfedge ${iHe} (vertex ${heVertexArr[iHe]}) is not reachable ` +
            `from its tail's vHalfedge orbit (vertex has disconnected fans / "hourglass")`,
        );
      }
    }

    return new SurfaceMesh(
      Int32Array.from(heVertexArr),
      heNextArr,
      heFaceArr,
      vHalfedgeArr,
      fHalfedgeArr,
      vertexCount,
      nFaces,
      nHalfedges,
    );
  }

  // ---------------------------------------------------------------------------
  // Mutation: edge flip.
  //
  // Direct port of `SurfaceMesh::flip(Edge eFlip, bool preventSelfEdges)` from
  // `src/surface/surface_mesh.cpp`. We always set `preventSelfEdges = true`:
  // FlipOut depends on the resulting mesh remaining simplicial.
  // ---------------------------------------------------------------------------

  /**
   * In-place combinatorial flip of edge `e`. Returns `false` (and leaves the
   * mesh untouched) if the flip is illegal:
   *   - `e` is on the boundary
   *   - either incident face is non-triangular
   *   - the two opposite vertices are already directly connected by another
   *     edge (would create a duplicate edge / non-simplicial mesh)
   *
   * If the edge can be flipped, the four halfedges and two faces are rewired
   * in place. Edge index `e` and the four halfedge indices on the diamond are
   * preserved (only their `next`/`vertex`/`face` slots change).
   *
   * @returns `true` on success, `false` if illegal
   */
  flipEdge(e: number): boolean {
    if (e < 0 || e >= this.nEdges) {
      throw new RangeError(`edge ${e} out of range [0, ${this.nEdges})`);
    }

    // Geometry-central names: ha1/ha2/ha3 around face A; hb1/hb2/hb3 around B.
    // ha1 and hb1 are the two halfedges of the edge.
    const ha1 = this.edgeHalfedge(e);
    const hb1 = this.twin(ha1);

    if (this.isBoundaryHalfedge(ha1) || this.isBoundaryHalfedge(hb1)) return false;

    const ha2 = this.heNextArr[ha1]!;
    const ha3 = this.heNextArr[ha2]!;
    if (this.heNextArr[ha3] !== ha1) return false; // face A not a triangle

    const hb2 = this.heNextArr[hb1]!;
    const hb3 = this.heNextArr[hb2]!;
    if (this.heNextArr[hb3] !== hb1) return false; // face B not a triangle

    // Vertices: va = ha1.vertex, vb = hb1.vertex (current edge endpoints),
    //           vc = ha3.vertex, vd = hb3.vertex (opposite vertices).
    const va = this.heVertexArr[ha1]!;
    const vb = this.heVertexArr[hb1]!;
    const vc = this.heVertexArr[ha3]!;
    const vd = this.heVertexArr[hb3]!;

    // Don't allow degenerate flips that incident on a degree-1 vertex.
    if (ha2 === hb1 || hb2 === ha1) return false;

    // Prevent creating a duplicate edge: vc and vd already connected.
    for (const n of this.verticesAroundVertex(vc)) {
      if (n === vd) return false;
    }

    const fa = this.heFaceArr[ha1]!;
    const fb = this.heFaceArr[hb1]!;

    // Update vertex outgoing pointers if they pointed at the edge being
    // flipped — those halfedges will no longer originate at va/vb.
    if (this.vHalfedgeArr[va] === ha1) this.vHalfedgeArr[va] = hb2;
    if (this.vHalfedgeArr[vb] === hb1) this.vHalfedgeArr[vb] = ha2;
    // (vc and vd cannot be invalidated by the flip — they each gain an
    //  outgoing halfedge, but their existing one is unaffected.)

    // Face pointers: faces still bordered by ha1 and hb1 respectively.
    this.fHalfedgeArr[fa] = ha1;
    this.fHalfedgeArr[fb] = hb1;

    // Halfedge `next` pointers — rewire the two triangles.
    // After flip, face A = (vc, vd, va) walked as ha1 → hb3 → ha2 → ha1
    // and face B = (vd, vc, vb) walked as hb1 → ha3 → hb2 → hb1.
    this.heNextArr[ha1] = hb3;
    this.heNextArr[hb3] = ha2;
    this.heNextArr[ha2] = ha1;
    this.heNextArr[hb1] = ha3;
    this.heNextArr[ha3] = hb2;
    this.heNextArr[hb2] = hb1;

    // Halfedge tail vertices: ha1 now starts at vc, hb1 now starts at vd.
    // (ha2, ha3, hb2, hb3 keep their tails.)
    this.heVertexArr[ha1] = vc;
    this.heVertexArr[hb1] = vd;

    // Halfedge face owners: ha3 moves from face A → face B; hb3 moves the
    // other way. ha1, ha2 stay in fa; hb1, hb2 stay in fb.
    this.heFaceArr[ha3] = fb;
    this.heFaceArr[hb3] = fa;

    return true;
  }

  // ---------------------------------------------------------------------------
  // Mutations: vertex insertion. Ported from geometry-central:
  //   src/surface/manifold_surface_mesh.cpp::splitEdgeTriangular
  //   src/surface/manifold_surface_mesh.cpp::insertVertexAlongEdge
  //   src/surface/manifold_surface_mesh.cpp::connectVertices
  //   src/surface/manifold_surface_mesh.cpp::insertVertex
  //
  // These add new vertices, edges, faces, and halfedges to an existing mesh
  // by growing the underlying typed arrays in place. The combinatorial
  // bookkeeping mirrors gc's primitives; the L3 layer (signposts, lengths)
  // is responsible for filling in geometric data for the new elements.
  //
  // All new vertex/edge/face/halfedge indices are appended to the end of
  // their respective ranges (no reuse of holes — gc's mesh has a "compress"
  // path for that, but our array layout is dense so we only ever append).
  // ---------------------------------------------------------------------------

  /**
   * Grow `heVertexArr` / `heNextArr` / `heFaceArr` by `extra` halfedge slots.
   * Extra slots are initialised to `INVALID_INDEX`.
   */
  private growHalfedgeArrays(extra: number): void {
    const newLen = this._nHalfedges + extra;
    const newV = new Int32Array(newLen);
    const newN = new Int32Array(newLen);
    const newF = new Int32Array(newLen);
    newV.set(this.heVertexArr);
    newN.set(this.heNextArr);
    newF.set(this.heFaceArr);
    for (let i = this._nHalfedges; i < newLen; i++) {
      newV[i] = INVALID_INDEX;
      newN[i] = INVALID_INDEX;
      newF[i] = INVALID_INDEX;
    }
    this.heVertexArr = newV;
    this.heNextArr = newN;
    this.heFaceArr = newF;
    this._nHalfedges = newLen;
  }

  /** Grow `vHalfedgeArr` by 1 vertex slot, returning its new index. */
  private addVertex(): number {
    const v = this._nVertices;
    const grown = new Int32Array(v + 1);
    grown.set(this.vHalfedgeArr);
    grown[v] = INVALID_INDEX;
    this.vHalfedgeArr = grown;
    this._nVertices = v + 1;
    return v;
  }

  /** Grow `fHalfedgeArr` by 1 face slot, returning its new index. */
  private addFace(): number {
    const f = this._nFaces;
    const grown = new Int32Array(f + 1);
    grown.set(this.fHalfedgeArr);
    grown[f] = INVALID_INDEX;
    this.fHalfedgeArr = grown;
    this._nFaces = f + 1;
    return f;
  }

  /**
   * Allocate a fresh edge — i.e. two paired halfedges (he, he^1). Returns the
   * even halfedge id; the odd id is `+ 1`. Mirrors `getNewEdgeTriple` from gc:
   * we grow the halfedge arrays by 2 so `he ^ 1` lands on the partner.
   */
  private addEdge(): number {
    const he = this._nHalfedges;
    this.growHalfedgeArrays(2);
    return he;
  }

  /**
   * In-place combinatorial split of edge `e`. Both incident faces (or one,
   * if `e` is a boundary edge) must be triangles. Inserts a new vertex
   * subdividing `e`, then connects it to the opposite vertex of each
   * triangle, producing two (or one) new triangles per side.
   *
   * Direct port of `ManifoldSurfaceMesh::splitEdgeTriangular`
   * (`manifold_surface_mesh.cpp`):
   *
   *     Halfedge splitEdgeTriangular(Edge e) {
   *       Halfedge he = insertVertexAlongEdge(e);
   *       { Halfedge heOther = he.next().next(); connectVertices(he, heOther); }
   *       if (he.twin().isInterior()) {
   *         Halfedge heFirst = he.twin().next();
   *         Halfedge heOther = heFirst.next().next();
   *         connectVertices(heFirst, heOther);
   *       }
   *       return he;
   *     }
   *
   * Net change for an interior edge:
   *   +1 vertex, +3 edges, +2 faces, +6 halfedges
   * (2 from `insertVertexAlongEdge`, 2 per `connectVertices` call × 2 sides)
   *
   * Net change for a boundary edge:
   *   +1 vertex, +2 edges, +1 face, +4 halfedges.
   *
   * Returns the new vertex index plus the four (or three) new outgoing
   * halfedges from the new vertex (in CCW order around it).
   */
  splitEdgeTriangular(e: number): { newVertex: number; newHalfedgesFromNew: number[] } {
    if (e < 0 || e >= this.nEdges) {
      throw new RangeError(`edge ${e} out of range [0, ${this.nEdges})`);
    }

    const heACenter = this.edgeHalfedge(e);
    const heBCenter = this.twin(heACenter);
    const isBoundaryE = this.isBoundaryEdge(e);
    const isInteriorA = !this.isBoundaryHalfedge(heACenter);
    const isInteriorB = !this.isBoundaryHalfedge(heBCenter);

    // Triangularity check (only on the interior side(s)).
    if (isInteriorA) {
      const fa = this.heFaceArr[heACenter]!;
      if (this.faceDegree(fa) !== 3) {
        throw new Error(`splitEdgeTriangular: face ${fa} of edge ${e} is not a triangle`);
      }
    }
    if (isInteriorB) {
      const fb = this.heFaceArr[heBCenter]!;
      if (this.faceDegree(fb) !== 3) {
        throw new Error(`splitEdgeTriangular: face ${fb} of edge ${e} is not a triangle`);
      }
    }

    // ----- Phase 1: insertVertexAlongEdge ------------------------------------
    // Mirrors gc's `insertVertexAlongEdge`. Splits the edge into two by
    // inserting a vertex and one new edge; old half-edges keep their slots
    // but their tail/next get rewired so the original `heACenter` now starts
    // at the new vertex, and a fresh `heANew` (with twin `heBNew`) starts at
    // the original tail.

    // Save references before we mutate anything.
    const heANext = this.heNextArr[heACenter]!;
    const heBNext = this.heNextArr[heBCenter]!;
    // heAPrev = predecessor of heACenter inside its face cycle. For a
    // triangle with halfedges (heACenter, heANext, heAPrev), heAPrev =
    // next(heANext).
    let heAPrev = -1;
    if (isInteriorA) heAPrev = this.heNextArr[heANext]!;
    const fA = isInteriorA ? this.heFaceArr[heACenter]! : INVALID_INDEX;
    const fB = isInteriorB ? this.heFaceArr[heBCenter]! : INVALID_INDEX;
    const oldVBottom = this.heVertexArr[heACenter]!;
    const oldVBottomHe = this.vHalfedgeArr[oldVBottom]!;

    // Allocate new vertex + edge.
    const newV = this.addVertex();
    const heANew = this.addEdge();
    const heBNew = heANew ^ 1;

    // New vertex's outgoing halfedge anchor: heACenter (which after the
    // rewire starts at newV). For boundary case where heBCenter is the
    // boundary halfedge entering newV from the other side, gc's L1
    // boundary fix-up wants vHalfedge[boundaryVertex] = first interior
    // outgoing — and `heACenter` (which is the interior side, since `e`
    // is on the boundary => `heACenter` is interior, `heBCenter` is the
    // boundary) is exactly that.
    this.vHalfedgeArr[newV] = heACenter;

    // heANew: same tail as before (oldVBottom), face fA, next = heACenter.
    this.heVertexArr[heANew] = oldVBottom;
    this.heNextArr[heANew] = heACenter;
    this.heFaceArr[heANew] = fA;

    // heBNew: tail = newV, face fB, next = heBNext.
    this.heVertexArr[heBNew] = newV;
    this.heNextArr[heBNew] = heBNext;
    this.heFaceArr[heBNew] = fB;

    // Fix old halfedges:
    //   - heBCenter's next is now heBNew.
    //   - heAPrev's next is now heANew (only if interior side A).
    //   - heACenter's tail is now newV.
    this.heNextArr[heBCenter] = heBNew;
    if (isInteriorA) this.heNextArr[heAPrev] = heANew;
    this.heVertexArr[heACenter] = newV;

    // Preserve oldVBottom's vertexHalfedge invariant. If it was pointing at
    // heACenter (which now starts at newV), reroute to heANew.
    if (oldVBottomHe === heACenter) {
      this.vHalfedgeArr[oldVBottom] = heANew;
    }

    // ----- Phase 2: connectVertices on each interior side --------------------
    // Each call adds 1 edge + 1 face and rewires within the (now-quad)
    // face to produce two triangles.

    let newHeOnSideA = -1; // halfedge from newV toward the opposite vertex on side A
    let newHeOnSideB = -1; // halfedge from newV toward the opposite vertex on side B
    void heANext; void heBNext; // referenced only via cycle walk below

    if (isInteriorA) {
      // After phase 1, the face fA has cycle: heACenter -> heANext -> heAPrev -> heANew -> heACenter.
      // heOther = he.next().next() = heAPrev (in this cycle): start of heACenter is newV,
      // start of heAPrev is the opposite (apex) vertex. So `connectVertices(heACenter, heAPrev)`
      // creates the diagonal newV -> apex.
      const heOther = this.heNextArr[this.heNextArr[heACenter]!]!;
      const created = this.connectVertices_(heACenter, heOther, fA);
      newHeOnSideA = created;
    }

    if (isInteriorB) {
      // gc:
      //   heFirst = he.twin().next()       // where he = heACenter (returned from insertVertexAlongEdge)
      //   heOther = heFirst.next().next()
      //   connectVertices(heFirst, heOther)
      //
      // After phase 1, heBCenter's next was rewired to heBNew, so
      // `heACenter.twin().next() = heBCenter.next() = heBNew`. Thus
      // heFirst = heBNew, whose tail is newV. heOther = heFirst.next().next()
      // which lands on the boundary halfedge of face fB whose tail is the
      // apex (the third vertex of the original triangle face fB).
      const heFirst = heBNew;
      const heOther = this.heNextArr[this.heNextArr[heFirst]!]!;
      const created = this.connectVertices_(heFirst, heOther, fB);
      // `created` runs from heFirst.tail = newV outward, exactly what we want.
      newHeOnSideB = created;
    }

    // ----- Phase 3: collect outgoing halfedges from newV ---------------------
    // Iterate around newV using the CCW step. Should produce 4 halfedges
    // (interior case) or 3 (boundary case: 2 interior + 1 boundary).
    const outgoing: number[] = [];
    {
      const first = this.vHalfedgeArr[newV]!;
      let curr = first;
      let safety = 0;
      do {
        outgoing.push(curr);
        if (this.heFaceArr[curr] === INVALID_INDEX) break;
        curr = this.heNextArr[this.heNextArr[curr]!]! ^ 1;
        if (++safety > 32) {
          throw new Error('splitEdgeTriangular: vertex orbit at new vertex did not terminate');
        }
      } while (curr !== first);
    }

    void newHeOnSideA;
    void newHeOnSideB;
    void isBoundaryE;

    return { newVertex: newV, newHalfedgesFromNew: outgoing };
  }

  /**
   * Internal `connectVertices(heA, heB, fA)` — direct port of
   * `ManifoldSurfaceMesh::connectVertices`. Both halfedges must lie inside
   * the same face `fA`, must not be adjacent in the face cycle, and must
   * differ. Adds a new edge with halfedges (heANew, heBNew) plus a new
   * face fB; halfedges along the cycle from heA to heBNew become bordered
   * by fB.
   *
   * Returns the new halfedge `heANew` (whose tail is `vertex(heA)` and
   * whose face is fA).
   */
  private connectVertices_(heA: number, heB: number, fA: number): number {
    const vA = this.heVertexArr[heA]!;
    const vB = this.heVertexArr[heB]!;

    // Predecessors in the face cycle (they will rewire to point at the new edge).
    // For a quad cycle, prev = next(next(next(...))). We just walk forward.
    const heAPrev = this.findPrevInFace(heA);
    const heBPrev = this.findPrevInFace(heB);

    const heANew = this.addEdge();
    const heBNew = heANew ^ 1;
    const fB = this.addFace();

    // Faces
    this.fHalfedgeArr[fA] = heANew;
    this.fHalfedgeArr[fB] = heBNew;

    // New halfedges
    this.heVertexArr[heANew] = vA;
    this.heNextArr[heANew] = heB;
    this.heFaceArr[heANew] = fA;

    this.heVertexArr[heBNew] = vB;
    this.heNextArr[heBNew] = heA;
    this.heFaceArr[heBNew] = fB;

    // Stitch into existing cycles
    this.heNextArr[heAPrev] = heANew;
    this.heNextArr[heBPrev] = heBNew;

    // Reassign face owner for the half of the cycle now bounded by fB:
    // walk from heA forward until we reach heBNew.
    let curr = heA;
    let safety = 0;
    while (curr !== heBNew) {
      this.heFaceArr[curr] = fB;
      curr = this.heNextArr[curr]!;
      if (++safety > 32) {
        throw new Error('connectVertices: face cycle walk did not terminate');
      }
    }

    return heANew;
  }

  /** Predecessor of `he` along its face's `next` cycle. O(faceDegree). */
  private findPrevInFace(he: number): number {
    let curr = he;
    let safety = 0;
    while (true) {
      const n = this.heNextArr[curr]!;
      if (n === he) return curr;
      curr = n;
      if (++safety > 64) {
        throw new Error(`findPrevInFace: cycle from ${he} did not close`);
      }
    }
  }

  /**
   * In-place combinatorial face-vertex insertion. The face must be a
   * triangle (gc's general routine works on any polygon, but we only need
   * the triangle case for L3). Inserts a new vertex inside `f` and replaces
   * `f` with three triangles fanning from the new vertex to the three
   * existing corners.
   *
   * Direct port of `ManifoldSurfaceMesh::insertVertex` for the triangle case
   * (`manifold_surface_mesh.cpp`).
   *
   * Net change: +1 vertex, +3 edges, +2 faces (3 new minus 1 old), +6 halfedges.
   *
   * Returns the new vertex index plus the three outgoing halfedges from the
   * new vertex (in CCW order around it).
   */
  splitFace(f: number): { newVertex: number; newHalfedgesFromNew: number[] } {
    if (f < 0 || f >= this.nFaces) {
      throw new RangeError(`face ${f} out of range [0, ${this.nFaces})`);
    }
    if (this.faceDegree(f) !== 3) {
      throw new Error(`splitFace: face ${f} is not a triangle`);
    }

    // Gather the three boundary halfedges of `f` BEFORE mutating anything.
    const h0 = this.faceHalfedge(f);
    const h1 = this.heNextArr[h0]!;
    const h2 = this.heNextArr[h1]!;
    const boundary = [h0, h1, h2];

    // Allocate the center vertex.
    const centerV = this.addVertex();

    // Allocate three new edges (each `addEdge` grows by two halfedges):
    //   leadingHalfedges[i] points TOWARD the center from the tip of boundary[i]
    //   trailingHalfedges[i] points OUTWARD from the center to vertex(boundary[i])
    // Convention (matches gc): leadingHalfedges[i].twin() = trailingHalfedges[(i+1) % 3]
    // because trailing[(i+1)%3] starts at the same vertex (tip of boundary[i] =
    // tail of boundary[(i+1)%3]).
    const leading: number[] = [];
    const trailing: number[] = new Array(3);
    for (let i = 0; i < 3; i++) {
      const heL = this.addEdge();
      leading.push(heL);
      trailing[(i + 1) % 3] = heL ^ 1;
    }

    // Allocate two new faces (the third re-uses `f` as in gc's insertVertex).
    const innerFaces = [f, this.addFace(), this.addFace()];

    // Wire up.
    for (let i = 0; i < 3; i++) {
      const fi = innerFaces[i]!;
      const leadingHe = leading[i]!;
      const trailingHe = trailing[i]!;
      const boundaryHe = boundary[i]!;

      // Face anchor: any of the three halfedges of fi will do.
      this.fHalfedgeArr[fi] = boundaryHe;

      // leadingHe: tail = tip of boundaryHe = vertex(boundary[(i+1)%3]),
      //            face = fi, next = trailingHe.
      this.heVertexArr[leadingHe] = this.heVertexArr[boundary[(i + 1) % 3]!]!;
      this.heNextArr[leadingHe] = trailingHe;
      this.heFaceArr[leadingHe] = fi;

      // trailingHe: tail = centerV, face = fi, next = boundaryHe.
      this.heVertexArr[trailingHe] = centerV;
      this.heNextArr[trailingHe] = boundaryHe;
      this.heFaceArr[trailingHe] = fi;

      // boundaryHe: face = fi, next = leadingHe.
      this.heFaceArr[boundaryHe] = fi;
      this.heNextArr[boundaryHe] = leadingHe;
    }

    // Center vertex anchor: any of its trailing halfedges. Use trailing[0]
    // (interior — splitFace operates on interior faces only).
    this.vHalfedgeArr[centerV] = trailing[0]!;

    // Outgoing halfedges from the new vertex are the three trailings.
    // Iterate to make sure they come out in CCW order matching the existing
    // vertex orbit convention.
    const outgoing: number[] = [];
    {
      const first = this.vHalfedgeArr[centerV]!;
      let curr = first;
      let safety = 0;
      do {
        outgoing.push(curr);
        curr = this.heNextArr[this.heNextArr[curr]!]! ^ 1;
        if (++safety > 16) {
          throw new Error('splitFace: vertex orbit at new vertex did not terminate');
        }
      } while (curr !== first);
    }

    return { newVertex: centerV, newHalfedgesFromNew: outgoing };
  }
}

// ---------------------------------------------------------------------------
// Utility constructors that take a flat positions array. These don't belong on
// the class (the mesh holds *no* geometry — that's L2's job) but they're handy
// for tests and L2 plumbing. We re-export the connectivity factory only.
// ---------------------------------------------------------------------------

/**
 * Convenience: build a mesh from a positions list (Vec3[]) and a triangle
 * list. Discards the positions and just calls `fromFaces`. Provided so test
 * helpers can keep `{ vertices, faces }` records together without each call
 * site needing to drop the vertex array.
 */
export function meshFromPositionsAndFaces(
  vertices: ArrayLike<Vec3>,
  faces: ArrayLike<Triangle>,
): SurfaceMesh {
  return SurfaceMesh.fromFaces(faces, vertices.length);
}
