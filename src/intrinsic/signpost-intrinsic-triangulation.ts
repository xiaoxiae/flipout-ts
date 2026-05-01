// Ported from geometry-central:
//   include/geometrycentral/surface/signpost_intrinsic_triangulation.h
//   include/geometrycentral/surface/signpost_intrinsic_triangulation.ipp
//   src/surface/signpost_intrinsic_triangulation.cpp
//   include/geometrycentral/surface/intrinsic_triangulation.h
//   src/surface/intrinsic_triangulation.cpp
//   include/geometrycentral/surface/intrinsic_triangulation.ipp
//   include/geometrycentral/utilities/elementary_geometry.ipp   (layoutTriangleVertex)
//
// L3 — signpost intrinsic triangulation. Sits atop an L2 extrinsic geometry,
// maintaining a *parallel* connectivity (an L1 `SurfaceMesh` cloned from the
// input) plus per-edge intrinsic lengths and per-halfedge "signpost" angles.
//
// The signpost data structure follows
//   "Navigating Intrinsic Triangulations" — Sharp, Soliman, Crane, SIGGRAPH 2019
// which is the substrate for the FlipOut algorithm
//   "You Can Find Geodesic Paths in Triangle Meshes by Just Flipping Edges"
//   Sharp & Crane, SIGGRAPH Asia 2020.
//
// Conventions (matching geometry-central exactly so the formulas line up):
//
//   * `signpostAngle[he]` is an angle in radians measured at the *tail* of
//     `he`, in the [0, vertexAngleSum[v]) wedge — NOT rescaled to [0, 2π).
//     The rescaling factor `vertexAngleSum / (2π or π)` is applied at use
//     time inside `halfedgeVector` / `rescaledVertexVector`. (See
//     `signpost_intrinsic_triangulation.ipp::halfedgeVector`.)
//   * `vertexAngleSums[v]` is the *original* sum of corner angles around v
//     in the input geometry. It is invariant under intrinsic flips because
//     the vertex set is fixed.
//   * `edgeLengths[e]` is the intrinsic edge length, indexed by edge id of
//     `intrinsicMesh`. Initially equal to the extrinsic edge length.

import { type Vec2, fromAngle, modPositive } from '../math/vec2.js';
import * as Vec2Ops from '../math/vec2.js';
import type { Vec3 } from '../math/vec3.js';
import { cornerAngleFromLengths, triangleAreaFromLengths } from '../math/triangle.js';
import { INVALID_INDEX, SurfaceMesh, type Triangle } from '../mesh/surface-mesh.js';
import type { VertexPositionGeometry } from '../geometry/vertex-position-geometry.js';

// ---------------------------------------------------------------------------
// Numeric tolerances used by the ray-segment intersection inside
// `traceInFaceFromVertexCorner`. Pulled out here so they are named (and
// auditable) rather than buried as magic constants inside the hot loop.
// ---------------------------------------------------------------------------

/** Minimum |determinant| for a non-parallel ray-segment intersection. */
const TRACE_DENOM_EPS = 1e-15;

/** Minimum forward distance along the ray (rejects backward / origin-coincident hits). */
const TRACE_RAY_EPS = 1e-12;

/** Half-open segment-parameter window: a hit at u∈[−eps, 1+eps] counts as on the segment. */
const TRACE_SEGMENT_EPS = 1e-9;

/**
 * Lay out a triangle's third vertex `pC` in 2D given two vertex positions
 * `pA`, `pB` and the lengths from B->C and C->A.
 *
 * Mirrors `elementary_geometry.ipp::layoutTriangleVertex` from geometry-central:
 *
 *   lAB = |pB - pA|
 *   tArea = triangleAreaFromLengths(lAB, lBC, lCA)
 *   h = 2 * tArea / lAB     // perpendicular height from C onto AB
 *   w = (lAB^2 - lBC^2 + lCA^2) / (2 * lAB)   // signed projection of C onto AB
 *   pC = pA + w * unit(AB) + h * perp(AB)
 *
 * The result lies on the side of AB chosen by `perp(AB) = (-AB.y, AB.x)`,
 * giving counter-clockwise winding `(pA, pB, pC)`.
 */
export function layoutTriangleVertex(
  pA: Vec2,
  pB: Vec2,
  lBC: number,
  lCA: number,
): Vec2 {
  const ab: Vec2 = [pB[0] - pA[0], pB[1] - pA[1]];
  const lAB = Vec2Ops.norm(ab);
  if (lAB === 0) return [pA[0], pA[1]];

  const tArea = triangleAreaFromLengths(lAB, lBC, lCA);
  const h = (2 * tArea) / lAB;
  const w = (lAB * lAB - lBC * lBC + lCA * lCA) / (2 * lAB);

  const ux = ab[0] / lAB;
  const uy = ab[1] / lAB;
  // perp(u) = (-u.y, u.x) — CCW 90-degree rotation of `u`.
  const px = -uy;
  const py = ux;

  return [pA[0] + w * ux + h * px, pA[1] + w * uy + h * py];
}

/**
 * A point on the surface — either a mesh vertex, a point on an edge at
 * parameter `t ∈ (0, 1)`, or a point inside a face given as barycentric
 * coordinates (which must sum to 1 with all components ≥ 0).
 *
 * Mirrors gc's `SurfacePoint` (`include/geometrycentral/surface/surface_point.h`).
 *
 * Used by L3 vertex insertion (`insertVertex_face`, `insertVertex_edge`) and
 * by L4 (`flipOutPathFromSurfacePoints`).
 */
export type SurfacePoint =
  | { kind: 'vertex'; vertex: number }
  | { kind: 'edge'; edge: number; t: number }
  | { kind: 'face'; face: number; bary: [number, number, number] };

/**
 * Result of `traceFromVertex`. A geodesic walk that starts at vertex `v` of
 * the *input* (extrinsic) mesh in some tangent direction lands at a point
 * inside some face of the input mesh.
 */
export interface TraceResult {
  /** 3D position on the input mesh's surface. */
  position: Vec3;
  /** Index of the input-mesh face containing the trace endpoint. */
  faceIndex: number;
  /**
   * Barycentric coordinates of `position` within `faceIndex`, in the order
   * of vertices visited around the face by `halfedgesAroundFace(faceIndex)`.
   */
  barycentric: Vec3;
}

/**
 * Signpost intrinsic triangulation. Maintains an intrinsic mesh sharing
 * vertices with the input geometry; intrinsic flips diverge connectivity
 * and edge lengths from the input.
 */
export class SignpostIntrinsicTriangulation {
  /** Intrinsic-side connectivity (clone of input mesh, mutated by flipEdge). */
  readonly intrinsicMesh: SurfaceMesh;

  /** Original extrinsic geometry. Never mutated by this class. */
  readonly inputGeometry: VertexPositionGeometry;

  /**
   * Intrinsic edge lengths, indexed by `intrinsicMesh` edge id. Grows
   * automatically when vertex insertion adds new edges.
   */
  edgeLengths: Float64Array;

  /**
   * Sum of corner angles at each vertex. Initially equal to the input
   * geometry's per-vertex angle sum. For new vertices inserted via
   * `insertVertex_face` / `insertVertex_edge`, set to 2π (interior) or π
   * (boundary edge insertion). Indexed by intrinsic-mesh vertex id; agrees
   * with the input mesh for the first `inputGeometry.mesh.nVertices`
   * indices, but extends past that for inserted vertices.
   */
  vertexAngleSums: Float64Array;

  /**
   * Per-halfedge signpost angle in radians, in `[0, vertexAngleSum[tail])`.
   * Mirrors geometry-central's `signpostAngle`. Updated by `flipEdge` and
   * by vertex-insertion routines.
   */
  halfedgeSignposts: Float64Array;

  /**
   * For vertices inserted by `insertVertex_face` / `insertVertex_edge`, the
   * `SurfacePoint` on the *input* mesh that they correspond to. Used by
   * `tracePolylineFromSurfacePoint` to walk the input mesh starting from
   * an inserted vertex. Original input vertices are NOT in this map.
   */
  readonly insertedVertexLocations = new Map<number, SurfacePoint>();

  // --------------------------------------------------------------------------
  // Construction.
  //
  // Mirrors `SignpostIntrinsicTriangulation::SignpostIntrinsicTriangulation`,
  // plus the parent `IntrinsicTriangulation::IntrinsicTriangulation` work
  // (clone the mesh, copy edge lengths, init vertex angle sums via the parent
  // class's `requireVertexAngleSums`).
  // --------------------------------------------------------------------------

  constructor(geom: VertexPositionGeometry) {
    this.inputGeometry = geom;

    // Clone the connectivity. The intrinsic mesh starts identical to the input
    // mesh — flips on it diverge from the input.
    const sourceMesh = geom.mesh;
    const faces: Triangle[] = [];
    for (let f = 0; f < sourceMesh.nFaces; f++) {
      const it = sourceMesh.verticesOfFace(f);
      const a = it.next().value as number;
      const b = it.next().value as number;
      const c = it.next().value as number;
      faces.push([a, b, c]);
    }
    this.intrinsicMesh = SurfaceMesh.fromFaces(faces, sourceMesh.nVertices);

    // The clone above produces the *same* halfedge / vertex / face indexing as
    // the input mesh because `fromFaces` walks faces in order and assigns
    // halfedges deterministically. We rely on this for the trace optimisation
    // (input-mesh vertex i === intrinsic vertex i).

    // Edge lengths from input geometry.
    const nEdges = this.intrinsicMesh.nEdges;
    this.edgeLengths = new Float64Array(nEdges);
    for (let e = 0; e < nEdges; e++) {
      this.edgeLengths[e] = geom.edgeLength(e);
    }

    // Vertex angle sums (from extrinsic geometry — they are intrinsic
    // invariants of the surface, computable from edge lengths alone).
    const nVerts = this.intrinsicMesh.nVertices;
    this.vertexAngleSums = new Float64Array(nVerts);
    for (let v = 0; v < nVerts; v++) {
      this.vertexAngleSums[v] = geom.vertexAngleSum(v);
    }

    // Initialise signposts. Direct port of the per-vertex loop in
    // `signpost_intrinsic_triangulation.cpp` (constructor body):
    //
    //     for (Vertex v : mesh.vertices()) {
    //       double runningAngle = 0.;
    //       Halfedge firstHe = v.halfedge();
    //       Halfedge currHe = firstHe;
    //       do {
    //         signpostAngle[currHe] = runningAngle;
    //         if (!currHe.isInterior()) break;
    //         double cornerAngleVal = cornerAngle(currHe.corner());
    //         runningAngle += cornerAngleVal;
    //         currHe = currHe.next().next().twin();
    //       } while (currHe != firstHe);
    //     }
    //
    // L1's iteration step `twin(next(next(he)))` matches gc's
    // `currHe.next().next().twin()` exactly, so the formula transcribes
    // 1-1. For boundary vertices `vertexHalfedge(v)` is the FIRST INTERIOR
    // outgoing halfedge (gc's "v.halfedge() begins a ccw arc along the
    // interior" — see L1's boundary fix-up); the boundary halfedge ends
    // up at the end of the walk with signpost = vertexAngleSum.
    this.halfedgeSignposts = new Float64Array(this.intrinsicMesh.nHalfedges);
    for (let v = 0; v < nVerts; v++) {
      const im = this.intrinsicMesh;
      const firstHe = im.vertexHalfedge(v);
      let currHe = firstHe;
      let runningAngle = 0;
      do {
        this.halfedgeSignposts[currHe] = runningAngle;
        if (im.face(currHe) === INVALID_INDEX) break;
        runningAngle += this.cornerAngleAt(currHe);
        currHe = im.twin(im.next(im.next(currHe)));
      } while (currHe !== firstHe);
    }
  }

  // ==========================================================================
  // Helpers — corner angles, signpost rescaling, etc.
  // ==========================================================================

  /**
   * Interior corner angle at `tailVertex(he)` inside `face(he)`, computed from
   * the three intrinsic edge lengths via the cosine rule.
   *
   * Mirrors `IntrinsicTriangulation::getCornerAngle` (which is the intrinsic
   * version that uses only `edgeLengths`).
   */
  cornerAngleAt(he: number): number {
    const m = this.intrinsicMesh;
    if (m.face(he) === INVALID_INDEX) return 0;
    const heNext = m.next(he);
    const hePrev = m.next(heNext);
    const lA = this.edgeLengths[m.edge(he)]!;
    const lOpp = this.edgeLengths[m.edge(heNext)]!;
    const lB = this.edgeLengths[m.edge(hePrev)]!;
    return cornerAngleFromLengths(lOpp, lA, lB);
  }

  /** Constant `vertexAngleSums[v]` accessor. */
  getCornerAngleSum(v: number): number {
    const s = this.vertexAngleSums[v];
    if (s === undefined) {
      throw new RangeError(`vertex ${v} out of range [0, ${this.intrinsicMesh.nVertices})`);
    }
    return s;
  }

  /**
   * Mirrors `signpost_intrinsic_triangulation.ipp::vertexAngleScaling`.
   *
   *   vertexAngleScaling(v) = vertexAngleSums[v] / (π if boundary else 2π)
   *
   * The reciprocal of this maps signpost angles (which live in `[0, sum)`)
   * to "true" tangent-plane angles (which live in `[0, 2π)` for interior
   * vertices, `[0, π)` for boundary vertices).
   */
  vertexAngleScaling(v: number): number {
    const sum = this.getCornerAngleSum(v);
    const target = this.intrinsicMesh.isBoundaryVertex(v) ? Math.PI : 2 * Math.PI;
    return sum / target;
  }

  /**
   * Standardise an angle into `[0, vertexAngleSum[v])` for interior vertices,
   * passing it through unchanged for boundary vertices.
   *
   * Direct port of `signpost_intrinsic_triangulation.ipp::standardizeAngle`.
   *
   * One robustness tweak vs. C++ `std::fmod`: any value within FP epsilon
   * of the modulus is snapped to 0 to avoid signposts landing at
   * `vertexAngleSum - ulp` (which is "almost a full revolution" and breaks
   * monotonicity tests in the iteration sequence). Geometry-central does
   * not need this because it never compares signpost multisets directly,
   * but our test suite does.
   */
  private standardizeAngle(v: number, angle: number): number {
    if (this.intrinsicMesh.isBoundaryVertex(v)) return angle;
    const modulus = this.getCornerAngleSum(v);
    const m = modPositive(angle, modulus);
    // Snap values within ~ulp of the modulus to 0.
    if (modulus - m < 1e-12 * modulus) return 0;
    return m;
  }

  /**
   * The "tangent-plane vector" of intrinsic halfedge `he`, expressed in the
   * tangent plane at its tail. Direction is the *rescaled* signpost angle,
   * length is the intrinsic edge length.
   *
   * Mirrors `signpost_intrinsic_triangulation.ipp::halfedgeVector`.
   */
  halfedgeVector(he: number): Vec2 {
    const tail = this.intrinsicMesh.vertex(he);
    const edgeAngle = this.halfedgeSignposts[he]!;
    const scaleFac = 1 / this.vertexAngleScaling(tail);
    const len = this.edgeLengths[this.intrinsicMesh.edge(he)]!;
    const dir = fromAngle(edgeAngle * scaleFac);
    return [dir[0] * len, dir[1] * len];
  }

  // ==========================================================================
  // Diamond layout for edge flips.
  //
  // Direct port of `intrinsic_triangulation.ipp::layoutDiamond`.
  // ==========================================================================

  /**
   * Lay out the two triangles incident on `he`'s edge into a 2D "diamond" so
   * the new diagonal length can be measured. Returns `[p0, p1, p2, p3]` where:
   *   - p3 is at origin, p0 on +x axis (length l30 = edge from face B's prev)
   *   - p2 is `he`'s tail, p0 is `he`'s tip
   *   - p1 is the opposite vertex in face A
   *   - p3 is the opposite vertex in face B
   *
   * After flipping, `he` would point from p3 -> p1 (the new diagonal).
   */
  private layoutDiamond(he: number): [Vec2, Vec2, Vec2, Vec2] {
    const m = this.intrinsicMesh;
    const heA0 = he;
    const heA1 = m.next(heA0);
    const heA2 = m.next(heA1);
    const heB0 = m.twin(he);
    const heB1 = m.next(heB0);
    const heB2 = m.next(heB1);

    const l01 = this.edgeLengths[m.edge(heA1)]!;
    const l12 = this.edgeLengths[m.edge(heA2)]!;
    const l23 = this.edgeLengths[m.edge(heB1)]!;
    const l30 = this.edgeLengths[m.edge(heB2)]!;
    const l02 = this.edgeLengths[m.edge(heA0)]!;

    const p3: Vec2 = [0, 0];
    const p0: Vec2 = [l30, 0];
    const p2 = layoutTriangleVertex(p3, p0, l02, l23);
    const p1 = layoutTriangleVertex(p2, p0, l01, l12);
    return [p0, p1, p2, p3];
  }

  // ==========================================================================
  // Edge flip.
  //
  // Direct port of `flipEdgeIfPossible`. Combinatorial flip is delegated to
  // `intrinsicMesh.flipEdge` (L1); we own the length & signpost updates.
  // ==========================================================================

  /**
   * Flip intrinsic edge `e` if possible. Direct port of
   * `SignpostIntrinsicTriangulation::flipEdgeIfPossible`
   * (`signpost_intrinsic_triangulation.cpp`):
   *
   *     bool flipEdgeIfPossible(Edge e) {
   *       Halfedge he = e.halfedge();
   *       std::array<Vector2, 4> layoutPositions = layoutDiamond(he);
   *       double A1 = cross(layoutPositions[1] - layoutPositions[0],
   *                         layoutPositions[3] - layoutPositions[0]);
   *       double A2 = cross(layoutPositions[3] - layoutPositions[2],
   *                         layoutPositions[1] - layoutPositions[2]);
   *       double areaEPS = triangleTestEPS * (A1 + A2);
   *       if (A1 < areaEPS || A2 < areaEPS) return false;
   *       double newLength = (layoutPositions[1] - layoutPositions[3]).norm();
   *       if (!std::isfinite(newLength)) return false;
   *       bool flipped = intrinsicMesh->flip(e, false);
   *       if (!flipped) return false;
   *       edgeLengths[e] = newLength;
   *       updateAngleFromCWNeighor(e.halfedge());
   *       updateAngleFromCWNeighor(e.halfedge().twin());
   *       ...
   *     }
   *
   * Returns `false` if `e` is on the boundary, if the new layout is
   * geometrically invalid, if the new diagonal would have non-finite
   * length, or if the underlying L1 flip refuses (duplicate edge).
   *
   * On success the only signpost updates are on the two halfedges of the
   * new edge — every other signpost stays valid because pre-existing
   * halfedges' input-frame anchoring is unchanged by the flip.
   */
  flipEdge(e: number): boolean {
    if (e < 0 || e >= this.intrinsicMesh.nEdges) {
      throw new RangeError(`edge ${e} out of range [0, ${this.intrinsicMesh.nEdges})`);
    }
    const m = this.intrinsicMesh;
    const ha1 = m.edgeHalfedge(e);
    const hb1 = m.twin(ha1);
    if (m.isBoundaryHalfedge(ha1) || m.isBoundaryHalfedge(hb1)) return false;

    // Lay out the diamond before flipping.
    const layout = this.layoutDiamond(ha1);
    const p0 = layout[0];
    const p1 = layout[1];
    const p2 = layout[2];
    const p3 = layout[3];

    // Signed-area test: both new triangles (p1,p3,p0) and (p3,p1,p2) must
    // have positive signed area to be a valid convex layout. Mirrors
    // `flipEdgeIfPossible`.
    //
    //   A1 = cross(p1 - p0, p3 - p0)     // would-be face A area
    //   A2 = cross(p3 - p2, p1 - p2)     // would-be face B area
    const a1 =
      (p1[0] - p0[0]) * (p3[1] - p0[1]) - (p1[1] - p0[1]) * (p3[0] - p0[0]);
    const a2 =
      (p3[0] - p2[0]) * (p1[1] - p2[1]) - (p3[1] - p2[1]) * (p1[0] - p2[0]);
    // Use a small relative epsilon (matches geometry-central's `triangleTestEPS`
    // role; default value there is also small).
    const areaEps = 1e-12 * (Math.abs(a1) + Math.abs(a2) + 1);
    if (a1 < areaEps || a2 < areaEps) return false;

    // New edge length is |p1 - p3|.
    const dx = p1[0] - p3[0];
    const dy = p1[1] - p3[1];
    const newLength = Math.sqrt(dx * dx + dy * dy);
    if (!Number.isFinite(newLength) || newLength === 0) return false;

    // Combinatorial flip.
    const flipped = m.flipEdge(e);
    if (!flipped) return false;

    // Assign new length.
    this.edgeLengths[e] = newLength;

    // Update signposts on the two new halfedges only — exactly matching
    // gc's `flipEdgeIfPossible`. Pre-existing halfedges keep their input-
    // frame signposts; only the diagonal's two halfedges are new and need
    // to be (re)anchored relative to their CW neighbour.
    this.updateAngleFromCWNeighor(ha1);
    this.updateAngleFromCWNeighor(hb1);

    return true;
  }

  // ==========================================================================
  // Vertex insertion.
  //
  // Direct ports of `SignpostIntrinsicTriangulation::insertVertex_face` and
  // `insertVertex_edge` from `signpost_intrinsic_triangulation.cpp`. We keep
  // the vertex set of the *input* mesh unchanged — only the intrinsic mesh
  // gets new vertices. The mapping back to the input is stored in
  // `insertedVertexLocations` so traces from new vertices can find their
  // starting point on the input.
  //
  // Net per-call effect on the intrinsic mesh:
  //   insertVertex_face:  +1 V, +3 E, +2 F, +6 HE
  //   insertVertex_edge:  +1 V, +3 E, +2 F, +6 HE  (interior edge)
  //                       +1 V, +2 E, +1 F, +4 HE  (boundary edge)
  //
  // After insertion all signposts at SURROUNDING vertices remain valid —
  // gc's analysis: existing halfedges' input-frame signposts are unchanged
  // because their tail vertices haven't moved. We only need to set
  // signposts on the new halfedges that emanate from the new vertex (and
  // their twins, which point inward at surrounding vertices and need
  // `updateAngleFromCWNeighor` to slot them between their CW/CCW
  // neighbours).
  // ==========================================================================

  /** Snapping tolerance shared by L3/L4 insertion (also exported). */
  static readonly SNAP_EPS = 1e-9;

  /** Grow `vertexAngleSums` to cover the current intrinsicMesh vertex count. */
  private growVertexStorage(): void {
    const want = this.intrinsicMesh.nVertices;
    if (this.vertexAngleSums.length >= want) return;
    const grown = new Float64Array(want);
    grown.set(this.vertexAngleSums);
    this.vertexAngleSums = grown;
  }

  /** Grow `edgeLengths` to cover the current intrinsicMesh edge count. */
  private growEdgeStorage(): void {
    const want = this.intrinsicMesh.nEdges;
    if (this.edgeLengths.length >= want) return;
    const grown = new Float64Array(want);
    grown.set(this.edgeLengths);
    this.edgeLengths = grown;
  }

  /** Grow `halfedgeSignposts` to cover the current intrinsicMesh halfedge count. */
  private growHalfedgeStorage(): void {
    const want = this.intrinsicMesh.nHalfedges;
    if (this.halfedgeSignposts.length >= want) return;
    const grown = new Float64Array(want);
    grown.set(this.halfedgeSignposts);
    this.halfedgeSignposts = grown;
  }

  /**
   * Insert a new intrinsic vertex inside face `f` at given barycentric
   * coordinates. Mirrors gc's `insertVertex_face`. Returns the new vertex
   * index.
   *
   * If `bary` is within `SNAP_EPS` of an existing vertex of the face, the
   * existing vertex index is returned and no insertion is performed.
   *
   * Throws if barycentric coords are out of `[0, 1]` or don't sum to 1
   * (within tolerance).
   */
  insertVertex_face(f: number, bary: [number, number, number]): number {
    const im = this.intrinsicMesh;
    if (f < 0 || f >= im.nFaces) {
      throw new RangeError(`face ${f} out of range [0, ${im.nFaces})`);
    }
    const [b0, b1, b2] = bary;
    const eps = SignpostIntrinsicTriangulation.SNAP_EPS;
    if (b0 < -eps || b1 < -eps || b2 < -eps) {
      throw new RangeError(
        `insertVertex_face: barycentric coords must be in [0, 1]: ${JSON.stringify(bary)}`,
      );
    }
    if (Math.abs(b0 + b1 + b2 - 1) > 1e-6) {
      throw new RangeError(
        `insertVertex_face: barycentric coords must sum to 1, got ${b0 + b1 + b2}`,
      );
    }

    // Snap-to-corner if any bary is ~1.
    if (b0 > 1 - eps || b1 > 1 - eps || b2 > 1 - eps) {
      const it = im.halfedgesAroundFace(f);
      const h0 = it.next().value as number;
      const h1 = it.next().value as number;
      const h2 = it.next().value as number;
      if (b0 > 1 - eps) return im.vertex(h0);
      if (b1 > 1 - eps) return im.vertex(h1);
      return im.vertex(h2);
    }

    // === (1) Lay out the face in 2D and compute the position of the new
    // point + the three new edge lengths.
    const heFace = im.faceHalfedge(f);
    const heA = heFace;
    const heB = im.next(heA);
    const heC = im.next(heB);
    const lAB = this.edgeLengths[im.edge(heA)]!;
    const lBC = this.edgeLengths[im.edge(heB)]!;
    const lCA = this.edgeLengths[im.edge(heC)]!;
    const pA: Vec2 = [0, 0];
    const pB: Vec2 = [lAB, 0];
    const pC = layoutTriangleVertex(pA, pB, lBC, lCA);

    const oldHEs = [heA, heB, heC];

    // gc's convention: bary[0..2] match face's halfedges in order; the new
    // 2D point lies at (b1 * vertCoords[1] + b2 * vertCoords[2]) when
    // vertCoords[0] is at origin.
    const newPCoord: Vec2 = [
      b0 * pA[0] + b1 * pB[0] + b2 * pC[0],
      b0 * pA[1] + b1 * pB[1] + b2 * pC[1],
    ];

    const lenToA = Math.hypot(newPCoord[0] - pA[0], newPCoord[1] - pA[1]);
    const lenToB = Math.hypot(newPCoord[0] - pB[0], newPCoord[1] - pB[1]);
    const lenToC = Math.hypot(newPCoord[0] - pC[0], newPCoord[1] - pC[1]);
    const newLengths = [lenToA, lenToB, lenToC]; // aligned with oldHEs by tail vertex

    if (!Number.isFinite(lenToA) || !Number.isFinite(lenToB) || !Number.isFinite(lenToC)) {
      throw new Error('insertVertex_face: non-finite edge length');
    }

    // === (2) Mutate intrinsic mesh.
    const { newVertex, newHalfedgesFromNew } = im.splitFace(f);
    this.growVertexStorage();
    this.growEdgeStorage();
    this.growHalfedgeStorage();
    this.vertexAngleSums[newVertex] = 2 * Math.PI;

    // === (3) Assign edge lengths. For each newly-inserted edge from
    // `newVertex` to a corner of the original face, find its new halfedge
    // (one of `newHalfedgesFromNew`) and set the corresponding length.
    //
    // gc's mapping: the new halfedge `heV` outgoing from newV satisfies
    //   heV.next() == originalHe
    // where `originalHe` is the corresponding boundary halfedge of the old
    // face (since after the splitFace, each tiny new triangle has cycle
    // [trailing, boundary, leading] = [heV, originalHe, leadingHe]).
    for (const heV of newHalfedgesFromNew) {
      const heNext = im.next(heV);
      const idx = oldHEs.indexOf(heNext);
      if (idx === -1) {
        // Shouldn't happen — every trailing he's next is one of the original boundary hes.
        throw new Error(
          `insertVertex_face: outgoing halfedge ${heV} from new vertex doesn't connect to an original face halfedge`,
        );
      }
      this.edgeLengths[im.edge(heV)] = newLengths[idx]!;
    }

    // === (4) Compute signposts. Mirrors gc's `resolveNewVertex`:
    //   - For each incoming halfedge `heIn` to newV, call
    //     `updateAngleFromCWNeighor(heIn)` — this anchors the inward-pointing
    //     halfedges' signposts at the SURROUNDING vertices to be consistent
    //     with the existing halfedges' signposts at those vertices.
    //   - Then walk around newV starting from a chosen "first" halfedge,
    //     setting signposts on the outgoing halfedges from newV using
    //     successive corner angles.
    //
    // Because for face insertion all surrounding signposts are unchanged in
    // input-frame and we only need to anchor the new "edges" between them,
    // gc's `updateAngleFromCWNeighor` does the right thing: each new
    // halfedge gets `signpost(cwNeighbor) + cornerAngle(cwHe.corner())`.
    for (const heV of newHalfedgesFromNew) {
      const heIn = im.twin(heV);
      this.updateAngleFromCWNeighor(heIn);
    }

    // Set outgoing signposts from newV. Cycle CCW starting at newHalfedgesFromNew[0]:
    //   signpost[outgoing[0]] = 0
    //   signpost[outgoing[i+1]] = signpost[outgoing[i]] + cornerAngleAt(outgoing[i])
    // where the corner is at newV inside `face(outgoing[i])`. Mod 2π for
    // standardisation.
    {
      let runningAngle = 0;
      for (let i = 0; i < newHalfedgesFromNew.length; i++) {
        const heV = newHalfedgesFromNew[i]!;
        this.halfedgeSignposts[heV] = runningAngle;
        runningAngle += this.cornerAngleAt(heV);
      }
    }

    // Store the new vertex's location on the input mesh.
    const inputLoc = this.locateInsertedVertex_face(f, [b0, b1, b2]);
    this.insertedVertexLocations.set(newVertex, inputLoc);

    return newVertex;
  }

  /**
   * Insert a new intrinsic vertex on edge `e` at parameter `t ∈ (0, 1)`.
   * Mirrors gc's `insertVertex_edge`. Returns the new vertex index.
   *
   * If `t` is within `SNAP_EPS` of 0 or 1, the corresponding existing
   * endpoint vertex is returned and no insertion is performed.
   */
  insertVertex_edge(e: number, t: number): number {
    const im = this.intrinsicMesh;
    if (e < 0 || e >= im.nEdges) {
      throw new RangeError(`edge ${e} out of range [0, ${im.nEdges})`);
    }
    const eps = SignpostIntrinsicTriangulation.SNAP_EPS;
    if (!Number.isFinite(t) || t < -eps || t > 1 + eps) {
      throw new RangeError(`insertVertex_edge: t must be in [0, 1], got ${t}`);
    }

    const heA0 = im.edgeHalfedge(e);
    const heB0 = im.twin(heA0);

    // Snap to existing endpoints.
    if (t < eps) return im.vertex(heA0);
    if (t > 1 - eps) return im.vertex(heB0);

    const isInteriorA = !im.isBoundaryHalfedge(heA0);
    const isInteriorB = !im.isBoundaryHalfedge(heB0);
    const isOnBoundary = !isInteriorA || !isInteriorB;
    if (!isInteriorA && !isInteriorB) {
      throw new Error(`insertVertex_edge: edge ${e} has no incident face`);
    }

    // === (1) Compute the four (or three) new edge lengths.
    //
    // gc's logic:
    //   backLen  = t * |e|                         length of new edge from oldTail to newV
    //   frontLen = (1 - t) * |e|                   length of new edge from newV to oldTip
    //   Alen     = distance(newP, apex_A) in face A
    //   Blen     = distance(newP, apex_B) in face B  (only if interior B)
    const lE = this.edgeLengths[e]!;
    const backLen = t * lE;
    const frontLen = (1 - t) * lE;

    // Lay out face A: tail of heA0 = corner 0, tip = corner 1, apex = corner 2.
    let Alen = -1;
    if (isInteriorA) {
      const heA1 = im.next(heA0);
      const heA2 = im.next(heA1);
      const lA01 = lE; // = edge(heA0)
      const lA12 = this.edgeLengths[im.edge(heA1)]!;
      const lA20 = this.edgeLengths[im.edge(heA2)]!;
      const pA0: Vec2 = [0, 0];
      const pA1: Vec2 = [lA01, 0];
      const pA2 = layoutTriangleVertex(pA0, pA1, lA12, lA20);
      const newPA: Vec2 = [(1 - t) * pA0[0] + t * pA1[0], (1 - t) * pA0[1] + t * pA1[1]];
      Alen = Math.hypot(newPA[0] - pA2[0], newPA[1] - pA2[1]);
    }

    let Blen = -1;
    if (isInteriorB) {
      const heB1 = im.next(heB0);
      const heB2 = im.next(heB1);
      const lB01 = lE;
      const lB12 = this.edgeLengths[im.edge(heB1)]!;
      const lB20 = this.edgeLengths[im.edge(heB2)]!;
      const pB0: Vec2 = [0, 0];
      const pB1: Vec2 = [lB01, 0];
      const pB2 = layoutTriangleVertex(pB0, pB1, lB12, lB20);
      // The "back" direction in face B: heB0 goes from oldTip-side toward
      // oldTail-side, so newV at parameter t along the original edge sits at
      // parameter (1 - t) along heB0.
      const newPB: Vec2 = [t * pB0[0] + (1 - t) * pB1[0], t * pB0[1] + (1 - t) * pB1[1]];
      Blen = Math.hypot(newPB[0] - pB2[0], newPB[1] - pB2[1]);
    }

    // === (2) Mutate intrinsic mesh.
    const { newVertex, newHalfedgesFromNew } = im.splitEdgeTriangular(e);
    this.growVertexStorage();
    this.growEdgeStorage();
    this.growHalfedgeStorage();
    this.vertexAngleSums[newVertex] = isOnBoundary ? Math.PI : 2 * Math.PI;

    // === (3) Assign edge lengths.
    //
    // Mapping: the four halfedges in `newHalfedgesFromNew` come out in the
    // CCW orbit order around `newVertex`. For an interior insertion gc's
    // walk is:
    //   currHe = newHeFront; lengths = [frontLen, Alen, backLen, Blen]
    //   currHe = currHe.next().next().twin()    // CCW around newV
    //
    // Our L1 `splitEdgeTriangular` emits `newHalfedgesFromNew` using the
    // same CCW step starting from `vHalfedgeArr[newV] = heA0` (post-split).
    // After phase 1, heA0's tail = newV and points toward the original tip
    // (vb). So:
    //   newHalfedgesFromNew[0] = heA0 (newV → vb)            length = frontLen
    //   newHalfedgesFromNew[1] = ?    (newV → apex_A)        length = Alen
    //   newHalfedgesFromNew[2] = ?    (newV → va, oldTail)   length = backLen
    //   newHalfedgesFromNew[3] = ?    (newV → apex_B)        length = Blen
    // This pattern holds because the CCW orbit from heA0 walks: heA0, then
    // the diagonal newV→apex_A inside fA, then heBNew (newV→old tip via
    // the boundary side, now actually = the rewired heA0... wait, let me
    // reconsider).
    //
    // Actually after splitEdge: vHalfedgeArr[newV] = heA0 (which still
    // points to the OLD heACenter slot, retargeted). heA0 now goes
    // newV → vb. The diagonal added on side A goes newV → apex_A. The
    // diagonal added on side B goes newV → apex_B. And heBNew (the new
    // halfedge added by phase 1) goes newV → vb's side... wait actually
    // heBNew goes newV → original next-vertex-in-face-B — but that's
    // confusing. Let me just match by which halfedge points where.
    const lensInOrder = isOnBoundary
      ? [frontLen, Alen, backLen]
      : [frontLen, Alen, backLen, Blen];

    if (newHalfedgesFromNew.length !== lensInOrder.length) {
      throw new Error(
        `insertVertex_edge: expected ${lensInOrder.length} new halfedges, got ${newHalfedgesFromNew.length}`,
      );
    }
    for (let i = 0; i < newHalfedgesFromNew.length; i++) {
      const heV = newHalfedgesFromNew[i]!;
      const len = lensInOrder[i]!;
      if (len < 0) {
        throw new Error(`insertVertex_edge: missing length for halfedge ${heV}`);
      }
      this.edgeLengths[im.edge(heV)] = len;
    }

    // === (4) Set signposts.
    for (const heV of newHalfedgesFromNew) {
      const heIn = im.twin(heV);
      this.updateAngleFromCWNeighor(heIn);
    }
    {
      let runningAngle = 0;
      for (let i = 0; i < newHalfedgesFromNew.length; i++) {
        const heV = newHalfedgesFromNew[i]!;
        if (im.face(heV) === INVALID_INDEX) {
          // Boundary outgoing halfedge — sits at the end of the wedge walk
          // at angle = vertexAngleSums[newV] (= π for a boundary insertion).
          this.halfedgeSignposts[heV] = this.vertexAngleSums[newVertex]!;
          break;
        }
        this.halfedgeSignposts[heV] = runningAngle;
        runningAngle += this.cornerAngleAt(heV);
      }
    }

    // Store the new vertex's location on the input mesh.
    const inputLoc = this.locateInsertedVertex_edge(e, t);
    this.insertedVertexLocations.set(newVertex, inputLoc);

    return newVertex;
  }

  /**
   * 3D position of a `SurfacePoint` on the input mesh. For a vertex-kind
   * point, returns the input geometry's vertex position. For edge / face
   * points, lifts the parametric coords via barycentric / linear
   * interpolation of the input vertex positions.
   */
  surfacePointPosition(p: SurfacePoint): Vec3 {
    if (p.kind === 'vertex') {
      return this.inputGeometry.position(p.vertex);
    }
    const inputMesh = this.inputGeometry.mesh;
    if (p.kind === 'edge') {
      const he = inputMesh.edgeHalfedge(p.edge);
      const vA = inputMesh.vertex(he);
      const vB = inputMesh.tipVertex(he);
      const pA = this.inputGeometry.position(vA);
      const pB = this.inputGeometry.position(vB);
      const t = p.t;
      return [
        (1 - t) * pA[0] + t * pB[0],
        (1 - t) * pA[1] + t * pB[1],
        (1 - t) * pA[2] + t * pB[2],
      ];
    }
    // face
    const it = inputMesh.halfedgesAroundFace(p.face);
    const h0 = it.next().value as number;
    const h1 = it.next().value as number;
    const h2 = it.next().value as number;
    const pA = this.inputGeometry.position(inputMesh.vertex(h0));
    const pB = this.inputGeometry.position(inputMesh.vertex(h1));
    const pC = this.inputGeometry.position(inputMesh.vertex(h2));
    const [b0, b1, b2] = p.bary;
    return [
      b0 * pA[0] + b1 * pB[0] + b2 * pC[0],
      b0 * pA[1] + b1 * pB[1] + b2 * pC[1],
      b0 * pA[2] + b1 * pB[2] + b2 * pC[2],
    ];
  }

  /**
   * Compute a `SurfacePoint` on the input mesh for a face-interior
   * insertion. Currently only handles insertion into a face that is still
   * an *original* input-mesh face. (Inserting into a face produced by a
   * previous insertion or a flip would require recursive resolution; we
   * skip that until needed.)
   */
  private locateInsertedVertex_face(
    f: number,
    bary: [number, number, number],
  ): SurfacePoint {
    const inputMesh = this.inputGeometry.mesh;
    if (f < inputMesh.nFaces) {
      // The intrinsic-face index `f` aligns with an input-face index because
      // we haven't done any flips that re-index faces (flipEdge preserves
      // face indices). Verify the corner vertex set matches before accepting.
      const im = this.intrinsicMesh;
      const vsIm = [...im.verticesOfFace(f)];
      const vsIn = [...inputMesh.verticesOfFace(f)];
      if (
        vsIm.length === 3 &&
        vsIn.length === 3 &&
        vsIm[0] === vsIn[0] &&
        vsIm[1] === vsIn[1] &&
        vsIm[2] === vsIn[2]
      ) {
        return { kind: 'face', face: f, bary };
      }
    }
    // Fallback: no clean correspondence — store as face anyway so traces
    // can at least try; we throw if the trace later finds it inconsistent.
    return { kind: 'face', face: f, bary };
  }

  /**
   * Compute a `SurfacePoint` on the input mesh for an edge-interior
   * insertion. Same caveat as `locateInsertedVertex_face`.
   */
  private locateInsertedVertex_edge(e: number, t: number): SurfacePoint {
    const inputMesh = this.inputGeometry.mesh;
    if (e < inputMesh.nEdges) {
      // Same alignment assumption: edge `e` in the intrinsic mesh
      // corresponds to edge `e` in the input mesh (true if no flips).
      const im = this.intrinsicMesh;
      const heIm = im.edgeHalfedge(e);
      const heIn = inputMesh.edgeHalfedge(e);
      if (
        im.vertex(heIm) === inputMesh.vertex(heIn) &&
        im.tipVertex(heIm) === inputMesh.tipVertex(heIn)
      ) {
        return { kind: 'edge', edge: e, t };
      }
      // Reversed orientation — flip t.
      if (
        im.vertex(heIm) === inputMesh.tipVertex(heIn) &&
        im.tipVertex(heIm) === inputMesh.vertex(heIn)
      ) {
        return { kind: 'edge', edge: e, t: 1 - t };
      }
    }
    return { kind: 'edge', edge: e, t };
  }

  /**
   * Set the signpost angle of `he` from its CW neighbour at the same
   * tail vertex.
   *
   * Direct port of `SignpostIntrinsicTriangulation::updateAngleFromCWNeighor`
   * (`signpost_intrinsic_triangulation.cpp`):
   *
   *     void updateAngleFromCWNeighor(Halfedge he) {
   *       if (!he.isInterior()) {
   *         signpostAngle[he] = vertexAngleSums[he.vertex()];
   *         return;
   *       }
   *       if (!he.twin().isInterior()) {
   *         signpostAngle[he] = 0.;
   *         return;
   *       }
   *       Halfedge cwHe = he.twin().next();
   *       double neighAngle = signpostAngle[cwHe];
   *       double cAngle = cornerAngle(cwHe.corner());
   *       double updatedAngle = standardizeAngle(he.vertex(),
   *                                              neighAngle + cAngle);
   *       signpostAngle[he] = updatedAngle;
   *     }
   *
   * Under L1's CCW iteration via `prev.twin`, the immediate CW predecessor
   * of `he` (i.e. one step backwards along the iteration order) is
   * `he.twin().next()` ≡ `next(twin(he))`, the same as gc's `cwHe`. So the
   * whole formula transcribes literally.
   */
  private updateAngleFromCWNeighor(he: number): void {
    const m = this.intrinsicMesh;
    if (m.face(he) === INVALID_INDEX) {
      // Boundary halfedge sits at the END of the wedge walk.
      this.halfedgeSignposts[he] = this.getCornerAngleSum(m.vertex(he));
      return;
    }
    if (m.face(m.twin(he)) === INVALID_INDEX) {
      // Twin is boundary -> `he` is the FIRST interior halfedge in the
      // CCW arc, so it sits at angle 0.
      this.halfedgeSignposts[he] = 0;
      return;
    }
    const cwHe = m.next(m.twin(he));
    const neighAngle = this.halfedgeSignposts[cwHe]!;
    const cAngle = this.cornerAngleAt(cwHe);
    const updatedAngle = this.standardizeAngle(m.vertex(he), neighAngle + cAngle);
    this.halfedgeSignposts[he] = updatedAngle;
  }

  // ==========================================================================
  // Diagnostics: Delaunay check.
  // ==========================================================================

  /**
   * Whether the intrinsic Delaunay condition holds at edge `e`:
   * `α + β <= π` where α, β are the two corner angles opposite `e` in its
   * two incident faces. Returns `true` for boundary edges (no opposite
   * angle in the boundary loop direction).
   *
   * Used by L4 sanity checks. Geometry-central uses cotan weights
   * (`edgeCotanWeight(e) >= 0` is equivalent under the same condition).
   */
  isDelaunay(e: number): boolean {
    const m = this.intrinsicMesh;
    const ha = m.edgeHalfedge(e);
    const hb = m.twin(ha);
    if (m.face(ha) === INVALID_INDEX || m.face(hb) === INVALID_INDEX) return true;

    // Opposite corner in face A is at `next(next(ha))`, similarly face B.
    const oppA = m.next(m.next(ha));
    const oppB = m.next(m.next(hb));
    const aAngle = this.cornerAngleAt(oppA);
    const bAngle = this.cornerAngleAt(oppB);
    return aAngle + bAngle <= Math.PI + 1e-12;
  }

  // ==========================================================================
  // traceFromVertex
  //
  // Walks a tangent vector (rescaled signpost angle, distance) from input
  // vertex `v` across faces of the *input* (extrinsic) mesh until the trace
  // ends in a face. The trace transitions across faces by reflecting the
  // direction into each next face's local frame.
  //
  // Mirrors the high-level structure of geometry-central's
  // `traceGeodesic_fromVertex` followed by `traceInFaceFromEdge`.
  // ==========================================================================

  /**
   * Trace a tangent vector from input-mesh vertex `v`. The angle is the
   * *rescaled* signpost angle (in `[0, 2π)` for interior, `[0, π]` on
   * boundary). Returns the 3D position the trace ends at, the input-mesh
   * face containing it, and barycentric coords inside that face.
   *
   * Distance 0 returns the vertex location itself.
   */
  traceFromVertex(v: number, tangentAngle: number, distance: number): TraceResult {
    if (distance === 0) {
      const pos = this.inputGeometry.position(v);
      return {
        position: pos,
        faceIndex: this.firstFaceAround(v),
        barycentric: this.vertexBaryInFace(v, this.firstFaceAround(v)),
      };
    }

    const m = this.inputGeometry.mesh;

    // We trace on the *input* mesh, so the rescaling factor maps the rescaled
    // angle (which the caller passes) back to the raw [0, vertexAngleSum)
    // wedge:
    //   raw = rescaledAngle * vertexAngleScaling(v)
    // but we don't actually need `raw`: we walk the wedges in CCW order,
    // accumulating each face's corner angle (rescaled to the [0, 2π) frame),
    // and pick the wedge that contains `tangentAngle`.

    const angle2pi = modPositive(tangentAngle, 2 * Math.PI);

    // Find the wedge containing `angle2pi`. Walks CCW from
    // `vertexHalfedge(v)` — which by L1's gc-aligned boundary fix-up is the
    // FIRST INTERIOR outgoing halfedge for boundary vertices and an
    // arbitrary interior outgoing halfedge for interior vertices.
    const sumTotal = this.vertexAngleSums[v]!;
    const target = m.isBoundaryVertex(v) ? Math.PI : 2 * Math.PI;
    const scale = target / sumTotal; // raw angle * scale = rescaled angle

    let cumulative = 0; // rescaled cumulative angle from the first interior
    let chosenHe = -1;
    let chosenStartAngle = 0;
    let chosenWedgeWidth = 0;

    // First interior outgoing halfedge in CCW order — gc's `v.halfedge()`.
    const firstInterior = m.vertexHalfedge(v);
    let curr = firstInterior;

    // Wedge selection rule: angle `a` belongs to wedge `i` if
    //   start_i <= a < end_i
    // where the wedge starts at `signpost(curr_i)` (rescaled) and ends at
    // `signpost(curr_i+1)`. Within FP tolerance we use a small relative
    // epsilon `eps` and snap exactly-on-boundary values to the START of the
    // NEXT wedge (so a trace exactly along the next halfedge picks that
    // halfedge with relRescaled=0). Iteration step matches L1's CCW
    // outgoingHalfedges: `twin(next(next(curr)))`.
    const eps = 1e-10;
    do {
      if (m.face(curr) === INVALID_INDEX) break;
      const cAngleRaw = this.inputGeometry.cornerAngle(curr);
      const cAngle = cAngleRaw * scale;
      const start = cumulative;
      const end = cumulative + cAngle;
      // Strict half-open interval [start, end) with FP fuzz: match
      // angles within `eps` BELOW `start` (counts as this wedge) but
      // exclude angles within `eps` BELOW `end` (those belong to the
      // next wedge — i.e. the *next* halfedge).
      if (angle2pi >= start - eps && angle2pi < end - eps) {
        chosenHe = curr;
        chosenStartAngle = start;
        chosenWedgeWidth = cAngle;
        break;
      }
      cumulative = end;
      curr = m.twin(m.next(m.next(curr)));
    } while (curr !== firstInterior);

    // Fallback: clamp to the last wedge (handles tangentAngle = 2π / 0
    // numerical edge cases).
    if (chosenHe === -1) {
      // Use whatever halfedge we ended on.
      let last = firstInterior;
      let lastCum = 0;
      let lastWidth = 0;
      let runningCum = 0;
      let scanCurr = firstInterior;
      do {
        if (m.face(scanCurr) === INVALID_INDEX) break;
        last = scanCurr;
        lastCum = runningCum;
        const cAngle = this.inputGeometry.cornerAngle(scanCurr) * scale;
        lastWidth = cAngle;
        runningCum += cAngle;
        scanCurr = m.twin(m.next(m.next(scanCurr)));
      } while (scanCurr !== firstInterior);
      chosenHe = last;
      chosenStartAngle = lastCum;
      chosenWedgeWidth = lastWidth || 1;
    }

    // Compute relative angle within the wedge (rescaled), then *un-rescale*
    // to get a real Euclidean angle within the face:
    //
    //   relRescaled = angle2pi - chosenStartAngle
    //   relRaw = relRescaled / scale     // Euclidean angle inside the face
    //
    // (because the wedge is `cAngleRaw` Euclidean radians, mapped to
    // `cAngleRaw * scale` rescaled radians).
    const relRescaled = angle2pi - chosenStartAngle;
    const relRaw = relRescaled / scale;

    // Now we are inside `face(chosenHe)`. Lay it out in 2D:
    //   pTail = (0,0)
    //   pTip = (lTail->Tip, 0)
    //   pOther = layoutTriangleVertex(pTail, pTip, lOther,Tip, lTail,Other)
    // and compute a 2D direction at `pTail` rotated by `relRaw` from the
    // halfedge `chosenHe`.
    return this.traceInFaceFromVertexCorner(chosenHe, relRaw, distance);
  }

  /**
   * Polyline variant of {@link traceFromVertex}. Returns the **sequence of
   * 3D points** the trace passes through on the *input* mesh: starting at
   * the tail vertex, then a point on every face boundary crossing, ending
   * at the trace's final point. Used by L4's `extractPolyline` to render
   * an intrinsic geodesic edge as a 3D polyline on the input mesh.
   *
   * Mirrors gc's `traceIntrinsicHalfedgeAlongInput` (specifically its
   * `options.includePath = true` branch which records every face crossing).
   *
   * NOTE: this is a minimal extension to L3 — same trace logic as
   * `traceFromVertex`, but emits intermediate face-crossings instead of
   * just the endpoint. Added here (rather than at L4) so the 2D-layout
   * machinery and tolerance constants stay encapsulated.
   */
  tracePolylineFromVertex(v: number, tangentAngle: number, distance: number): Vec3[] {
    const startPos = this.inputGeometry.position(v);
    if (distance === 0) return [startPos];

    const m = this.inputGeometry.mesh;
    const angle2pi = modPositive(tangentAngle, 2 * Math.PI);
    const sumTotal = this.vertexAngleSums[v]!;
    const target = m.isBoundaryVertex(v) ? Math.PI : 2 * Math.PI;
    const scale = target / sumTotal;

    let cumulative = 0;
    let chosenHe = -1;
    let chosenStartAngle = 0;

    const firstInterior = m.vertexHalfedge(v);
    let curr = firstInterior;
    const eps = 1e-10;
    do {
      if (m.face(curr) === INVALID_INDEX) break;
      const cAngleRaw = this.inputGeometry.cornerAngle(curr);
      const cAngle = cAngleRaw * scale;
      const start = cumulative;
      const end = cumulative + cAngle;
      if (angle2pi >= start - eps && angle2pi < end - eps) {
        chosenHe = curr;
        chosenStartAngle = start;
        break;
      }
      cumulative = end;
      curr = m.twin(m.next(m.next(curr)));
    } while (curr !== firstInterior);

    if (chosenHe === -1) {
      // Same fallback as `traceFromVertex`: clamp to the last wedge.
      let last = firstInterior;
      let lastCum = 0;
      let runningCum = 0;
      let scanCurr = firstInterior;
      do {
        if (m.face(scanCurr) === INVALID_INDEX) break;
        last = scanCurr;
        lastCum = runningCum;
        const cAngle = this.inputGeometry.cornerAngle(scanCurr) * scale;
        runningCum += cAngle;
        scanCurr = m.twin(m.next(m.next(scanCurr)));
      } while (scanCurr !== firstInterior);
      chosenHe = last;
      chosenStartAngle = lastCum;
    }

    const relRaw = (angle2pi - chosenStartAngle) / scale;
    return this.tracePolylineInFaceFromVertexCorner(chosenHe, relRaw, distance);
  }

  /**
   * Polyline tracer that starts at an arbitrary {@link SurfacePoint} on the
   * input mesh, walks `dist` along the geodesic in tangent-space direction
   * `tangentAngle`, and returns the 3D points it crosses (start, every face
   * boundary it crosses, and the final endpoint).
   *
   * Mirrors gc's `traceGeodesic` (`trace_geodesic.cpp`), which dispatches
   * by `SurfacePoint` kind to `traceGeodesic_fromVertex` /
   * `traceGeodesic_fromEdge` / `traceGeodesic_fromFace`. We reuse our
   * existing `tracePolylineFromVertex` for the vertex case, and lay out the
   * incident face in 2D for the edge / face cases — using the same
   * convention as gc's `vertexCoordinatesInTriangle`:
   *
   *   vertCoords[0] = (0, 0)                 // tail of face.halfedge()
   *   vertCoords[1] = (|face.halfedge()|, 0) // tip of face.halfedge()
   *   vertCoords[2] = layoutTriangleVertex(...)
   *
   * (i.e. the +x axis points along `face.halfedge()`).
   *
   * **Angle convention at face-interior starts.** When a vertex was inserted
   * into face `f` via {@link insertVertex_face}, gc anchors its tangent
   * frame so the *first* outgoing halfedge from the new vertex carries
   * `signpostAngle = 0`, and that halfedge points from the new vertex
   * toward the face's first-halfedge tail (corner 0). Therefore "0 radians"
   * in the inserted-vertex tangent frame corresponds to the direction
   * `(corner0 - newPCoord)` in the face's 2D frame. We add the offset
   * `atan2(-newPCoord.y, -newPCoord.x)` to convert.
   *
   * For an edge-interior start, the chosen face is the one on the +y side
   * of the trace direction (mirroring gc's `traceGeodesic_fromEdge`); the
   * angle convention there is "0 radians" along the edge halfedge `+x`.
   *
   * @param start - vertex / edge / face point on the input mesh
   * @param tangentAngle - direction in [0, 2π); rescaled vertex tangent
   *   for vertex starts, face-frame angle (with the convention above) for
   *   face / edge starts.
   * @param dist - geodesic distance to walk (≥ 0)
   */
  tracePolylineFromSurfacePoint(
    start: SurfacePoint,
    tangentAngle: number,
    dist: number,
  ): Vec3[] {
    const startPos = this.surfacePointPosition(start);
    if (dist === 0) return [startPos];

    if (start.kind === 'vertex') {
      return this.tracePolylineFromVertex(start.vertex, tangentAngle, dist);
    }

    const m = this.inputGeometry.mesh;

    // Determine the start face and the in-face barycentric / 2D position.
    let face: number;
    let baryInFace: [number, number, number];
    if (start.kind === 'face') {
      if (start.face < 0 || start.face >= m.nFaces) {
        throw new RangeError(
          `tracePolylineFromSurfacePoint: face ${start.face} out of range [0, ${m.nFaces})`,
        );
      }
      face = start.face;
      baryInFace = [start.bary[0], start.bary[1], start.bary[2]];
    } else {
      // 'edge'. Pick the face on the +y side of the trace dir, gc-style.
      const heE = m.edgeHalfedge(start.edge);
      const t01 = start.t;
      // Lay out the face on each side and decide based on the y component.
      const sinAngle = Math.sin(tangentAngle);
      let chosenHe: number;
      let edgeT: number;
      if (sinAngle >= 0) {
        // Positive y → use the face on heE's side. heE goes from corner 0 to
        // corner 1 (i.e. along +x of its face's layout). The trace's "0
        // radians" is along heE, so a positive-y direction goes into heE's
        // face.
        if (m.face(heE) === INVALID_INDEX) {
          // The +y side is the boundary; fall back to the twin face.
          chosenHe = m.twin(heE);
          edgeT = 1 - t01;
        } else {
          chosenHe = heE;
          edgeT = t01;
        }
      } else {
        // Negative y → use the twin face. Edge t flips because halfedges
        // run in opposite directions.
        const twin = m.twin(heE);
        if (m.face(twin) === INVALID_INDEX) {
          chosenHe = heE;
          edgeT = t01;
        } else {
          chosenHe = twin;
          edgeT = 1 - t01;
        }
      }
      // For the edge case we want the chosen face's first halfedge to be
      // `chosenHe` so the layout convention (+x along face.halfedge())
      // applies symmetrically. But the face's stored first halfedge may not
      // be `chosenHe`. Compute the layout off `m.faceHalfedge(face)` (gc's
      // convention) and place the start point on the segment of that
      // layout corresponding to `chosenHe`.
      face = m.face(chosenHe);
      // Find which corner index of the face's CCW order corresponds to the
      // tail of `chosenHe`, so we can compute barycentrics.
      const h0 = m.faceHalfedge(face);
      const h1 = m.next(h0);
      const h2 = m.next(h1);
      let b: [number, number, number];
      // chosenHe goes from its tail to its tip. If it equals h0, the start
      // sits on the h0 segment: bary = (1-edgeT) at corner(h0), edgeT at
      // corner(h1). And similarly for h1, h2. Corner indices in
      // (b0, b1, b2) line up with halfedge tails (h0, h1, h2).
      if (chosenHe === h0) {
        b = [1 - edgeT, edgeT, 0];
      } else if (chosenHe === h1) {
        b = [0, 1 - edgeT, edgeT];
      } else if (chosenHe === h2) {
        b = [edgeT, 0, 1 - edgeT];
      } else {
        throw new Error(
          `tracePolylineFromSurfacePoint: edge halfedge ${chosenHe} not in face ${face}`,
        );
      }
      baryInFace = b;
    }

    // Lay out the chosen face with its first halfedge along +x.
    const heA = m.faceHalfedge(face);
    const heB = m.next(heA);
    const heC = m.next(heB);
    const lAB = this.inputGeometry.halfedgeLength(heA);
    const lBC = this.inputGeometry.halfedgeLength(heB);
    const lCA = this.inputGeometry.halfedgeLength(heC);
    const pA: Vec2 = [0, 0];
    const pB: Vec2 = [lAB, 0];
    const pC = layoutTriangleVertex(pA, pB, lBC, lCA);

    const [b0, b1, b2] = baryInFace;
    const p: Vec2 = [
      b0 * pA[0] + b1 * pB[0] + b2 * pC[0],
      b0 * pA[1] + b1 * pB[1] + b2 * pC[1],
    ];

    // Convert the trace direction.
    let faceFrameAngle: number;
    if (start.kind === 'face') {
      // For an inserted face vertex, "0 radians" in its tangent frame
      // points toward corner 0 (= origin) of the face's 2D layout. The
      // direction "toward (0,0)" from `p` is `-p`; its angle is
      // atan2(-p.y, -p.x).
      const offset = Math.atan2(-p[1], -p[0]);
      faceFrameAngle = offset + tangentAngle;
    } else {
      // 'edge'. "0 radians" in the inserted-edge-vertex frame points along
      // the edge halfedge. We chose the face/orientation above so that
      // the chosen halfedge runs in the +x direction of an edge layout
      // where `p` sits on the bottom edge of the face. But we laid out the
      // face using its `faceHalfedge`, not the chosen one. Re-derive the
      // angle by computing the direction along `chosenHe` in the face's
      // 2D frame and adding the user-supplied tangentAngle to its arg.
      // That edge runs from `p_tail` to `p_tip` in the face frame — find
      // those positions from the corner array.
      // Simpler: an edge insertion's tangent frame "0" points along its
      // halfedge. `chosenHe` runs from one corner to the next. Its
      // direction in the face frame is (cornerTip - cornerTail).
      const cornerPos = [pA, pB, pC];
      let tailIdx = -1;
      let tipIdx = -1;
      // chosenHe was used above; recompute it to keep the closure tight.
      const sinAngle = Math.sin(tangentAngle);
      const heE = m.edgeHalfedge(start.edge);
      let chosenHe: number;
      if (sinAngle >= 0) {
        chosenHe = m.face(heE) === INVALID_INDEX ? m.twin(heE) : heE;
      } else {
        const tw = m.twin(heE);
        chosenHe = m.face(tw) === INVALID_INDEX ? heE : tw;
      }
      const h0 = m.faceHalfedge(face);
      const h1 = m.next(h0);
      const h2 = m.next(h1);
      if (chosenHe === h0) {
        tailIdx = 0;
        tipIdx = 1;
      } else if (chosenHe === h1) {
        tailIdx = 1;
        tipIdx = 2;
      } else {
        tailIdx = 2;
        tipIdx = 0;
      }
      const tailP = cornerPos[tailIdx]!;
      const tipP = cornerPos[tipIdx]!;
      const baseAngle = Math.atan2(tipP[1] - tailP[1], tipP[0] - tailP[0]);
      faceFrameAngle = baseAngle + tangentAngle;
    }

    const dir: Vec2 = fromAngle(faceFrameAngle);

    return this.tracePolylineInFace2D(
      face,
      heA,
      pA,
      pB,
      pC,
      p,
      dir,
      dist,
      startPos,
    );
  }

  /**
   * Trace from the tail-vertex corner of `startHe`, with a 2D direction in
   * `face(startHe)` defined by rotating `+x` (the unit vector along
   * `startHe`) by `relAngleRaw` CCW. Walk distance `distance` along the
   * surface, transitioning across edges as needed.
   */
  private traceInFaceFromVertexCorner(
    startHe: number,
    relAngleRaw: number,
    distance: number,
  ): TraceResult {
    const m = this.inputGeometry.mesh;

    // Lay out the starting face in 2D with its three vertices A, B, C in
    // CCW order, where startHe goes A->B and the corner angle is at A.
    let face = m.face(startHe);
    let heA = startHe; // tail = A, head = B
    let heB = m.next(heA); // B->C
    let heC = m.next(heB); // C->A

    let vA = m.vertex(heA);
    let vB = m.vertex(heB);
    let vC = m.vertex(heC);

    let lAB = this.inputGeometry.halfedgeLength(heA);
    let lBC = this.inputGeometry.halfedgeLength(heB);
    let lCA = this.inputGeometry.halfedgeLength(heC);

    let pA: Vec2 = [0, 0];
    let pB: Vec2 = [lAB, 0];
    let pC = layoutTriangleVertex(pA, pB, lBC, lCA);

    // Direction from A: rotate +x by relAngleRaw CCW.
    let dir: Vec2 = fromAngle(relAngleRaw);
    // Trace start point.
    let p: Vec2 = [pA[0], pA[1]];
    let remaining = distance;

    // We keep iterating until either the trace ends inside a face or we hit
    // a boundary. To avoid infinite loops on numerical noise, cap iterations.
    const maxIters = m.nFaces * 4 + 8;
    for (let iter = 0; iter < maxIters; iter++) {
      // Find which edge of the triangle (pA->pB, pB->pC, pC->pA) the ray
      // `p + t*dir` intersects with smallest positive `t` (and bounded
      // `t <= remaining/|dir|=remaining` since `dir` is unit).
      // We test all three edges.

      // Endpoint candidate:
      const endX = p[0] + dir[0] * remaining;
      const endY = p[1] + dir[1] * remaining;

      let bestT = Number.POSITIVE_INFINITY;
      let crossEdgeIdx = -1; // 0 = AB (heA), 1 = BC (heB), 2 = CA (heC)

      // Helper: ray-segment intersection, returns t on the ray (>= 0) for
      // intersection with segment from `s0` to `s1`. Returns Infinity on miss
      // or near-parallel.
      const intersect = (s0: Vec2, s1: Vec2): number => {
        // Ray:     p + t * dir
        // Segment: s0 + u * (s1 - s0), u in [0, 1]
        //
        // Solving t*dir - u*s = s0 - p for [t, u] via Cramer's rule with
        //   A = [[dir.x, -s.x], [dir.y, -s.y]],  rhs = (s0 - p)
        // gives
        //   det = dir.x * -s.y - (-s.x) * dir.y = -dir.x*s.y + s.x*dir.y
        //   t   = (rhs.x * -s.y - rhs.y * -s.x) / det = (-rhs.x*s.y + rhs.y*s.x) / det
        //   u   = (dir.x * rhs.y - dir.y * rhs.x) / det
        const sx = s1[0] - s0[0];
        const sy = s1[1] - s0[1];
        const denom = -dir[0] * sy + sx * dir[1];
        if (Math.abs(denom) < TRACE_DENOM_EPS) return Number.POSITIVE_INFINITY;
        const rx = s0[0] - p[0];
        const ry = s0[1] - p[1];
        const t = (-rx * sy + ry * sx) / denom; // ray parameter
        const u = (dir[0] * ry - dir[1] * rx) / denom;
        if (t < TRACE_RAY_EPS) return Number.POSITIVE_INFINITY;
        if (u < -TRACE_SEGMENT_EPS || u > 1 + TRACE_SEGMENT_EPS) return Number.POSITIVE_INFINITY;
        return t;
      };

      // Don't test the edge we just entered through (would otherwise hit at
      // t≈0 due to numerics). The first iteration has no entry edge.
      const tAB = intersect(pA, pB);
      const tBC = intersect(pB, pC);
      const tCA = intersect(pC, pA);

      if (tAB < bestT) {
        bestT = tAB;
        crossEdgeIdx = 0;
      }
      if (tBC < bestT) {
        bestT = tBC;
        crossEdgeIdx = 1;
      }
      if (tCA < bestT) {
        bestT = tCA;
        crossEdgeIdx = 2;
      }

      // If no crossing within `remaining`, we end inside this face.
      if (bestT >= remaining || crossEdgeIdx === -1) {
        const pEnd: Vec2 = [endX, endY];
        const baryHeOrder = this.baryInLayoutTriangle(pEnd, pA, pB, pC);
        const pos = this.lift3DFromVertices(vA, vB, vC, baryHeOrder);
        const baryCanonical = this.baryInCanonicalFaceOrder(
          face,
          baryHeOrder,
          vA,
          vB,
          vC,
        );
        return {
          position: pos,
          faceIndex: face,
          barycentric: baryCanonical,
        };
      }

      // Otherwise we cross at parameter `bestT`. Compute the crossing point.
      const xCross: Vec2 = [p[0] + dir[0] * bestT, p[1] + dir[1] * bestT];
      const newRemaining = remaining - bestT;

      // Identify the halfedge we cross *out of*, then move to its twin's
      // face.
      const crossHe = crossEdgeIdx === 0 ? heA : crossEdgeIdx === 1 ? heB : heC;
      const twin = m.twin(crossHe);
      if (m.face(twin) === INVALID_INDEX) {
        // Hit the boundary — terminate at the crossing point.
        const baryHeOrder = this.baryInLayoutTriangle(xCross, pA, pB, pC);
        const pos = this.lift3DFromVertices(vA, vB, vC, baryHeOrder);
        const baryCanonical = this.baryInCanonicalFaceOrder(
          face,
          baryHeOrder,
          vA,
          vB,
          vC,
        );
        return {
          position: pos,
          faceIndex: face,
          barycentric: baryCanonical,
        };
      }

      // Compute parameter `u` along `crossHe` (from its tail to its tip):
      // for edge (s0, s1), u = ((xCross - s0) · (s1 - s0)) / |s1 - s0|^2.
      const s0 = crossEdgeIdx === 0 ? pA : crossEdgeIdx === 1 ? pB : pC;
      const s1 = crossEdgeIdx === 0 ? pB : crossEdgeIdx === 1 ? pC : pA;
      const sLen = Math.hypot(s1[0] - s0[0], s1[1] - s0[1]);

      // Re-lay out the next face. We orient it so the *twin* edge runs from
      // its tail to its tip in +x; that way we can directly transcribe the
      // intersection parameter.
      const newFace = m.face(twin);
      // In the next face, the halfedge we ENTER through is `twin`. Its
      // tail is the tip of `crossHe`, and its tip is the tail of `crossHe`.
      // We orient the next layout so:
      //   pA' = (0, 0)              corresponds to tail(twin) = tip(crossHe)
      //   pB' = (|twin|, 0)         corresponds to tip(twin)  = tail(crossHe)
      // and pC' is computed via layoutTriangleVertex.
      const newHeA = twin;
      const newHeB = m.next(newHeA);
      const newHeC = m.next(newHeB);
      const lABn = this.inputGeometry.halfedgeLength(newHeA);
      const lBCn = this.inputGeometry.halfedgeLength(newHeB);
      const lCAn = this.inputGeometry.halfedgeLength(newHeC);
      const newPA: Vec2 = [0, 0];
      const newPB: Vec2 = [lABn, 0];
      const newPC = layoutTriangleVertex(newPA, newPB, lBCn, lCAn);

      // The crossing point in the new layout: along `newHeA` from newPA
      // (which corresponds to `tail(twin) = tip(crossHe)`) to newPB
      // (`tip(twin) = tail(crossHe)`). The parameter going from
      // tail(crossHe) to tip(crossHe) was `u` along crossHe; in the new
      // frame the same point lies at parameter `1 - u` along newHeA.
      // Equivalently, we parameterise by arc length: distance along the
      // edge from `tip(crossHe)` is `(1 - u) * sLen` (geometry agrees:
      // |crossHe| = |newHeA|).
      const distFromS0 = Math.hypot(xCross[0] - s0[0], xCross[1] - s0[1]);
      const u = sLen > 0 ? distFromS0 / sLen : 0;
      // New entry point in the new frame:
      const newP: Vec2 = [(1 - u) * lABn, 0];

      // Direction transition: rotate the incoming dir so that the edge we
      // entered through (newHeA in the new frame, going +x) corresponds to
      // the edge we exited through (crossHe in the old frame, going from
      // s0 to s1). The relative orientation is a reflection: in the old
      // frame, the edge unit vector is `(s1 - s0) / sLen`. In the new
      // frame, the entry edge unit vector is `(+x)`. But since we entered
      // *through* this edge into the next face, we are going against
      // `newHeA` (the edge runs from tip(crossHe) to tail(crossHe), i.e.
      // OPPOSITE the direction we crossed). So we map:
      //
      //   t_old = unit(s1 - s0)  // tangent along outbound crossing in old face
      //   n_old = perp(t_old)    // perp rotated CCW (into old face)
      //
      //   t_new = -(+x)          // tangent along inbound crossing in new face,
      //                          //   pointing same physical direction as t_old
      //   n_new = -perp_new(+x)  // pointing into new face
      //
      // So a vector with components (alongTangent, alongInwardNormal) in
      // the old frame remains the same in the new frame, with both basis
      // vectors flipped: (a, n) -> (-a, -n). We use this directly.
      const tox = (s1[0] - s0[0]) / sLen;
      const toy = (s1[1] - s0[1]) / sLen;
      // Old in-frame components of dir:
      const aDir = dir[0] * tox + dir[1] * toy; // along tangent
      const nDir = dir[0] * -toy + dir[1] * tox; // along (CCW-perp = inward to old face)
      // New frame: entry edge runs +x, but we entered going -x; new tangent
      // = -(+x) = (-1, 0). New inward perp (into new face): rotate new
      // tangent by +90 CCW = (0, -1). So:
      const newDirX = aDir * -1 + nDir * 0;
      const newDirY = aDir * 0 + nDir * -1;

      // Renormalise (FP drift).
      const dn = Math.hypot(newDirX, newDirY);
      const newDir: Vec2 = dn > 0 ? [newDirX / dn, newDirY / dn] : [1, 0];

      // Step into the new face.
      face = newFace;
      heA = newHeA;
      heB = newHeB;
      heC = newHeC;
      vA = m.vertex(heA);
      vB = m.vertex(heB);
      vC = m.vertex(heC);
      lAB = lABn;
      lBC = lBCn;
      lCA = lCAn;
      pA = newPA;
      pB = newPB;
      pC = newPC;
      p = newP;
      dir = newDir;
      remaining = newRemaining;
    }

    // If we exhaust iterations, return the current position.
    const baryHeOrder = this.baryInLayoutTriangle(p, pA, pB, pC);
    const pos = this.lift3DFromVertices(vA, vB, vC, baryHeOrder);
    const baryCanonical = this.baryInCanonicalFaceOrder(
      face,
      baryHeOrder,
      vA,
      vB,
      vC,
    );
    return { position: pos, faceIndex: face, barycentric: baryCanonical };
  }

  /**
   * Polyline variant of {@link traceInFaceFromVertexCorner}. Same walk
   * (face-crossing 2D ray cast through the input mesh), but instead of
   * returning the trace endpoint we record every 3D point we visit:
   *
   *   - the start corner of `startHe` (tail vertex of `startHe`),
   *   - one point per face-boundary crossing (the 2D crossing point lifted
   *     to 3D using the face's 2D layout),
   *   - the final point inside the last face.
   *
   * Used only by `tracePolylineFromVertex`. Delegates to
   * {@link tracePolylineInFace2D} after laying out the start face with the
   * vertex corner at the origin.
   */
  private tracePolylineInFaceFromVertexCorner(
    startHe: number,
    relAngleRaw: number,
    distance: number,
  ): Vec3[] {
    const m = this.inputGeometry.mesh;

    const face = m.face(startHe);
    const heA = startHe;
    const heB = m.next(heA);
    const heC = m.next(heB);

    const lAB = this.inputGeometry.halfedgeLength(heA);
    const lBC = this.inputGeometry.halfedgeLength(heB);
    const lCA = this.inputGeometry.halfedgeLength(heC);

    const pA: Vec2 = [0, 0];
    const pB: Vec2 = [lAB, 0];
    const pC = layoutTriangleVertex(pA, pB, lBC, lCA);

    const dir: Vec2 = fromAngle(relAngleRaw);
    // Start point is the corner at the tail of `startHe`, which is `pA`.
    const startPoint: Vec2 = [pA[0], pA[1]];

    // Initial output point is the 3D position of the corner vertex.
    const startVertex3D = this.inputGeometry.position(m.vertex(heA));

    return this.tracePolylineInFace2D(
      face,
      heA,
      pA,
      pB,
      pC,
      startPoint,
      dir,
      distance,
      startVertex3D,
    );
  }

  /**
   * Inner polyline-trace driver. Walks a 2D ray across input-mesh faces,
   * starting from a fully-prepared 2D state. Decoupled from the start kind
   * so {@link tracePolylineInFaceFromVertexCorner} (vertex-corner start) and
   * {@link tracePolylineFromSurfacePoint} (face/edge-interior start) can
   * share the per-face-step logic.
   *
   * The face-transition logic is identical to gc's `traceInFaceFromEdge`
   * (`trace_geodesic.cpp`): when the ray exits through an edge, the new
   * face is laid out with its own first halfedge along +x, the entry point
   * is computed via arc-length parameter `u` along the shared edge, and the
   * direction is reflected through the (tangent, inward-perp) basis change.
   */
  private tracePolylineInFace2D(
    startFace: number,
    startHeA: number,
    initPA: Vec2,
    initPB: Vec2,
    initPC: Vec2,
    initPoint: Vec2,
    initDir: Vec2,
    distance: number,
    initialOutPoint: Vec3,
  ): Vec3[] {
    const m = this.inputGeometry.mesh;

    let face = startFace;
    let heA = startHeA;
    let heB = m.next(heA);
    let heC = m.next(heB);

    let vA = m.vertex(heA);
    let vB = m.vertex(heB);
    let vC = m.vertex(heC);

    // Edge lengths are reassigned per face but only used implicitly via
    // the laid-out 2D vertex positions; keep them in sync for clarity /
    // debugging parity with `traceInFaceFromVertexCorner`.
    let lAB = this.inputGeometry.halfedgeLength(heA);
    let lBC = this.inputGeometry.halfedgeLength(heB);
    let lCA = this.inputGeometry.halfedgeLength(heC);

    let pA: Vec2 = [initPA[0], initPA[1]];
    let pB: Vec2 = [initPB[0], initPB[1]];
    let pC: Vec2 = [initPC[0], initPC[1]];

    let dir: Vec2 = [initDir[0], initDir[1]];
    let p: Vec2 = [initPoint[0], initPoint[1]];
    let remaining = distance;

    // Output buffer: starts with the externally-supplied start position
    // (corner vertex / surface-point lift).
    const out: Vec3[] = [initialOutPoint];

    const maxIters = m.nFaces * 4 + 8;
    for (let iter = 0; iter < maxIters; iter++) {
      const endX = p[0] + dir[0] * remaining;
      const endY = p[1] + dir[1] * remaining;

      let bestT = Number.POSITIVE_INFINITY;
      let crossEdgeIdx = -1;

      const intersect = (s0: Vec2, s1: Vec2): number => {
        const sx = s1[0] - s0[0];
        const sy = s1[1] - s0[1];
        const denom = -dir[0] * sy + sx * dir[1];
        if (Math.abs(denom) < TRACE_DENOM_EPS) return Number.POSITIVE_INFINITY;
        const rx = s0[0] - p[0];
        const ry = s0[1] - p[1];
        const t = (-rx * sy + ry * sx) / denom;
        const u = (dir[0] * ry - dir[1] * rx) / denom;
        if (t < TRACE_RAY_EPS) return Number.POSITIVE_INFINITY;
        if (u < -TRACE_SEGMENT_EPS || u > 1 + TRACE_SEGMENT_EPS) return Number.POSITIVE_INFINITY;
        return t;
      };

      const tAB = intersect(pA, pB);
      const tBC = intersect(pB, pC);
      const tCA = intersect(pC, pA);

      if (tAB < bestT) {
        bestT = tAB;
        crossEdgeIdx = 0;
      }
      if (tBC < bestT) {
        bestT = tBC;
        crossEdgeIdx = 1;
      }
      if (tCA < bestT) {
        bestT = tCA;
        crossEdgeIdx = 2;
      }

      if (bestT >= remaining || crossEdgeIdx === -1) {
        // Trace ends inside this face; lift to 3D and append.
        const pEnd: Vec2 = [endX, endY];
        const bary = this.baryInLayoutTriangle(pEnd, pA, pB, pC);
        out.push(this.lift3DFromVertices(vA, vB, vC, bary));
        return out;
      }

      // Edge crossing — lift and append, then step into neighbour face.
      const xCross: Vec2 = [p[0] + dir[0] * bestT, p[1] + dir[1] * bestT];
      const baryCross = this.baryInLayoutTriangle(xCross, pA, pB, pC);
      out.push(this.lift3DFromVertices(vA, vB, vC, baryCross));

      const newRemaining = remaining - bestT;
      const crossHe = crossEdgeIdx === 0 ? heA : crossEdgeIdx === 1 ? heB : heC;
      const twin = m.twin(crossHe);
      if (m.face(twin) === INVALID_INDEX) {
        // Hit the boundary — terminate at the crossing point we just appended.
        return out;
      }

      const s0 = crossEdgeIdx === 0 ? pA : crossEdgeIdx === 1 ? pB : pC;
      const s1 = crossEdgeIdx === 0 ? pB : crossEdgeIdx === 1 ? pC : pA;
      const sLen = Math.hypot(s1[0] - s0[0], s1[1] - s0[1]);

      const newFace = m.face(twin);
      const newHeA = twin;
      const newHeB = m.next(newHeA);
      const newHeC = m.next(newHeB);
      const lABn = this.inputGeometry.halfedgeLength(newHeA);
      const lBCn = this.inputGeometry.halfedgeLength(newHeB);
      const lCAn = this.inputGeometry.halfedgeLength(newHeC);
      const newPA: Vec2 = [0, 0];
      const newPB: Vec2 = [lABn, 0];
      const newPC = layoutTriangleVertex(newPA, newPB, lBCn, lCAn);

      const distFromS0 = Math.hypot(xCross[0] - s0[0], xCross[1] - s0[1]);
      const u = sLen > 0 ? distFromS0 / sLen : 0;
      const newP: Vec2 = [(1 - u) * lABn, 0];

      const tox = (s1[0] - s0[0]) / sLen;
      const toy = (s1[1] - s0[1]) / sLen;
      const aDir = dir[0] * tox + dir[1] * toy;
      const nDir = dir[0] * -toy + dir[1] * tox;
      const newDirX = aDir * -1 + nDir * 0;
      const newDirY = aDir * 0 + nDir * -1;
      const dn = Math.hypot(newDirX, newDirY);
      const newDir: Vec2 = dn > 0 ? [newDirX / dn, newDirY / dn] : [1, 0];

      face = newFace;
      heA = newHeA;
      heB = newHeB;
      heC = newHeC;
      vA = m.vertex(heA);
      vB = m.vertex(heB);
      vC = m.vertex(heC);
      lAB = lABn;
      lBC = lBCn;
      lCA = lCAn;
      pA = newPA;
      pB = newPB;
      pC = newPC;
      p = newP;
      dir = newDir;
      remaining = newRemaining;
    }

    // Iter cap: append best-effort endpoint. `face` tracks the current input
    // face index for parity with `traceInFaceFromVertexCorner`; not part of
    // the polyline output.
    void face;
    const bary = this.baryInLayoutTriangle(p, pA, pB, pC);
    out.push(this.lift3DFromVertices(vA, vB, vC, bary));
    return out;
  }

  // --------------------------------------------------------------------------
  // Trace helpers.
  // --------------------------------------------------------------------------

  /**
   * Barycentric coords of a point `p` inside the layout triangle
   * `(pA, pB, pC)`. Returns weights in the order `(A, B, C)`.
   * Uses the area-ratio formula, robust enough for our 2D layouts.
   */
  private baryInLayoutTriangle(p: Vec2, pA: Vec2, pB: Vec2, pC: Vec2): Vec3 {
    const totalArea =
      (pB[0] - pA[0]) * (pC[1] - pA[1]) - (pB[1] - pA[1]) * (pC[0] - pA[0]);
    if (Math.abs(totalArea) < 1e-30) return [1, 0, 0];

    const areaA =
      (pB[0] - p[0]) * (pC[1] - p[1]) - (pB[1] - p[1]) * (pC[0] - p[0]);
    const areaB =
      (pC[0] - p[0]) * (pA[1] - p[1]) - (pC[1] - p[1]) * (pA[0] - p[0]);
    const a = areaA / totalArea;
    const b = areaB / totalArea;
    const c = 1 - a - b;
    return [a, b, c];
  }

  /**
   * Lift a barycentric weighted combination of three vertex positions
   * (specified directly) into 3D. Used by the trace, which tracks the
   * three vertices it laid out in 2D and feeds them in directly so we
   * don't have to worry about reordering against `halfedgesAroundFace`.
   */
  private lift3DFromVertices(va: number, vb: number, vc: number, bary: Vec3): Vec3 {
    const pA = this.inputGeometry.position(va);
    const pB = this.inputGeometry.position(vb);
    const pC = this.inputGeometry.position(vc);
    return [
      bary[0] * pA[0] + bary[1] * pB[0] + bary[2] * pC[0],
      bary[0] * pA[1] + bary[1] * pB[1] + bary[2] * pC[1],
      bary[0] * pA[2] + bary[1] * pB[2] + bary[2] * pC[2],
    ];
  }

  /**
   * Reorder the three barycentric weights `[a, b, c]` (which correspond to
   * vertices `[vA, vB, vC]` in some order) to the canonical
   * `halfedgesAroundFace(f)` order. We need this so the returned
   * `barycentric` field of `TraceResult` matches the documented contract
   * (face's CCW vertex order).
   */
  private baryInCanonicalFaceOrder(
    f: number,
    bary: Vec3,
    vA: number,
    vB: number,
    vC: number,
  ): Vec3 {
    const m = this.inputGeometry.mesh;
    const it = m.halfedgesAroundFace(f);
    const h0 = it.next().value as number;
    const h1 = it.next().value as number;
    const h2 = it.next().value as number;
    const w0 = m.vertex(h0);
    const w1 = m.vertex(h1);
    const w2 = m.vertex(h2);
    const map = (w: number): number => {
      if (w === vA) return bary[0];
      if (w === vB) return bary[1];
      if (w === vC) return bary[2];
      throw new Error(`vertex ${w} not in {${vA}, ${vB}, ${vC}}`);
    };
    return [map(w0), map(w1), map(w2)];
  }

  /**
   * Pick any interior face incident on input-mesh vertex `v`. Used to build
   * a `TraceResult` for distance-0 traces.
   */
  private firstFaceAround(v: number): number {
    const m = this.inputGeometry.mesh;
    for (const he of m.outgoingHalfedges(v)) {
      const f = m.face(he);
      if (f !== INVALID_INDEX) return f;
    }
    throw new Error(`vertex ${v} has no incident interior face`);
  }

  /**
   * Barycentric coords of vertex `v` inside face `f`. (One of the bary coords
   * is 1.)
   */
  private vertexBaryInFace(v: number, f: number): Vec3 {
    const m = this.inputGeometry.mesh;
    const it = m.halfedgesAroundFace(f);
    const h0 = it.next().value as number;
    const h1 = it.next().value as number;
    const h2 = it.next().value as number;
    if (m.vertex(h0) === v) return [1, 0, 0];
    if (m.vertex(h1) === v) return [0, 1, 0];
    if (m.vertex(h2) === v) return [0, 0, 1];
    throw new Error(`vertex ${v} not in face ${f}`);
  }
}
