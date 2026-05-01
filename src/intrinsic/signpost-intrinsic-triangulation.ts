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

  /** Intrinsic edge lengths, indexed by `intrinsicMesh` edge id. */
  readonly edgeLengths: Float64Array;

  /**
   * Sum of corner angles at each vertex (interior angle if interior, the
   * "boundary angle" sum if on the boundary). Constant under intrinsic
   * flips. Indexed by vertex id; same indexing on the input and intrinsic
   * meshes (vertices are shared).
   */
  readonly vertexAngleSums: Float64Array;

  /**
   * Per-halfedge signpost angle in radians, in `[0, vertexAngleSum[tail])`.
   * Mirrors geometry-central's `signpostAngle`. Updated by `flipEdge`.
   */
  readonly halfedgeSignposts: Float64Array;

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
