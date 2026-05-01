// Ported from geometry-central:
//   include/geometrycentral/surface/vertex_position_geometry.h
//   include/geometrycentral/surface/vertex_position_geometry.ipp
//   src/surface/vertex_position_geometry.cpp
//   src/surface/intrinsic_geometry_interface.cpp   (corner angles, vertex angle sums)
//   src/surface/embedded_geometry_interface.cpp    (face normals)
//
// L2 — extrinsic geometry: a `SurfaceMesh` (L1 connectivity) plus a per-vertex
// position array. All quantities are pure functions of the current positions;
// no caching machinery — callers wanting to amortise work cache results
// themselves. This deliberately omits geometry-central's
// `requireFaceAreas()` / cached-quantity infrastructure: FlipOut and the
// signpost layer mostly need *immediate* per-element values during edge
// flipping, where caches would be invalidated on every step anyway.
//
// What is *not* ported (and why):
//   - `vertexDualArea`, `halfedgeCotanWeight`, `edgeCotanWeight`,
//     `edgeDihedralAngle`, `vertexMeanCurvature`, `vertexMinPrincipalCurvature`,
//     `vertexMaxPrincipalCurvature`, `vertexDualMeanCurvatureNormal`:
//     not needed by FlipOut or the signpost intrinsic triangulation.
//   - `copy` / `reinterpretTo`: the geometry is just `(mesh, positions)`;
//     callers can construct a new instance directly if they need a copy.
//   - The cached-quantity API (`requireXxx` / `unrequireXxx`): unnecessary
//     without curvature flow / cotan-Laplacian solves.

import type { Vec3 } from '../math/vec3.js';
import { cross, distance, norm, scale, sub } from '../math/vec3.js';
import { cornerAngleFromLengths, triangleArea } from '../math/triangle.js';
import { INVALID_INDEX, type SurfaceMesh } from '../mesh/surface-mesh.js';

/**
 * Triangle-mesh embedded geometry: a `SurfaceMesh` with per-vertex 3-D
 * positions. Provides the immediate per-element quantities (edge length,
 * face area, face normal, corner angle, vertex angle defect) that the
 * signpost intrinsic triangulation and FlipOut need.
 *
 * `positions` is captured by reference; callers must not mutate the array
 * after construction (the field is typed `readonly Vec3[]` to nudge this,
 * but TypeScript can't prevent in-place writes by alias). If you need to
 * change positions, build a new `VertexPositionGeometry`.
 */
export class VertexPositionGeometry {
  /** The connectivity (halfedge) mesh. */
  readonly mesh: SurfaceMesh;
  /** Position of each vertex, indexed by vertex id. Length === `mesh.nVertices`. */
  readonly positions: readonly Vec3[];

  /**
   * Build a geometry from a mesh and one position per vertex. Throws if the
   * positions array length doesn't match the mesh's vertex count.
   */
  constructor(mesh: SurfaceMesh, positions: readonly Vec3[]) {
    if (positions.length !== mesh.nVertices) {
      throw new RangeError(
        `positions.length (${positions.length}) !== mesh.nVertices (${mesh.nVertices})`,
      );
    }
    this.mesh = mesh;
    this.positions = positions;
  }

  // ---------------------------------------------------------------------------
  // Edge / halfedge lengths.
  //
  // From `vertex_position_geometry.ipp::edgeLength`:
  //   pA = positions[he.vertex()]
  //   pB = positions[he.next().vertex()]
  //   return |pA - pB|
  //
  // Note `he.next().vertex()` is the *tail* of `next(he)`, which equals the
  // *tip* of `he` itself (these are the same vertex in a triangle face).
  // ---------------------------------------------------------------------------

  /** Position of vertex `v`. Throws if `v` is out of range. */
  position(v: number): Vec3 {
    const p = this.positions[v];
    if (p === undefined) {
      throw new RangeError(`vertex ${v} out of range [0, ${this.mesh.nVertices})`);
    }
    return p;
  }

  /** Length of edge `e`: distance between its two endpoints. */
  edgeLength(e: number): number {
    const he = this.mesh.edgeHalfedge(e);
    return this.halfedgeLength(he);
  }

  /**
   * Length of halfedge `he`. Identical to `edgeLength(edge(he))` by definition,
   * exposed because intrinsic-triangulation code routinely thinks in
   * halfedge-keyed quantities.
   */
  halfedgeLength(he: number): number {
    const tail = this.mesh.tailVertex(he);
    const tip = this.mesh.tipVertex(he);
    return distance(this.position(tail), this.position(tip));
  }

  /**
   * Vector from `tailVertex(he)` to `tipVertex(he)`. Mirrors
   * geometry-central's `halfedgeVector` (vpg.ipp:130).
   */
  halfedgeVector(he: number): Vec3 {
    const tail = this.mesh.tailVertex(he);
    const tip = this.mesh.tipVertex(he);
    return sub(this.position(tip), this.position(tail));
  }

  // ---------------------------------------------------------------------------
  // Face quantities.
  //
  // From `vertex_position_geometry.ipp::faceArea` (triangle-only branch):
  //   walk three corners, take 0.5 * |cross(pB - pA, pC - pA)|
  // Same source used for `faceNormal`, but normalised. We delegate to L0's
  // `triangleArea` and an inline cross-product for the normal; this keeps the
  // boundary-of-mesh check in one place (we *don't* iterate over boundary
  // halfedges since this is the immediate version, called only with valid
  // face indices).
  // ---------------------------------------------------------------------------

  /**
   * Area of triangular face `f`. Computed from the cross product of two
   * edge vectors, so degenerate (collinear) triangles return 0 cleanly.
   */
  faceArea(f: number): number {
    const [pA, pB, pC] = this.facePositions(f);
    return triangleArea(pA, pB, pC);
  }

  /**
   * Outward unit normal of triangular face `f`. Direction follows the CCW
   * winding of the face's halfedges. Returns `[0, 0, 0]` for a degenerate
   * face (matches `Vec3Ops.normalize` behaviour from L0).
   */
  faceNormal(f: number): Vec3 {
    const [pA, pB, pC] = this.facePositions(f);
    const n = cross(sub(pB, pA), sub(pC, pA));
    const len = norm(n);
    if (len === 0) return [0, 0, 0];
    return [n[0] / len, n[1] / len, n[2] / len];
  }

  /** Centroid (average of the three vertex positions) of triangular face `f`. */
  faceCentroid(f: number): Vec3 {
    const [pA, pB, pC] = this.facePositions(f);
    return scale(
      [pA[0] + pB[0] + pC[0], pA[1] + pB[1] + pC[1], pA[2] + pB[2] + pC[2]],
      1 / 3,
    );
  }

  /**
   * Helper: return the three vertex positions of a triangular face in CCW
   * order. Throws if the face is non-triangular (defensive — `SurfaceMesh`
   * is triangle-only by construction, but the check protects against bugs
   * in future mutation methods).
   */
  private facePositions(f: number): [Vec3, Vec3, Vec3] {
    const h0 = this.mesh.faceHalfedge(f);
    const h1 = this.mesh.next(h0);
    const h2 = this.mesh.next(h1);
    if (this.mesh.next(h2) !== h0) {
      throw new Error(`face ${f} is non-triangular`);
    }
    const pA = this.position(this.mesh.vertex(h0));
    const pB = this.position(this.mesh.vertex(h1));
    const pC = this.position(this.mesh.vertex(h2));
    return [pA, pB, pC];
  }

  // ---------------------------------------------------------------------------
  // Corner angles & vertex angle defect.
  //
  // For *intrinsic* purposes (and to share code with L3 once it lives there)
  // we compute corner angles from edge lengths via the cosine rule rather
  // than the cosine-of-unit-vectors form in `vpg.ipp::cornerAngle`. The
  // length-only form is what `intrinsic_geometry_interface.cpp::computeCornerAngles`
  // does, and is more numerically robust for sliver triangles than
  // `acos(dot(unit(a), unit(b)))`. See `cornerAngleFromLengths` in
  // `src/math/triangle.ts` for the Kahan-flavoured version.
  //
  // Halfedge convention reminder (see `src/mesh/surface-mesh.ts` header):
  //   In a triangle face, `cornerAngle(he)` is the interior angle at
  //   `tailVertex(he)`. The two adjacent edges of that corner are the
  //   halfedge `he` itself (going to the next vertex) and the halfedge
  //   `prev(he) = next(next(he))` reversed (going to the previous vertex).
  //   The opposite edge is `next(he)`, connecting the other two corners.
  // ---------------------------------------------------------------------------

  /**
   * Interior corner angle at `tailVertex(he)`, inside the face on the left
   * of `he`. Computed from the three edge lengths via the cosine rule.
   * Throws if `he` is a boundary halfedge (no incident face).
   */
  cornerAngle(he: number): number {
    if (this.mesh.face(he) === INVALID_INDEX) {
      throw new Error(
        `cornerAngle: halfedge ${he} is on the boundary (face = INVALID_INDEX)`,
      );
    }
    const heNext = this.mesh.next(he);
    const hePrev = this.mesh.next(heNext); // == prev(he) in a triangle
    // Sides of the corner at tailVertex(he):
    //   lA = length of `he`        (corner -> next vertex)
    //   lB = length of `prev(he)` reversed, == length(hePrev) (prev -> corner)
    // Opposite side:
    //   lOpp = length of `next(he)`  (next vertex -> prev vertex)
    const lA = this.halfedgeLength(he);
    const lB = this.halfedgeLength(hePrev);
    const lOpp = this.halfedgeLength(heNext);
    return cornerAngleFromLengths(lOpp, lA, lB);
  }

  /**
   * Sum of interior corner angles at vertex `v`, summed across every
   * incident *interior* face. Boundary-halfedge corners are skipped (they
   * are undefined). Mirrors
   * `intrinsic_geometry_interface.cpp::computeVertexAngleSums`.
   */
  vertexAngleSum(v: number): number {
    let sum = 0;
    for (const he of this.mesh.outgoingHalfedges(v)) {
      // The corner at `v` inside `face(he)` is `cornerAngle(he)`.
      // Skip boundary halfedges (no face on the left).
      if (this.mesh.face(he) === INVALID_INDEX) continue;
      sum += this.cornerAngle(he);
    }
    return sum;
  }

  /**
   * Angle defect at vertex `v`:
   *   - interior vertex: `2π − vertexAngleSum(v)`
   *   - boundary vertex: `π − vertexAngleSum(v)` (the geodesic curvature
   *     contribution; the unrolled neighbourhood of a boundary vertex
   *     should fan out to a half-disk = π).
   *
   * Summed over all vertices this gives `2π · χ` for closed manifolds and
   * the Gauss-Bonnet result `2π · χ` (counting both Gaussian curvature at
   * interior vertices and geodesic curvature at boundary vertices) for
   * meshes with boundary.
   */
  vertexAngleDefect(v: number): number {
    const sum = this.vertexAngleSum(v);
    const target = this.mesh.isBoundaryVertex(v) ? Math.PI : 2 * Math.PI;
    return target - sum;
  }

  /**
   * Total angle defect, summed over every vertex. By the discrete
   * Gauss-Bonnet theorem this equals `2π · χ` for any closed triangle mesh,
   * and `2π · χ` (with χ accounting for boundary loops) for meshes with
   * boundary — provided the boundary `π − sum` convention above is used.
   */
  totalAngleDefect(): number {
    let total = 0;
    for (let v = 0; v < this.mesh.nVertices; v++) {
      total += this.vertexAngleDefect(v);
    }
    return total;
  }

}
