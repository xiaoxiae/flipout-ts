// Tests for `tracePolylineFromSurfacePoint`.
//
// This is a polyline tracer that starts at any SurfacePoint (vertex / edge /
// face) on the input mesh, walks a tangent direction for a given distance,
// and emits the 3D polyline that crosses input-mesh face boundaries.
//
// Mirrors the high-level behaviour of geometry-central's `traceGeodesic`
// (`src/surface/trace_geodesic.cpp`) — specifically its dispatch by
// `SurfacePoint` kind. We do NOT bit-match gc on a vector-typed direction
// (we accept a scalar tangent angle); the underlying geometry is the same.
//
// Conventions verified by these tests:
//
//   * vertex-kind start: identical output to `tracePolylineFromVertex`.
//   * face-kind start: "0 radians" in the inserted-vertex tangent frame
//     points toward face's corner 0 in the input face's 2D laydown
//     (matching gc's `vertexCoordinatesInTriangle`, where corner 0 is at
//     the origin and +x runs along `face.halfedge()`).
//   * distance-zero trace: returns just the start position.

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import type { SurfacePoint } from '../../../src/intrinsic/index.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { cube, flatGrid, flatQuad, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

interface MeshDataLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function buildSit(m: MeshDataLike): SignpostIntrinsicTriangulation {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  return new SignpostIntrinsicTriangulation(geom);
}

function dist3(a: readonly number[], b: readonly number[]): number {
  return Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!, a[2]! - b[2]!);
}

function summedLength(polyline: readonly (readonly number[])[]): number {
  let s = 0;
  for (let i = 1; i < polyline.length; i++) {
    s += dist3(polyline[i - 1]!, polyline[i]!);
  }
  return s;
}

describe('tracePolylineFromSurfacePoint — vertex start delegates to tracePolylineFromVertex', () => {
  it.each<[string, MeshDataLike, number, number, number]>([
    ['cube v0, angle 0, dist 0.5', cube(), 0, 0, 0.5],
    ['cube v0, angle π/3, dist 1.0', cube(), 0, Math.PI / 3, 1.0],
    ['icosahedron v3, angle π/4, dist 1.5', icosahedron(), 3, Math.PI / 4, 1.5],
    ['flatGrid 4 v5, angle 0.7, dist 0.4', flatGrid(4, 1), 5, 0.7, 0.4],
  ])('%s', (_, m, v, angle, distance) => {
    const sit = buildSit(m);
    const ref = sit.tracePolylineFromVertex(v, angle, distance);
    const got = sit.tracePolylineFromSurfacePoint({ kind: 'vertex', vertex: v }, angle, distance);
    expect(got.length).toBe(ref.length);
    for (let i = 0; i < ref.length; i++) {
      for (let k = 0; k < 3; k++) {
        expect(got[i]![k]).toBeCloseTo(ref[i]![k]!, 12);
      }
    }
  });
});

describe('tracePolylineFromSurfacePoint — distance zero', () => {
  it('vertex start, dist 0 returns just the start position', () => {
    const sit = buildSit(cube());
    const out = sit.tracePolylineFromSurfacePoint({ kind: 'vertex', vertex: 0 }, 0, 0);
    expect(out.length).toBe(1);
    const p = sit.inputGeometry.position(0);
    expect(out[0]![0]).toBeCloseTo(p[0]!, 12);
    expect(out[0]![1]).toBeCloseTo(p[1]!, 12);
    expect(out[0]![2]).toBeCloseTo(p[2]!, 12);
  });

  it('face-interior start, dist 0 returns just the surface position', () => {
    const sit = buildSit(flatQuad());
    const sp: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const out = sit.tracePolylineFromSurfacePoint(sp, 1.234, 0);
    expect(out.length).toBe(1);
    const p = sit.surfacePointPosition(sp);
    expect(out[0]![0]).toBeCloseTo(p[0]!, 12);
    expect(out[0]![1]).toBeCloseTo(p[1]!, 12);
    expect(out[0]![2]).toBeCloseTo(p[2]!, 12);
  });
});

describe('tracePolylineFromSurfacePoint — face-interior start on flat geometry', () => {
  it('flatQuad face 0 centroid: tracing toward corner 0 lands within distance × dir', () => {
    // flatQuad face 0 = (0,0,0)-(1,0,0)-(1,1,0). The centroid is at
    // (2/3, 1/3, 0). "0 radians" in our inserted-vertex tangent frame
    // points toward corner 0 = (0,0,0). Distance 0.1 lands at the
    // centroid moved 0.1 in that direction.
    const sit = buildSit(flatQuad());
    const sp: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const dist = 0.1;
    const out = sit.tracePolylineFromSurfacePoint(sp, 0, dist);
    expect(out.length).toBe(2);

    const start = out[0]!;
    expect(start[0]).toBeCloseTo(2 / 3, 8);
    expect(start[1]).toBeCloseTo(1 / 3, 8);
    expect(start[2]).toBeCloseTo(0, 8);

    // Direction toward (0,0,0) from (2/3, 1/3, 0) is normalised (-2, -1, 0).
    const norm = Math.hypot(-2 / 3, -1 / 3);
    const dx = (-2 / 3) / norm;
    const dy = (-1 / 3) / norm;
    const end = out[1]!;
    expect(end[0]).toBeCloseTo(2 / 3 + dx * dist, 8);
    expect(end[1]).toBeCloseTo(1 / 3 + dy * dist, 8);
    expect(end[2]).toBeCloseTo(0, 8);
  });

  it('flatQuad face 0 centroid: tracing in opposite directions gives reflected endpoints', () => {
    const sit = buildSit(flatQuad());
    const sp: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const dist = 0.05;
    const ang = 0.7;
    const fwd = sit.tracePolylineFromSurfacePoint(sp, ang, dist);
    const bwd = sit.tracePolylineFromSurfacePoint(sp, ang + Math.PI, dist);
    const center = sit.surfacePointPosition(sp);
    const fEnd = fwd[fwd.length - 1]!;
    const bEnd = bwd[bwd.length - 1]!;
    // start positions are equal (both equal to centroid)
    expect(fwd[0]![0]).toBeCloseTo(center[0]!, 10);
    expect(bwd[0]![0]).toBeCloseTo(center[0]!, 10);
    // endpoints are reflected through the centroid: (fEnd + bEnd) / 2 == center
    expect((fEnd[0] + bEnd[0]) / 2).toBeCloseTo(center[0]!, 8);
    expect((fEnd[1] + bEnd[1]) / 2).toBeCloseTo(center[1]!, 8);
    expect((fEnd[2] + bEnd[2]) / 2).toBeCloseTo(center[2]!, 8);
  });

  it('flatGrid face 0 interior: trace within the face emits 2 points', () => {
    // flatGrid(4, 1) face 0 = unit triangle in [0, 1/3]. Centroid lies
    // inside; a small trace stays inside the same face.
    const sit = buildSit(flatGrid(4, 1));
    const sp: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const out = sit.tracePolylineFromSurfacePoint(sp, 0.3, 0.05);
    expect(out.length).toBe(2);
  });

  it('flatGrid face 0: trace crossing into a neighbour face emits ≥ 3 points', () => {
    // Trace far enough that we leave face 0. Empirically a tangent angle
    // of 5π/4 from face 0's centroid points back across multiple internal
    // grid faces (away from the grid boundary).
    const sit = buildSit(flatGrid(4, 1));
    const sp: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const out = sit.tracePolylineFromSurfacePoint(sp, (5 * Math.PI) / 4, 0.6);
    expect(out.length).toBeGreaterThanOrEqual(3);
    // trace length should match input distance (flat surface, geodesic =
    // straight line).
    expect(summedLength(out)).toBeCloseTo(0.6, 6);
  });
});

describe('tracePolylineFromSurfacePoint — face-interior start on a curved tetrahedron', () => {
  it('tetrahedron face 0 centroid: tracing far enough crosses ≥ 1 face boundary', () => {
    const sit = buildSit(tetrahedron());
    const sp: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    // Tetrahedron face edges have length 2*sqrt(2) ≈ 2.828; trace 3.0 to
    // be sure we cross a boundary.
    const out = sit.tracePolylineFromSurfacePoint(sp, 1.2, 3.0);
    expect(out.length).toBeGreaterThanOrEqual(3);
    expect(summedLength(out)).toBeCloseTo(3.0, 6);

    // Endpoint should lie within the tetrahedron's bounding box [-1, 1]³.
    for (const p of out) {
      for (let k = 0; k < 3; k++) {
        expect(p[k]).toBeGreaterThanOrEqual(-1.0001);
        expect(p[k]).toBeLessThanOrEqual(1.0001);
      }
    }
  });

  it('tetrahedron face 0 centroid: tracing zero distance returns one point', () => {
    const sit = buildSit(tetrahedron());
    const sp: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const out = sit.tracePolylineFromSurfacePoint(sp, 0, 0);
    expect(out.length).toBe(1);
    const p = sit.surfacePointPosition(sp);
    expect(dist3(out[0]!, p)).toBeLessThan(1e-12);
  });
});

describe('tracePolylineFromSurfacePoint — face-interior start with non-uniform bary', () => {
  it('flatQuad face 0, bary near corner 0: trace toward corner 0 endpoint is close to corner', () => {
    // bary = (0.95, 0.025, 0.025) puts the start near corner 0 of face 0
    // (= (0,0,0)). Tracing in direction θ=0 (= toward corner 0) for
    // distance 0.04 should bring us near (0,0,0).
    const sit = buildSit(flatQuad());
    const sp: SurfacePoint = { kind: 'face', face: 0, bary: [0.95, 0.025, 0.025] };
    const startPos = sit.surfacePointPosition(sp);
    const distToCorner = Math.hypot(startPos[0]!, startPos[1]!, startPos[2]!);
    const out = sit.tracePolylineFromSurfacePoint(sp, 0, distToCorner);
    expect(out.length).toBeGreaterThanOrEqual(2);
    const last = out[out.length - 1]!;
    expect(last[0]).toBeCloseTo(0, 4);
    expect(last[1]).toBeCloseTo(0, 4);
  });
});
