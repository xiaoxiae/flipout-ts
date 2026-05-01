/**
 * Tests for `flipOutPathFromSurfacePoints` — the SurfacePoint-aware FlipOut
 * convenience function. Verifies behaviour parity with `flipOutPath` for
 * vertex-only inputs, plus geometric correctness for edge / face inputs.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import type { SurfacePoint } from '../../../src/intrinsic/index.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import {
  flipOutPath,
  flipOutPathFromSurfacePoints,
  SNAP_EPS,
} from '../../../src/flipout/flip-edge-network.js';
import { cube, flatGrid, flatQuad, icosahedron } from '../../_helpers/meshes.js';

interface MeshDataLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}

function buildSit(m: MeshDataLike): SignpostIntrinsicTriangulation {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  return new SignpostIntrinsicTriangulation(geom);
}

describe('flipOutPathFromSurfacePoints — vertex-only inputs match flipOutPath', () => {
  it.each<[string, MeshDataLike, number, number]>([
    ['flatGrid 4x4 v0 → v15', flatGrid(4, 1), 0, 15],
    ['cube v0 → v6', cube(), 0, 6],
    ['icosahedron v0 → v3', icosahedron(), 0, 3],
  ])('%s', (_, m, vSrc, vDst) => {
    const sit1 = buildSit(m);
    const r1 = flipOutPath(sit1, vSrc, vDst);
    const sit2 = buildSit(m);
    const r2 = flipOutPathFromSurfacePoints(
      sit2,
      { kind: 'vertex', vertex: vSrc },
      { kind: 'vertex', vertex: vDst },
    );
    expect(r2.length).toBeCloseTo(r1.length, 12);
    expect(r2.iterations).toBe(r1.iterations);
    expect(r2.converged).toBe(r1.converged);
    expect(r2.polyline.length).toBe(r1.polyline.length);
    for (let i = 0; i < r1.polyline.length; i++) {
      for (let k = 0; k < 3; k++) {
        expect(r2.polyline[i]![k]).toBeCloseTo(r1.polyline[i]![k]!, 12);
      }
    }
  });
});

describe('flipOutPathFromSurfacePoints — edge midpoint to vertex on flat quad', () => {
  it('flatQuad: edge midpoint of edge 0-1 to vertex 2 has analytic length', () => {
    // flatQuad: vertices (0,0,0), (1,0,0), (1,1,0), (0,1,0). Edge 0-1's
    // midpoint is (0.5, 0, 0). Distance to (1,1,0) is sqrt(0.25 + 1) =
    // sqrt(1.25) ≈ 1.1180.
    const sit = buildSit(flatQuad());
    const im = sit.intrinsicMesh;
    let edge01 = -1;
    for (let e = 0; e < im.nEdges; e++) {
      const h = im.edgeHalfedge(e);
      const va = im.vertex(h);
      const vb = im.tipVertex(h);
      if ((va === 0 && vb === 1) || (va === 1 && vb === 0)) {
        edge01 = e;
        break;
      }
    }
    expect(edge01).toBeGreaterThanOrEqual(0);

    const src: SurfacePoint = { kind: 'edge', edge: edge01, t: 0.5 };
    const dst: SurfacePoint = { kind: 'vertex', vertex: 2 };
    const r = flipOutPathFromSurfacePoints(sit, src, dst);

    expect(r.length).toBeCloseTo(Math.sqrt(1.25), 8);
    expect(r.converged).toBe(true);
    // First polyline point should be near (0.5, 0, 0).
    expect(r.polyline[0]![0]).toBeCloseTo(0.5, 8);
    expect(r.polyline[0]![1]).toBeCloseTo(0, 8);
    expect(r.polyline[0]![2]).toBeCloseTo(0, 8);
    // Last polyline point should be near (1, 1, 0).
    const last = r.polyline[r.polyline.length - 1]!;
    expect(last[0]).toBeCloseTo(1, 8);
    expect(last[1]).toBeCloseTo(1, 8);
    expect(last[2]).toBeCloseTo(0, 8);
  });
});

describe('flipOutPathFromSurfacePoints — face center to face center on flat grid', () => {
  it('flatGrid 5x5: face-interior point to another face-interior point has Euclidean length', () => {
    // Two face centroids on a flat plane should give the Euclidean distance
    // between the two centroids as the geodesic length (since the grid is
    // flat).
    const sit = buildSit(flatGrid(5, 1));
    const im = sit.intrinsicMesh;

    const f0 = 0;
    // pick a far-away face index. flatGrid(5) has 4*4*2 = 32 faces.
    const f1 = im.nFaces - 1;

    // Compute centroids (in input mesh, which equals intrinsic at this point).
    const inputMesh = sit.inputGeometry.mesh;
    const verticesOf = (f: number): [number, number, number] => {
      const it = inputMesh.halfedgesAroundFace(f);
      const a = it.next().value as number;
      const b = it.next().value as number;
      const c = it.next().value as number;
      return [inputMesh.vertex(a), inputMesh.vertex(b), inputMesh.vertex(c)];
    };
    const verts0 = verticesOf(f0);
    const verts1 = verticesOf(f1);
    const pos = (v: number) => sit.inputGeometry.position(v);
    const centroid = (verts: [number, number, number]): [number, number, number] => {
      const a = pos(verts[0]);
      const b = pos(verts[1]);
      const c = pos(verts[2]);
      return [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
    };
    const c0 = centroid(verts0);
    const c1 = centroid(verts1);
    const expectedLen = Math.hypot(c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2]);

    const src: SurfacePoint = { kind: 'face', face: f0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const dst: SurfacePoint = { kind: 'face', face: f1, bary: [1 / 3, 1 / 3, 1 / 3] };
    const r = flipOutPathFromSurfacePoints(sit, src, dst);

    // On a flat manifold the geodesic length matches the Euclidean distance
    // (and triangle edges may add slack; we test closeness within ~1%).
    expect(r.length).toBeGreaterThan(expectedLen - 0.05);
    expect(r.length).toBeLessThan(expectedLen + 0.05);
    expect(r.converged).toBe(true);

    // Polyline starts at c0 and ends at c1 (within FP tolerance).
    expect(r.polyline[0]![0]).toBeCloseTo(c0[0], 8);
    expect(r.polyline[0]![1]).toBeCloseTo(c0[1], 8);
    const last = r.polyline[r.polyline.length - 1]!;
    expect(last[0]).toBeCloseTo(c1[0], 8);
    expect(last[1]).toBeCloseTo(c1[1], 8);
  });
});

describe('flipOutPathFromSurfacePoints — snap behaviour', () => {
  it('flatQuad: t=1e-12 on an edge snaps to the edge endpoint', () => {
    const sit = buildSit(flatQuad());
    const im = sit.intrinsicMesh;
    let edge02 = -1;
    for (let e = 0; e < im.nEdges; e++) {
      const h = im.edgeHalfedge(e);
      const va = im.vertex(h);
      const vb = im.tipVertex(h);
      if ((va === 0 && vb === 2) || (va === 2 && vb === 0)) {
        edge02 = e;
        break;
      }
    }

    const before = sit.intrinsicMesh.nVertices;
    const src: SurfacePoint = { kind: 'edge', edge: edge02, t: 1e-12 };
    const dst: SurfacePoint = { kind: 'vertex', vertex: 1 };
    flipOutPathFromSurfacePoints(sit, src, dst);
    // No insertion should have happened — the src snapped to its endpoint.
    expect(sit.intrinsicMesh.nVertices).toBe(before);
  });

  it('cube: face bary (1, 0, 0) snaps to the face corner', () => {
    const sit = buildSit(cube());
    const before = sit.intrinsicMesh.nVertices;
    const src: SurfacePoint = { kind: 'face', face: 0, bary: [1, 0, 0] };
    const dst: SurfacePoint = { kind: 'vertex', vertex: 6 };
    flipOutPathFromSurfacePoints(sit, src, dst);
    expect(sit.intrinsicMesh.nVertices).toBe(before);
  });
});

describe('flipOutPathFromSurfacePoints — refuses degenerate input', () => {
  it('throws if src and dst resolve to the same vertex', () => {
    const sit = buildSit(flatQuad());
    expect(() =>
      flipOutPathFromSurfacePoints(
        sit,
        { kind: 'vertex', vertex: 0 },
        { kind: 'vertex', vertex: 0 },
      ),
    ).toThrow();
  });

  it('throws if src snaps to dst vertex via face bary', () => {
    const sit = buildSit(flatQuad());
    expect(() =>
      flipOutPathFromSurfacePoints(
        sit,
        { kind: 'face', face: 0, bary: [1, 0, 0] },
        { kind: 'vertex', vertex: 0 },
      ),
    ).toThrow();
  });
});

describe('flipOutPathFromSurfacePoints — face-interior to vertex on cube', () => {
  it('cube: face center to opposite-corner vertex returns finite path', () => {
    const sit = buildSit(cube());
    const src: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const dst: SurfacePoint = { kind: 'vertex', vertex: 6 };
    const r = flipOutPathFromSurfacePoints(sit, src, dst);
    expect(r.length).toBeGreaterThan(0);
    expect(Number.isFinite(r.length)).toBe(true);
    expect(r.converged).toBe(true);
    expect(r.polyline.length).toBeGreaterThanOrEqual(2);
  });
});

describe('flipOutPathFromSurfacePoints — face-to-face polyline has > 2 points', () => {
  // These tests exercise the bug we fixed: face-interior to face-interior
  // FlipOut paths used to fall back to a 2-point straight line for the
  // "both endpoints inserted" segment. Now they trace through the input
  // mesh face-by-face.

  function bbox(verts: readonly (readonly [number, number, number])[]): {
    lo: [number, number, number];
    hi: [number, number, number];
  } {
    const lo: [number, number, number] = [Infinity, Infinity, Infinity];
    const hi: [number, number, number] = [-Infinity, -Infinity, -Infinity];
    for (const v of verts) {
      for (let k = 0; k < 3; k++) {
        if (v[k]! < lo[k]!) lo[k] = v[k]!;
        if (v[k]! > hi[k]!) hi[k] = v[k]!;
      }
    }
    return { lo, hi };
  }

  function summedLen(polyline: readonly (readonly number[])[]): number {
    let s = 0;
    for (let i = 1; i < polyline.length; i++) {
      const a = polyline[i - 1]!;
      const b = polyline[i]!;
      s += Math.hypot(b[0]! - a[0]!, b[1]! - a[1]!, b[2]! - a[2]!);
    }
    return s;
  }

  it('cube: face 0 centroid → face 6 centroid has > 2 polyline points', () => {
    const m = cube();
    const sit = buildSit(m);
    const src: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const dst: SurfacePoint = { kind: 'face', face: 6, bary: [1 / 3, 1 / 3, 1 / 3] };
    const r = flipOutPathFromSurfacePoints(sit, src, dst);
    expect(r.polyline.length).toBeGreaterThan(2);
    // polyline length should equal flipout length (within FP)
    expect(summedLen(r.polyline)).toBeCloseTo(r.length, 6);
    // every polyline point lies in the cube's bbox (within tolerance)
    const { lo, hi } = bbox(m.vertices);
    const eps = 1e-6;
    for (const p of r.polyline) {
      for (let k = 0; k < 3; k++) {
        expect(p[k]!).toBeGreaterThanOrEqual(lo[k]! - eps);
        expect(p[k]!).toBeLessThanOrEqual(hi[k]! + eps);
      }
    }
  });

  it('cube: face 4 centroid → face 10 centroid has > 2 polyline points', () => {
    const m = cube();
    const sit = buildSit(m);
    const src: SurfacePoint = { kind: 'face', face: 4, bary: [1 / 3, 1 / 3, 1 / 3] };
    const dst: SurfacePoint = { kind: 'face', face: 10, bary: [1 / 3, 1 / 3, 1 / 3] };
    const r = flipOutPathFromSurfacePoints(sit, src, dst);
    expect(r.polyline.length).toBeGreaterThan(2);
    expect(summedLen(r.polyline)).toBeCloseTo(r.length, 6);
  });

  it('icosahedron: face 0 centroid → face 10 centroid has > 2 polyline points', () => {
    const m = icosahedron();
    const sit = buildSit(m);
    const src: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const dst: SurfacePoint = { kind: 'face', face: 10, bary: [1 / 3, 1 / 3, 1 / 3] };
    const r = flipOutPathFromSurfacePoints(sit, src, dst);
    expect(r.polyline.length).toBeGreaterThan(2);
    expect(summedLen(r.polyline)).toBeCloseTo(r.length, 6);
    // points within icosahedron bbox (radius 1 from origin since the mesh
    // is unit-sphere-inscribed, with some triangle slack)
    const { lo, hi } = bbox(m.vertices);
    const eps = 1e-6;
    for (const p of r.polyline) {
      for (let k = 0; k < 3; k++) {
        expect(p[k]!).toBeGreaterThanOrEqual(lo[k]! - eps);
        expect(p[k]!).toBeLessThanOrEqual(hi[k]! + eps);
      }
    }
  });

  it('flatGrid 5: face-to-face polyline length matches flipout length', () => {
    const m = flatGrid(5, 1);
    const sit = buildSit(m);
    const src: SurfacePoint = { kind: 'face', face: 0, bary: [1 / 3, 1 / 3, 1 / 3] };
    const dst: SurfacePoint = { kind: 'face', face: sit.intrinsicMesh.nFaces - 1, bary: [1 / 3, 1 / 3, 1 / 3] };
    const r = flipOutPathFromSurfacePoints(sit, src, dst);
    expect(r.polyline.length).toBeGreaterThan(2);
    expect(summedLen(r.polyline)).toBeCloseTo(r.length, 6);
  });
});

describe('flipOutPathFromSurfacePoints — SNAP_EPS exported', () => {
  it('SNAP_EPS is a finite positive constant', () => {
    expect(SNAP_EPS).toBeGreaterThan(0);
    expect(Number.isFinite(SNAP_EPS)).toBe(true);
    expect(SNAP_EPS).toBeLessThan(1e-6);
  });
});
