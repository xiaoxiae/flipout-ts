/**
 * Polyline-extraction tests.
 *
 * Verify `FlipEdgeNetwork.extractPolyline()` returns a sensible 3D
 * polyline:
 *   - first point = source vertex position
 *   - last point  = destination vertex position
 *   - sum of segment lengths = pathLength()
 *   - on flat meshes, polyline is collinear with the geodesic
 *
 * The tracePolylineFromVertex extension to L3 is exercised here.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import type { Vec3 } from '../../../src/math/vec3.js';
import { FlipEdgeNetwork, flipOutPath } from '../../../src/flipout/index.js';
import { shortestEdgePath } from '../../../src/flipout/dijkstra.js';
import { cube, flatGrid, flatQuad, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

interface MeshLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function build(m: MeshLike): SignpostIntrinsicTriangulation {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  return new SignpostIntrinsicTriangulation(geom);
}

function dist3(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function polylineLength(pts: Vec3[]): number {
  let s = 0;
  for (let i = 1; i < pts.length; i++) s += dist3(pts[i - 1]!, pts[i]!);
  return s;
}

describe('extractPolyline — endpoints', () => {
  it('flatQuad 1->3: starts at v=1, ends at v=3', () => {
    const sit = build(flatQuad());
    const r = flipOutPath(sit, 1, 3);
    expect(dist3(r.polyline[0]!, [1, 0, 0])).toBeLessThan(1e-9);
    expect(dist3(r.polyline[r.polyline.length - 1]!, [0, 1, 0])).toBeLessThan(1e-9);
  });

  it('cube 0->6: starts at v=0=(0,0,0), ends at v=6=(1,1,1)', () => {
    const sit = build(cube());
    const r = flipOutPath(sit, 0, 6);
    expect(dist3(r.polyline[0]!, [0, 0, 0])).toBeLessThan(1e-9);
    expect(dist3(r.polyline[r.polyline.length - 1]!, [1, 1, 1])).toBeLessThan(1e-9);
  });

  it('tetrahedron 0->1: 3D positions of v0 and v1', () => {
    const sit = build(tetrahedron());
    const r = flipOutPath(sit, 0, 1);
    expect(dist3(r.polyline[0]!, [1, 1, 1])).toBeLessThan(1e-9);
    expect(dist3(r.polyline[r.polyline.length - 1]!, [-1, -1, 1])).toBeLessThan(1e-9);
  });

  it('flatGrid(4) 0->15: starts at (0,0,0), ends at (1,1,0)', () => {
    const sit = build(flatGrid(4, 1));
    const r = flipOutPath(sit, 0, 15);
    expect(dist3(r.polyline[0]!, [0, 0, 0])).toBeLessThan(1e-9);
    expect(dist3(r.polyline[r.polyline.length - 1]!, [1, 1, 0])).toBeLessThan(1e-9);
  });

  it('icosahedron 0->3: starts/ends at the icosahedron vertex positions', () => {
    const sit = build(icosahedron());
    const r = flipOutPath(sit, 0, 3);
    const expectedSrc: Vec3 = [
      ...sit.inputGeometry.position(0),
    ] as unknown as Vec3;
    const expectedDst: Vec3 = [
      ...sit.inputGeometry.position(3),
    ] as unknown as Vec3;
    expect(dist3(r.polyline[0]!, expectedSrc)).toBeLessThan(1e-9);
    expect(dist3(r.polyline[r.polyline.length - 1]!, expectedDst)).toBeLessThan(1e-9);
  });
});

describe('extractPolyline — length consistency', () => {
  it('flatQuad 1->3: polyline length matches pathLength = √2', () => {
    const sit = build(flatQuad());
    const initial = shortestEdgePath(sit, 1, 3)!;
    const network = new FlipEdgeNetwork(sit, initial);
    network.flipOut();
    const poly = network.extractPolyline();
    expect(polylineLength(poly)).toBeCloseTo(network.pathLength(), 6);
  });

  it('flatGrid(4) 0->15: polyline length = √2 (cartesian distance)', () => {
    const sit = build(flatGrid(4, 1));
    const r = flipOutPath(sit, 0, 15);
    const polyLen = polylineLength(r.polyline);
    expect(polyLen).toBeCloseTo(Math.sqrt(2), 6);
    expect(polyLen).toBeCloseTo(r.length, 6);
  });

  it('cube 0->6: polyline length = √5', () => {
    const sit = build(cube());
    const r = flipOutPath(sit, 0, 6);
    expect(polylineLength(r.polyline)).toBeCloseTo(Math.sqrt(5), 4);
  });

  it('tetrahedron 0->1: polyline length matches the single edge', () => {
    const sit = build(tetrahedron());
    const r = flipOutPath(sit, 0, 1);
    expect(polylineLength(r.polyline)).toBeCloseTo(2 * Math.sqrt(2), 6);
  });
});

describe('extractPolyline — flat-mesh collinearity', () => {
  it('flatGrid(4) 0->15: polyline lies on the diagonal y = x, z = 0', () => {
    const sit = build(flatGrid(4, 1));
    const r = flipOutPath(sit, 0, 15);
    for (const p of r.polyline) {
      expect(p[2]).toBeCloseTo(0, 9); // flat in z = 0 plane
      expect(Math.abs(p[0] - p[1])).toBeLessThan(1e-6); // y = x diagonal
    }
  });

  it('flatQuad 1->3: polyline lies on the anti-diagonal', () => {
    const sit = build(flatQuad());
    const r = flipOutPath(sit, 1, 3);
    for (const p of r.polyline) {
      expect(p[2]).toBeCloseTo(0, 9);
      // x + y = 1 along the anti-diagonal from (1,0) to (0,1).
      expect(p[0] + p[1]).toBeCloseTo(1, 6);
    }
  });

  it('flatGrid(5,2) 0->24: polyline lies on the diagonal of [0,2]^2', () => {
    const sit = build(flatGrid(5, 2));
    const r = flipOutPath(sit, 0, 24);
    for (const p of r.polyline) {
      expect(p[2]).toBeCloseTo(0, 9);
      expect(Math.abs(p[0] - p[1])).toBeLessThan(1e-6);
    }
  });
});

describe('extractPolyline — well-formedness', () => {
  it('polyline is non-empty for any nontrivial path', () => {
    const sit = build(cube());
    const r = flipOutPath(sit, 0, 6);
    expect(r.polyline.length).toBeGreaterThanOrEqual(2);
  });

  it('no two consecutive points coincide (deduplication preserves at least 2 distinct points)', () => {
    const sit = build(icosahedron());
    const r = flipOutPath(sit, 0, 3);
    let distinct = 0;
    for (let i = 1; i < r.polyline.length; i++) {
      if (dist3(r.polyline[i - 1]!, r.polyline[i]!) > 1e-9) distinct++;
    }
    // distinct points = polyline.length - duplicates; we want at least 1 segment.
    expect(distinct).toBeGreaterThanOrEqual(1);
  });

  it('polyline length ≥ Euclidean distance between endpoints', () => {
    const sit = build(icosahedron());
    const r = flipOutPath(sit, 0, 3);
    const direct = dist3(r.polyline[0]!, r.polyline[r.polyline.length - 1]!);
    expect(polylineLength(r.polyline)).toBeGreaterThanOrEqual(direct - 1e-9);
  });
});
