/**
 * Face area, normal, and centroid tests.
 *
 * Hand-computed values on tetrahedron, cube, flat quad, flat grid, plus
 * normal-direction sanity checks (cube faces point outward, flat-quad
 * normal is +z) and unit-length checks.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { dot, norm, sub, type Vec3 } from '../../../src/math/vec3.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import {
  cube,
  flatGrid,
  flatQuad,
  icosahedron,
  singleTriangle,
  tetrahedron,
} from '../../_helpers/meshes.js';

function build(m: { vertices: Vec3[]; faces: readonly (readonly [number, number, number])[] }) {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  return new VertexPositionGeometry(mesh, m.vertices);
}

describe('VertexPositionGeometry.faceArea', () => {
  it('single triangle (3-4-5 in xy plane) has area 0.5', () => {
    const g = build(singleTriangle());
    expect(g.faceArea(0)).toBeCloseTo(0.5, 12);
  });

  it('tetrahedron: all four faces are equilateral with side 2*sqrt(2)', () => {
    const g = build(tetrahedron());
    const s = 2 * Math.sqrt(2);
    const expected = (Math.sqrt(3) / 4) * s * s; // = 2*sqrt(3)
    for (let f = 0; f < g.mesh.nFaces; f++) {
      expect(g.faceArea(f)).toBeCloseTo(expected, 12);
    }
    // Total surface area = 4 * 2*sqrt(3) = 8*sqrt(3).
    let total = 0;
    for (let f = 0; f < g.mesh.nFaces; f++) total += g.faceArea(f);
    expect(total).toBeCloseTo(8 * Math.sqrt(3), 12);
  });

  it('cube: 12 right triangles each with area 1/2, total surface area 6', () => {
    const g = build(cube());
    expect(g.mesh.nFaces).toBe(12);
    let total = 0;
    for (let f = 0; f < g.mesh.nFaces; f++) {
      expect(g.faceArea(f)).toBeCloseTo(0.5, 12);
      total += g.faceArea(f);
    }
    expect(total).toBeCloseTo(6, 12);
  });

  it('flat quad: total area = 1', () => {
    const g = build(flatQuad());
    let total = 0;
    for (let f = 0; f < g.mesh.nFaces; f++) total += g.faceArea(f);
    expect(total).toBeCloseTo(1, 12);
  });

  it('flat grid n=5 size=2: total area = 4', () => {
    const g = build(flatGrid(5, 2));
    let total = 0;
    for (let f = 0; f < g.mesh.nFaces; f++) total += g.faceArea(f);
    expect(total).toBeCloseTo(4, 12);
  });

  it('icosahedron has 20 equal-area faces', () => {
    const g = build(icosahedron());
    expect(g.mesh.nFaces).toBe(20);
    const ref = g.faceArea(0);
    for (let f = 1; f < g.mesh.nFaces; f++) {
      expect(g.faceArea(f)).toBeCloseTo(ref, 12);
    }
  });
});

describe('VertexPositionGeometry.faceNormal', () => {
  it('flat quad normal points in +z direction', () => {
    const g = build(flatQuad());
    for (let f = 0; f < g.mesh.nFaces; f++) {
      const n = g.faceNormal(f);
      expect(n[0]).toBeCloseTo(0, 12);
      expect(n[1]).toBeCloseTo(0, 12);
      expect(n[2]).toBeCloseTo(1, 12);
    }
  });

  it('every face normal is unit length', () => {
    for (const m of [cube(), tetrahedron(), icosahedron(), flatGrid(3)]) {
      const g = build(m);
      for (let f = 0; f < g.mesh.nFaces; f++) {
        expect(norm(g.faceNormal(f))).toBeCloseTo(1, 12);
      }
    }
  });

  it('cube face normals point outward (dot with centroid - center > 0)', () => {
    const g = build(cube());
    const center: Vec3 = [0.5, 0.5, 0.5];
    for (let f = 0; f < g.mesh.nFaces; f++) {
      const n = g.faceNormal(f);
      const c = g.faceCentroid(f);
      const d = dot(n, sub(c, center));
      expect(d).toBeGreaterThan(0);
    }
  });

  it('flat-grid normals all point in +z', () => {
    const g = build(flatGrid(4));
    for (let f = 0; f < g.mesh.nFaces; f++) {
      const n = g.faceNormal(f);
      expect(n[2]).toBeCloseTo(1, 12);
    }
  });

  it('icosahedron face normals point outward (centred at origin)', () => {
    const g = build(icosahedron());
    for (let f = 0; f < g.mesh.nFaces; f++) {
      const n = g.faceNormal(f);
      const c = g.faceCentroid(f);
      // c is roughly along the outward radial direction (origin-centred).
      expect(dot(n, c)).toBeGreaterThan(0);
    }
  });

  it('reversing face winding flips the normal', () => {
    const m = singleTriangle();
    const reversed = {
      vertices: m.vertices,
      faces: m.faces.map((f) => [f[0], f[2], f[1]] as const),
    };
    const g1 = build(m);
    const g2 = build(reversed);
    const n1 = g1.faceNormal(0);
    const n2 = g2.faceNormal(0);
    expect(n2[0]).toBeCloseTo(-n1[0], 12);
    expect(n2[1]).toBeCloseTo(-n1[1], 12);
    expect(n2[2]).toBeCloseTo(-n1[2], 12);
  });
});

describe('VertexPositionGeometry.faceCentroid', () => {
  it('flat quad face 0 = [0,1,2] has centroid (2/3, 1/3, 0)', () => {
    const g = build(flatQuad());
    const c = g.faceCentroid(0);
    expect(c[0]).toBeCloseTo(2 / 3, 12);
    expect(c[1]).toBeCloseTo(1 / 3, 12);
    expect(c[2]).toBeCloseTo(0, 12);
  });

  it('tetrahedron centroid average is the origin (mesh is symmetric)', () => {
    const g = build(tetrahedron());
    let sum: Vec3 = [0, 0, 0];
    for (let f = 0; f < g.mesh.nFaces; f++) {
      const c = g.faceCentroid(f);
      sum = [sum[0] + c[0], sum[1] + c[1], sum[2] + c[2]];
    }
    sum = [sum[0] / g.mesh.nFaces, sum[1] / g.mesh.nFaces, sum[2] / g.mesh.nFaces];
    expect(sum[0]).toBeCloseTo(0, 12);
    expect(sum[1]).toBeCloseTo(0, 12);
    expect(sum[2]).toBeCloseTo(0, 12);
  });
});
