/**
 * Vertex angle sum / defect / Gauss-Bonnet tests.
 *
 * Closed manifolds: total angle defect = 2π · χ.
 *   tetrahedron:  χ = 2 -> total defect = 4π, per-vertex defect = π
 *   cube:         χ = 2 -> total defect = 4π, per-vertex defect = π/2
 *   icosahedron:  χ = 2 -> total defect = 4π, per-vertex defect = π/3
 *
 * Disk topology (flat grid):
 *   χ = 1 -> total defect = 2π. Interior vertices: angle sum = 2π so defect = 0.
 *   Boundary corners contribute via the (π − sum) convention.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { type Vec3 } from '../../../src/math/vec3.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import {
  cube,
  flatGrid,
  flatQuad,
  icosahedron,
  tetrahedron,
} from '../../_helpers/meshes.js';

function build(m: { vertices: Vec3[]; faces: readonly (readonly [number, number, number])[] }) {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  return new VertexPositionGeometry(mesh, m.vertices);
}

describe('VertexPositionGeometry.vertexAngleSum', () => {
  it('tetrahedron: every vertex has angle sum = π (3 equilateral corners)', () => {
    const g = build(tetrahedron());
    for (let v = 0; v < g.mesh.nVertices; v++) {
      expect(g.vertexAngleSum(v)).toBeCloseTo(Math.PI, 12);
    }
  });

  it('icosahedron: every vertex has angle sum = 5π/3 (5 equilateral corners)', () => {
    const g = build(icosahedron());
    for (let v = 0; v < g.mesh.nVertices; v++) {
      expect(g.vertexAngleSum(v)).toBeCloseTo((5 * Math.PI) / 3, 10);
    }
  });

  it('cube: every vertex has angle sum = 3π/2 (six π/4 + π/2 corners distributed)', () => {
    const g = build(cube());
    for (let v = 0; v < g.mesh.nVertices; v++) {
      expect(g.vertexAngleSum(v)).toBeCloseTo((3 * Math.PI) / 2, 12);
    }
  });

  it('flat grid n=3: interior vertex (1,1) has angle sum = 2π', () => {
    const g = build(flatGrid(3));
    const vInterior = 1 * 3 + 1; // row 1, col 1
    expect(g.vertexAngleSum(vInterior)).toBeCloseTo(2 * Math.PI, 12);
  });

  it('flat grid n=4: each interior vertex has angle sum = 2π', () => {
    const g = build(flatGrid(4));
    for (let j = 1; j < 3; j++) {
      for (let i = 1; i < 3; i++) {
        const v = j * 4 + i;
        expect(g.vertexAngleSum(v)).toBeCloseTo(2 * Math.PI, 12);
      }
    }
  });
});

describe('VertexPositionGeometry.vertexAngleDefect', () => {
  it('tetrahedron: every vertex has defect = π', () => {
    const g = build(tetrahedron());
    for (let v = 0; v < g.mesh.nVertices; v++) {
      expect(g.vertexAngleDefect(v)).toBeCloseTo(Math.PI, 12);
    }
  });

  it('cube: every vertex has defect = π/2', () => {
    const g = build(cube());
    for (let v = 0; v < g.mesh.nVertices; v++) {
      expect(g.vertexAngleDefect(v)).toBeCloseTo(Math.PI / 2, 12);
    }
  });

  it('icosahedron: every vertex has defect = π/3', () => {
    const g = build(icosahedron());
    for (let v = 0; v < g.mesh.nVertices; v++) {
      expect(g.vertexAngleDefect(v)).toBeCloseTo(Math.PI / 3, 10);
    }
  });

  it('flat grid n=4: interior vertices have defect = 0', () => {
    const g = build(flatGrid(4));
    for (let j = 1; j < 3; j++) {
      for (let i = 1; i < 3; i++) {
        const v = j * 4 + i;
        expect(g.vertexAngleDefect(v)).toBeCloseTo(0, 12);
      }
    }
  });

  it('flat quad: every corner has angle sum π/2 and defect π/2', () => {
    // Triangulation: [0,1,2] and [0,2,3] (diagonal v0->v2).
    //
    // Per-vertex angle sums (computing each interior corner from positions):
    //   v0=(0,0,0) belongs to both triangles. In [0,1,2] the corner at v0
    //     is between (v1,v2) -> π/4. In [0,2,3] between (v2,v3) -> π/4.
    //     Total π/2.
    //   v2=(1,1,0) similarly: π/4 + π/4 = π/2.
    //   v1=(1,0,0) belongs only to [0,1,2]; the right-angle corner -> π/2.
    //   v3=(0,1,0) belongs only to [0,2,3]; the right-angle corner -> π/2.
    //
    // All four are boundary vertices, so defect = π − π/2 = π/2 each;
    // total = 4 · π/2 = 2π = 2π·χ(=1). ✓
    const g = build(flatQuad());
    for (let v = 0; v < 4; v++) {
      expect(g.mesh.isBoundaryVertex(v)).toBe(true);
      expect(g.vertexAngleSum(v)).toBeCloseTo(Math.PI / 2, 12);
      expect(g.vertexAngleDefect(v)).toBeCloseTo(Math.PI / 2, 12);
    }
  });
});

describe('VertexPositionGeometry.totalAngleDefect (Gauss-Bonnet)', () => {
  it('tetrahedron: total defect = 4π = 2π · χ(=2)', () => {
    const g = build(tetrahedron());
    expect(g.totalAngleDefect()).toBeCloseTo(4 * Math.PI, 12);
    expect(g.mesh.eulerCharacteristic).toBe(2);
  });

  it('cube: total defect = 4π = 2π · χ(=2)', () => {
    const g = build(cube());
    expect(g.totalAngleDefect()).toBeCloseTo(4 * Math.PI, 12);
    expect(g.mesh.eulerCharacteristic).toBe(2);
  });

  it('icosahedron: total defect = 4π = 2π · χ(=2)', () => {
    const g = build(icosahedron());
    expect(g.totalAngleDefect()).toBeCloseTo(4 * Math.PI, 10);
    expect(g.mesh.eulerCharacteristic).toBe(2);
  });

  it('flat quad (disk): total defect = 2π · χ(=1) = 2π', () => {
    const g = build(flatQuad());
    expect(g.totalAngleDefect()).toBeCloseTo(2 * Math.PI, 12);
    expect(g.mesh.eulerCharacteristic).toBe(1);
  });

  it('flat grid n=4: total defect = 2π · χ(=1) = 2π', () => {
    const g = build(flatGrid(4));
    expect(g.totalAngleDefect()).toBeCloseTo(2 * Math.PI, 12);
    expect(g.mesh.eulerCharacteristic).toBe(1);
  });

  it('flat grid n=6: total defect = 2π · χ(=1) = 2π', () => {
    const g = build(flatGrid(6));
    expect(g.totalAngleDefect()).toBeCloseTo(2 * Math.PI, 12);
    expect(g.mesh.eulerCharacteristic).toBe(1);
  });
});
