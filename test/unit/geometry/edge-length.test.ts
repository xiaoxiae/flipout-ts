/**
 * Edge / halfedge length tests.
 *
 * Covers symmetry (`edgeLength(e) == halfedgeLength(twin(he))`), agreement
 * with the L0 `distance` primitive, and exact values on tetrahedron, cube,
 * flat quad, and flat grid (hand-computed).
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { distance, type Vec3 } from '../../../src/math/vec3.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { cube, flatGrid, flatQuad, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

function build(m: { vertices: Vec3[]; faces: readonly (readonly [number, number, number])[] }) {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  return new VertexPositionGeometry(mesh, m.vertices);
}

describe('VertexPositionGeometry.edgeLength', () => {
  it('tetrahedron (cube-corner variant) has all six edges of length 2*sqrt(2)', () => {
    const g = build(tetrahedron());
    const expected = 2 * Math.sqrt(2);
    for (let e = 0; e < g.mesh.nEdges; e++) {
      expect(g.edgeLength(e)).toBeCloseTo(expected, 12);
    }
  });

  it('cube edges: 12 axis edges (length 1) and 6 face-diagonal edges (length sqrt(2))', () => {
    const g = build(cube());
    let nAxis = 0;
    let nDiag = 0;
    for (let e = 0; e < g.mesh.nEdges; e++) {
      const len = g.edgeLength(e);
      if (Math.abs(len - 1) < 1e-12) nAxis++;
      else if (Math.abs(len - Math.sqrt(2)) < 1e-12) nDiag++;
      else throw new Error(`unexpected edge length ${len}`);
    }
    expect(nAxis).toBe(12);
    expect(nDiag).toBe(6);
  });

  it('flat quad: 4 axis edges (length 1) and 1 diagonal edge (length sqrt(2))', () => {
    const g = build(flatQuad());
    let nAxis = 0;
    let nDiag = 0;
    for (let e = 0; e < g.mesh.nEdges; e++) {
      const len = g.edgeLength(e);
      if (Math.abs(len - 1) < 1e-12) nAxis++;
      else if (Math.abs(len - Math.sqrt(2)) < 1e-12) nDiag++;
    }
    expect(nAxis).toBe(4);
    expect(nDiag).toBe(1);
  });

  it('flat grid n=4 size=1: axis edges length 1/3, diagonal edges length sqrt(2)/3', () => {
    const g = build(flatGrid(4, 1));
    let nAxis = 0;
    let nDiag = 0;
    for (let e = 0; e < g.mesh.nEdges; e++) {
      const len = g.edgeLength(e);
      if (Math.abs(len - 1 / 3) < 1e-12) nAxis++;
      else if (Math.abs(len - Math.sqrt(2) / 3) < 1e-12) nDiag++;
      else throw new Error(`grid: unexpected edge length ${len}`);
    }
    // n=4 grid: 2*n*(n-1) axis edges = 24, (n-1)^2 diagonals = 9.
    expect(nAxis).toBe(24);
    expect(nDiag).toBe(9);
  });

  it('halfedgeLength agrees with edgeLength for both halfedges of every edge', () => {
    const g = build(icosahedron());
    for (let e = 0; e < g.mesh.nEdges; e++) {
      const h = g.mesh.edgeHalfedge(e);
      const t = g.mesh.twin(h);
      const le = g.edgeLength(e);
      expect(g.halfedgeLength(h)).toBeCloseTo(le, 12);
      expect(g.halfedgeLength(t)).toBeCloseTo(le, 12);
    }
  });

  it('halfedgeLength matches the L0 distance primitive on raw positions', () => {
    const m = icosahedron();
    const g = build(m);
    for (let h = 0; h < g.mesh.nHalfedges; h++) {
      const tail = m.vertices[g.mesh.tailVertex(h)]!;
      const tip = m.vertices[g.mesh.tipVertex(h)]!;
      expect(g.halfedgeLength(h)).toBeCloseTo(distance(tail, tip), 12);
    }
  });

  it('icosahedron: all 30 edges have the same length (regular polyhedron)', () => {
    const g = build(icosahedron());
    const ref = g.edgeLength(0);
    for (let e = 1; e < g.mesh.nEdges; e++) {
      expect(g.edgeLength(e)).toBeCloseTo(ref, 12);
    }
  });

  it('halfedgeVector returns tip - tail and is anti-symmetric in the twin', () => {
    const g = build(cube());
    for (let h = 0; h < g.mesh.nHalfedges; h++) {
      const v = g.halfedgeVector(h);
      const vt = g.halfedgeVector(g.mesh.twin(h));
      expect(v[0]).toBeCloseTo(-vt[0], 12);
      expect(v[1]).toBeCloseTo(-vt[1], 12);
      expect(v[2]).toBeCloseTo(-vt[2], 12);
    }
  });
});
