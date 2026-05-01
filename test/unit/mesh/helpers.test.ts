/**
 * Sanity tests for the shared mesh factories. The factories are consumed by
 * other test files (and will be by L2/L3) so getting them wrong is expensive.
 *
 * Where possible we cross-check vertex layouts against the JSON fixtures
 * produced by `tools/gen_fixtures.py`.
 */

import { describe, expect, it } from 'vitest';

import {
  cube,
  flatGrid,
  flatQuad,
  icosahedron,
  singleTriangle,
  tetrahedron,
  twoDisjointTriangles,
} from '../../_helpers/meshes.js';
import { loadFixture } from '../../_helpers/load-fixture.js';

describe('tetrahedron()', () => {
  it('produces 4 vertices and 4 faces', () => {
    const m = tetrahedron();
    expect(m.vertices).toHaveLength(4);
    expect(m.faces).toHaveLength(4);
  });

  it('matches fixtures/tetrahedron-edge.json::mesh.vertices', () => {
    const m = tetrahedron();
    const fx = loadFixture('tetrahedron-edge');
    expect(fx.mesh.vertices).toEqual(m.vertices);
    expect(fx.mesh.faces).toEqual(m.faces);
  });

  it('all six edges have side length 2*sqrt(2)', () => {
    const m = tetrahedron();
    const expected = 2 * Math.sqrt(2);
    const pairs: [number, number][] = [
      [0, 1],
      [0, 2],
      [0, 3],
      [1, 2],
      [1, 3],
      [2, 3],
    ];
    for (const [a, b] of pairs) {
      const va = m.vertices[a]!;
      const vb = m.vertices[b]!;
      const d = Math.hypot(va[0] - vb[0], va[1] - vb[1], va[2] - vb[2]);
      expect(d).toBeCloseTo(expected, 12);
    }
  });
});

describe('cube()', () => {
  it('produces 8 vertices and 12 faces', () => {
    const m = cube();
    expect(m.vertices).toHaveLength(8);
    expect(m.faces).toHaveLength(12);
  });

  it('matches fixtures/cube-edge.json::mesh.vertices', () => {
    const m = cube();
    const fx = loadFixture('cube-edge');
    expect(fx.mesh.vertices).toEqual(m.vertices);
    expect(fx.mesh.faces).toEqual(m.faces);
  });

  it('every vertex is on the unit cube [0,1]^3', () => {
    const m = cube();
    for (const [x, y, z] of m.vertices) {
      expect([0, 1]).toContain(x);
      expect([0, 1]).toContain(y);
      expect([0, 1]).toContain(z);
    }
  });
});

describe('icosahedron()', () => {
  it('produces 12 vertices and 20 faces', () => {
    const m = icosahedron();
    expect(m.vertices).toHaveLength(12);
    expect(m.faces).toHaveLength(20);
  });

  it('matches fixtures/icosahedron-edge.json::mesh.vertices', () => {
    const m = icosahedron();
    const fx = loadFixture('icosahedron-edge');
    // Allow tiny FP drift from the projection.
    expect(fx.mesh.vertices).toHaveLength(m.vertices.length);
    for (let i = 0; i < m.vertices.length; i++) {
      const got = m.vertices[i]!;
      const want = fx.mesh.vertices[i]!;
      for (let k = 0; k < 3; k++) expect(got[k]).toBeCloseTo(want[k]!, 12);
    }
    expect(fx.mesh.faces).toEqual(m.faces);
  });

  it('every vertex lies on the unit sphere', () => {
    const m = icosahedron();
    for (const v of m.vertices) {
      const n = Math.hypot(v[0], v[1], v[2]);
      expect(n).toBeCloseTo(1, 12);
    }
  });
});

describe('flatQuad()', () => {
  it('produces 4 vertices and 2 faces', () => {
    const m = flatQuad();
    expect(m.vertices).toHaveLength(4);
    expect(m.faces).toHaveLength(2);
  });

  it('matches fixtures/quad-edge.json::mesh', () => {
    const m = flatQuad();
    const fx = loadFixture('quad-edge');
    expect(fx.mesh.vertices).toEqual(m.vertices);
    expect(fx.mesh.faces).toEqual(m.faces);
  });
});

describe('flatGrid()', () => {
  it('default 4x4 produces 16 vertices and 18 faces', () => {
    const m = flatGrid(4, 1);
    expect(m.vertices).toHaveLength(16);
    expect(m.faces).toHaveLength(18);
  });

  it('matches fixtures/grid-edge.json::mesh', () => {
    const m = flatGrid(4, 1);
    const fx = loadFixture('grid-edge');
    expect(fx.mesh.vertices).toEqual(m.vertices);
    expect(fx.mesh.faces).toEqual(m.faces);
  });

  it('rejects n < 2', () => {
    expect(() => flatGrid(1)).toThrow(/>= 2/);
    expect(() => flatGrid(0)).toThrow(/>= 2/);
  });

  it('rejects non-integer n', () => {
    expect(() => flatGrid(3.5)).toThrow(/integer/);
  });

  it('size parameter scales the grid uniformly', () => {
    const m = flatGrid(2, 5);
    // 2x2 corners: (0,0,0), (5,0,0), (0,5,0), (5,5,0)
    expect(m.vertices).toEqual([
      [0, 0, 0],
      [5, 0, 0],
      [0, 5, 0],
      [5, 5, 0],
    ]);
  });
});

describe('singleTriangle() / twoDisjointTriangles()', () => {
  it('singleTriangle has 3 vertices and 1 face', () => {
    const m = singleTriangle();
    expect(m.vertices).toHaveLength(3);
    expect(m.faces).toHaveLength(1);
    expect(m.faces[0]).toEqual([0, 1, 2]);
  });

  it('twoDisjointTriangles has 6 vertices and 2 faces with no shared indices', () => {
    const m = twoDisjointTriangles();
    expect(m.vertices).toHaveLength(6);
    expect(m.faces).toHaveLength(2);
    const v0 = new Set(m.faces[0]);
    const v1 = new Set(m.faces[1]);
    for (const v of v0) expect(v1.has(v)).toBe(false);
  });
});
