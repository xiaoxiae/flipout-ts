/**
 * Corner angle tests.
 *
 * Equilateral triangles have all corners π/3; right 3-4-5 has angles
 * arctan(3/4), arctan(4/3), π/2; the three corner angles of any planar
 * triangle sum to π. Boundary halfedges throw.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { type Vec3 } from '../../../src/math/vec3.js';
import { INVALID_INDEX, SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
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

describe('VertexPositionGeometry.cornerAngle', () => {
  it('equilateral triangle: all three corner angles are π/3', () => {
    // Build a single equilateral triangle by hand.
    const s = 2.5;
    const verts: Vec3[] = [
      [0, 0, 0],
      [s, 0, 0],
      [s / 2, (s * Math.sqrt(3)) / 2, 0],
    ];
    const mesh = SurfaceMesh.fromFaces([[0, 1, 2]] as const, 3);
    const g = new VertexPositionGeometry(mesh, verts);
    for (const he of mesh.halfedgesAroundFace(0)) {
      expect(g.cornerAngle(he)).toBeCloseTo(Math.PI / 3, 12);
    }
  });

  it('right triangle 3-4-5: corner angles arctan(3/4), arctan(4/3), π/2', () => {
    // Vertices: A=(0,0,0), B=(3,0,0), C=(0,4,0). Face [A,B,C].
    // Corner at A: between AB and AC -> angle = π/2 (the right angle).
    // Corner at B: between BC (length 5) and BA (length 3) -> arctan(4/3).
    // Corner at C: between CA (length 4) and CB (length 5) -> arctan(3/4).
    const verts: Vec3[] = [
      [0, 0, 0],
      [3, 0, 0],
      [0, 4, 0],
    ];
    const mesh = SurfaceMesh.fromFaces([[0, 1, 2]] as const, 3);
    const g = new VertexPositionGeometry(mesh, verts);

    // Halfedge from face: walk via halfedgesAroundFace and inspect tail.
    const angles = new Map<number, number>(); // tail vertex -> angle
    for (const he of mesh.halfedgesAroundFace(0)) {
      angles.set(mesh.vertex(he), g.cornerAngle(he));
    }
    expect(angles.get(0)!).toBeCloseTo(Math.PI / 2, 12);
    expect(angles.get(1)!).toBeCloseTo(Math.atan2(4, 3), 12);
    expect(angles.get(2)!).toBeCloseTo(Math.atan2(3, 4), 12);
  });

  it('any triangle face: the three corner angles sum to π', () => {
    for (const m of [singleTriangle(), tetrahedron(), cube(), icosahedron(), flatGrid(3)]) {
      const g = build(m);
      for (let f = 0; f < g.mesh.nFaces; f++) {
        let sum = 0;
        for (const he of g.mesh.halfedgesAroundFace(f)) {
          sum += g.cornerAngle(he);
        }
        expect(sum).toBeCloseTo(Math.PI, 11);
      }
    }
  });

  it('tetrahedron: every corner angle is π/3 (each face is equilateral)', () => {
    const g = build(tetrahedron());
    for (let f = 0; f < g.mesh.nFaces; f++) {
      for (const he of g.mesh.halfedgesAroundFace(f)) {
        expect(g.cornerAngle(he)).toBeCloseTo(Math.PI / 3, 12);
      }
    }
  });

  it('icosahedron: every corner angle is π/3 (each face is equilateral)', () => {
    const g = build(icosahedron());
    for (let f = 0; f < g.mesh.nFaces; f++) {
      for (const he of g.mesh.halfedgesAroundFace(f)) {
        expect(g.cornerAngle(he)).toBeCloseTo(Math.PI / 3, 10);
      }
    }
  });

  it('cube triangulated faces: corner angles are {π/2, π/4, π/4} per face', () => {
    const g = build(cube());
    for (let f = 0; f < g.mesh.nFaces; f++) {
      const angles: number[] = [];
      for (const he of g.mesh.halfedgesAroundFace(f)) {
        angles.push(g.cornerAngle(he));
      }
      angles.sort((a, b) => a - b);
      expect(angles[0]!).toBeCloseTo(Math.PI / 4, 12);
      expect(angles[1]!).toBeCloseTo(Math.PI / 4, 12);
      expect(angles[2]!).toBeCloseTo(Math.PI / 2, 12);
    }
  });

  it('throws on a boundary halfedge', () => {
    const g = build(flatQuad());
    // Find a boundary halfedge.
    let boundaryHe = -1;
    for (let h = 0; h < g.mesh.nHalfedges; h++) {
      if (g.mesh.face(h) === INVALID_INDEX) {
        boundaryHe = h;
        break;
      }
    }
    expect(boundaryHe).toBeGreaterThanOrEqual(0);
    expect(() => g.cornerAngle(boundaryHe)).toThrow(/boundary/);
  });
});
