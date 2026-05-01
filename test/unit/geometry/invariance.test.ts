/**
 * Symmetry / invariance tests:
 *
 *   - Rigid motion (rotation + translation) preserves edge lengths,
 *     face areas, corner angles, and angle defects.
 *   - Uniform scaling by k scales lengths by k, areas by k^2, and leaves
 *     angles untouched.
 *   - Construction validates that positions.length matches mesh.nVertices.
 *   - Spot-check that the cube factory matches the position list saved in
 *     the `cube-edge.json` fixture (proves L2 ↔ fixture coupling).
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { type Vec3 } from '../../../src/math/vec3.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { loadFixture } from '../../_helpers/load-fixture.js';
import {
  cube,
  flatGrid,
  icosahedron,
  singleTriangle,
  tetrahedron,
} from '../../_helpers/meshes.js';

function build(m: { vertices: Vec3[]; faces: readonly (readonly [number, number, number])[] }) {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  return new VertexPositionGeometry(mesh, m.vertices);
}

/** Apply a 3x3 rotation matrix `R` followed by translation `t`. */
function rigidTransform(p: Vec3, R: readonly number[], t: Vec3): Vec3 {
  const x = R[0]! * p[0] + R[1]! * p[1] + R[2]! * p[2] + t[0];
  const y = R[3]! * p[0] + R[4]! * p[1] + R[5]! * p[2] + t[1];
  const z = R[6]! * p[0] + R[7]! * p[1] + R[8]! * p[2] + t[2];
  return [x, y, z];
}

/** Build a non-trivial rotation matrix via three Euler angles. Determinant +1, columns orthonormal. */
function rotationMatrix(yaw: number, pitch: number, roll: number): number[] {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  // R = Rz(yaw) * Ry(pitch) * Rx(roll); standard ZYX order.
  return [
    cy * cp,
    cy * sp * sr - sy * cr,
    cy * sp * cr + sy * sr,
    sy * cp,
    sy * sp * sr + cy * cr,
    sy * sp * cr - cy * sr,
    -sp,
    cp * sr,
    cp * cr,
  ];
}

const MESHES: Array<{ name: string; data: () => { vertices: Vec3[]; faces: readonly (readonly [number, number, number])[] } }> = [
  { name: 'tetrahedron', data: tetrahedron },
  { name: 'cube', data: cube },
  { name: 'icosahedron', data: icosahedron },
  { name: 'flatGrid(4)', data: () => flatGrid(4) },
  { name: 'singleTriangle', data: singleTriangle },
];

describe('VertexPositionGeometry: rigid-motion invariance', () => {
  const R = rotationMatrix(0.7, -1.1, 0.3);
  const t: Vec3 = [3.5, -2.25, 0.1];

  for (const { name, data } of MESHES) {
    it(`${name}: edge lengths invariant under rigid motion`, () => {
      const m = data();
      const original = build(m);
      const transformed = build({
        vertices: m.vertices.map((p) => rigidTransform(p, R, t)),
        faces: m.faces,
      });
      for (let e = 0; e < original.mesh.nEdges; e++) {
        expect(transformed.edgeLength(e)).toBeCloseTo(original.edgeLength(e), 11);
      }
    });

    it(`${name}: face areas invariant under rigid motion`, () => {
      const m = data();
      const original = build(m);
      const transformed = build({
        vertices: m.vertices.map((p) => rigidTransform(p, R, t)),
        faces: m.faces,
      });
      for (let f = 0; f < original.mesh.nFaces; f++) {
        expect(transformed.faceArea(f)).toBeCloseTo(original.faceArea(f), 11);
      }
    });

    it(`${name}: corner angles invariant under rigid motion`, () => {
      const m = data();
      const original = build(m);
      const transformed = build({
        vertices: m.vertices.map((p) => rigidTransform(p, R, t)),
        faces: m.faces,
      });
      for (let h = 0; h < original.mesh.nHalfedges; h++) {
        if (original.mesh.face(h) < 0) continue;
        expect(transformed.cornerAngle(h)).toBeCloseTo(original.cornerAngle(h), 11);
      }
    });

    it(`${name}: vertex angle defect invariant under rigid motion`, () => {
      const m = data();
      const original = build(m);
      const transformed = build({
        vertices: m.vertices.map((p) => rigidTransform(p, R, t)),
        faces: m.faces,
      });
      for (let v = 0; v < original.mesh.nVertices; v++) {
        expect(transformed.vertexAngleDefect(v)).toBeCloseTo(original.vertexAngleDefect(v), 11);
      }
    });
  }
});

describe('VertexPositionGeometry: uniform scaling', () => {
  const k = 2.5;

  it('icosahedron scaled by k: edge lengths scale by k', () => {
    const m = icosahedron();
    const original = build(m);
    const scaled = build({
      vertices: m.vertices.map((p) => [k * p[0], k * p[1], k * p[2]] as Vec3),
      faces: m.faces,
    });
    for (let e = 0; e < original.mesh.nEdges; e++) {
      expect(scaled.edgeLength(e)).toBeCloseTo(k * original.edgeLength(e), 10);
    }
  });

  it('icosahedron scaled by k: face areas scale by k^2', () => {
    const m = icosahedron();
    const original = build(m);
    const scaled = build({
      vertices: m.vertices.map((p) => [k * p[0], k * p[1], k * p[2]] as Vec3),
      faces: m.faces,
    });
    for (let f = 0; f < original.mesh.nFaces; f++) {
      expect(scaled.faceArea(f)).toBeCloseTo(k * k * original.faceArea(f), 10);
    }
  });

  it('cube scaled by k: corner angles unchanged', () => {
    const m = cube();
    const original = build(m);
    const scaled = build({
      vertices: m.vertices.map((p) => [k * p[0], k * p[1], k * p[2]] as Vec3),
      faces: m.faces,
    });
    for (let h = 0; h < original.mesh.nHalfedges; h++) {
      if (original.mesh.face(h) < 0) continue;
      expect(scaled.cornerAngle(h)).toBeCloseTo(original.cornerAngle(h), 12);
    }
  });

  it('cube scaled by k: vertex angle defect unchanged (angles are scale-invariant)', () => {
    const m = cube();
    const original = build(m);
    const scaled = build({
      vertices: m.vertices.map((p) => [k * p[0], k * p[1], k * p[2]] as Vec3),
      faces: m.faces,
    });
    for (let v = 0; v < original.mesh.nVertices; v++) {
      expect(scaled.vertexAngleDefect(v)).toBeCloseTo(original.vertexAngleDefect(v), 12);
    }
  });
});

describe('VertexPositionGeometry: construction & validation', () => {
  it('constructor stores mesh and positions verbatim', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const g = new VertexPositionGeometry(mesh, m.vertices);
    expect(g.mesh).toBe(mesh);
    expect(g.positions).toBe(m.vertices);
    expect(g.positions.length).toBe(mesh.nVertices);
  });

  it('throws when positions array length disagrees with mesh.nVertices', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(() => new VertexPositionGeometry(mesh, m.vertices.slice(0, 3))).toThrow(/positions/);
    expect(
      () => new VertexPositionGeometry(mesh, [...m.vertices, [0, 0, 0]] as Vec3[]),
    ).toThrow(/positions/);
  });

  it('position(v) throws on out-of-range index', () => {
    const g = build(tetrahedron());
    expect(() => g.position(-1)).toThrow();
    expect(() => g.position(g.mesh.nVertices)).toThrow();
  });

  it('cube vertex coordinates match cube-edge.json fixture (L2 / fixture parity)', () => {
    const m = cube();
    const fixture = loadFixture('cube-edge');
    expect(m.vertices.length).toBe(fixture.mesh.vertices.length);
    for (let i = 0; i < m.vertices.length; i++) {
      const ours = m.vertices[i]!;
      const theirs = fixture.mesh.vertices[i]!;
      expect(ours[0]).toBeCloseTo(theirs[0], 12);
      expect(ours[1]).toBeCloseTo(theirs[1], 12);
      expect(ours[2]).toBeCloseTo(theirs[2], 12);
    }
  });
});
