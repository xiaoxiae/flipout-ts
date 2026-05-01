/**
 * Halfedge indexing invariants.
 *
 * Verifies the algebraic properties that L3+ relies on:
 *   - twin(twin(he)) === he
 *   - on a closed triangle mesh, next³(he) === he
 *   - vertex(twin(he)) === tipVertex(he)
 *   - edge(2*e) === e and edgeHalfedge(e) === 2*e
 *   - implicit-twin convention: twin(he) === he ^ 1
 */

import { describe, expect, it } from 'vitest';

import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import {
  cube,
  flatGrid,
  flatQuad,
  icosahedron,
  singleTriangle,
  tetrahedron,
} from '../../_helpers/meshes.js';

const ALL_MESHES = [
  { name: 'tetrahedron', build: tetrahedron },
  { name: 'cube', build: cube },
  { name: 'icosahedron', build: icosahedron },
  { name: 'flat quad', build: flatQuad },
  { name: 'flat 3x3 grid', build: () => flatGrid(3) },
  { name: 'single triangle', build: singleTriangle },
];

describe('halfedge invariant: twin is an involution', () => {
  for (const { name, build } of ALL_MESHES) {
    it(`${name}: twin(twin(he)) === he for every halfedge`, () => {
      const m = build();
      const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      for (let h = 0; h < mesh.nHalfedges; h++) {
        expect(mesh.twin(mesh.twin(h))).toBe(h);
      }
    });

    it(`${name}: implicit twin rule twin(he) === he ^ 1`, () => {
      const m = build();
      const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      for (let h = 0; h < mesh.nHalfedges; h++) {
        expect(mesh.twin(h)).toBe(h ^ 1);
      }
    });

    it(`${name}: edge mapping is consistent (edge(2k) === k)`, () => {
      const m = build();
      const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      for (let e = 0; e < mesh.nEdges; e++) {
        const h = mesh.edgeHalfedge(e);
        expect(mesh.edge(h)).toBe(e);
        expect(mesh.edge(mesh.twin(h))).toBe(e);
      }
    });
  }
});

describe('halfedge invariant: face cycle of length 3 (closed meshes)', () => {
  const CLOSED = [
    { name: 'tetrahedron', build: tetrahedron },
    { name: 'cube', build: cube },
    { name: 'icosahedron', build: icosahedron },
  ];

  for (const { name, build } of CLOSED) {
    it(`${name}: next(next(next(he))) === he for every interior halfedge`, () => {
      const m = build();
      const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      for (let h = 0; h < mesh.nHalfedges; h++) {
        if (mesh.isBoundaryHalfedge(h)) continue;
        const cycled = mesh.next(mesh.next(mesh.next(h)));
        expect(cycled).toBe(h);
      }
    });
  }
});

describe('halfedge invariant: vertex(twin(he)) === tip(he)', () => {
  for (const { name, build } of ALL_MESHES) {
    it(`${name}: heads and tails are consistent`, () => {
      const m = build();
      const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      for (let h = 0; h < mesh.nHalfedges; h++) {
        // head of he == tail of twin(he)
        expect(mesh.tipVertex(h)).toBe(mesh.vertex(mesh.twin(h)));
        // tip of he == tail of next(he) (standard half-edge invariant)
        expect(mesh.tipVertex(h)).toBe(mesh.vertex(mesh.next(h)));
      }
    });
  }
});

describe('halfedge invariant: face pointer consistency', () => {
  for (const { name, build } of ALL_MESHES) {
    it(`${name}: every halfedge in face(f)'s cycle has face === f`, () => {
      const m = build();
      const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      for (let f = 0; f < mesh.nFaces; f++) {
        for (const h of mesh.halfedgesAroundFace(f)) {
          expect(mesh.face(h)).toBe(f);
        }
      }
    });

    it(`${name}: faceHalfedge(f) returns a halfedge with face === f`, () => {
      const m = build();
      const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      for (let f = 0; f < mesh.nFaces; f++) {
        const h = mesh.faceHalfedge(f);
        expect(mesh.face(h)).toBe(f);
      }
    });

    it(`${name}: vertexHalfedge(v) returns a halfedge with vertex === v`, () => {
      const m = build();
      const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      for (let v = 0; v < mesh.nVertices; v++) {
        const h = mesh.vertexHalfedge(v);
        expect(mesh.vertex(h)).toBe(v);
      }
    });
  }
});

describe('halfedge convention: vertex(he) is the tail (origin)', () => {
  it('first face of the tetrahedron has its halfedges starting at the listed vertices', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    // Face 0 is [0, 1, 2]. The three halfedges of face 0 should have tails
    // 0, 1, 2 in CCW order.
    const tails: number[] = [];
    for (const h of mesh.halfedgesAroundFace(0)) {
      tails.push(mesh.vertex(h));
    }
    expect(tails).toEqual([0, 1, 2]);
  });

  it('tip vertices of face 0 of the tetrahedron are 1, 2, 0 (rotation of tails)', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const tips: number[] = [];
    for (const h of mesh.halfedgesAroundFace(0)) {
      tips.push(mesh.tipVertex(h));
    }
    expect(tips).toEqual([1, 2, 0]);
  });
});
