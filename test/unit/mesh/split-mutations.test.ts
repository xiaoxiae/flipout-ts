/**
 * Vertex insertion mutations: `splitEdgeTriangular` and `splitFace`.
 *
 * Ported from gc tests around `ManifoldSurfaceMesh::splitEdgeTriangular`
 * and `ManifoldSurfaceMesh::insertVertex` (face-vertex insertion).
 *
 * The combinatorial deltas, manifold invariants, and Euler-characteristic
 * preservation are all verified here. Geometry (lengths, signposts,
 * positions) is handled by L3 — these tests only inspect connectivity.
 */

import { describe, expect, it } from 'vitest';

import { INVALID_INDEX, SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { cube, flatGrid, flatQuad, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

interface MeshData {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}

function build(m: MeshData): SurfaceMesh {
  return SurfaceMesh.fromFaces(m.faces, m.vertices.length);
}

function findInteriorEdge(mesh: SurfaceMesh): number {
  for (let e = 0; e < mesh.nEdges; e++) {
    if (!mesh.isBoundaryEdge(e)) return e;
  }
  return -1;
}

function findBoundaryEdge(mesh: SurfaceMesh): number {
  for (let e = 0; e < mesh.nEdges; e++) {
    if (mesh.isBoundaryEdge(e)) return e;
  }
  return -1;
}

function checkHalfedgeInvariants(mesh: SurfaceMesh): void {
  for (let h = 0; h < mesh.nHalfedges; h++) {
    expect(mesh.twin(mesh.twin(h))).toBe(h);
    expect(mesh.tipVertex(h)).toBe(mesh.vertex(mesh.next(h)));
    if (!mesh.isBoundaryHalfedge(h)) {
      // Triangle face: next^3 = identity.
      expect(mesh.next(mesh.next(mesh.next(h)))).toBe(h);
    }
  }
}

function checkVertexHalfedgeAnchors(mesh: SurfaceMesh): void {
  for (let v = 0; v < mesh.nVertices; v++) {
    const h = mesh.vertexHalfedge(v);
    expect(mesh.vertex(h)).toBe(v);
  }
}

function checkFaceHalfedgeAnchors(mesh: SurfaceMesh): void {
  for (let f = 0; f < mesh.nFaces; f++) {
    const h = mesh.faceHalfedge(f);
    expect(mesh.face(h)).toBe(f);
  }
}

function vertexDegrees(mesh: SurfaceMesh): number[] {
  const d: number[] = [];
  for (let v = 0; v < mesh.nVertices; v++) d.push(mesh.vertexDegree(v));
  return d;
}

describe('splitEdgeTriangular — element-count deltas', () => {
  it.each<[string, MeshData]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['flatGrid 4x4', flatGrid(4, 1)],
  ])('%s: interior edge → +1V, +3E, +2F, +6H', (_, m) => {
    const mesh = build(m);
    const e = findInteriorEdge(mesh);
    expect(e).toBeGreaterThanOrEqual(0);
    const nV = mesh.nVertices;
    const nE = mesh.nEdges;
    const nF = mesh.nFaces;
    const nH = mesh.nHalfedges;
    mesh.splitEdgeTriangular(e);
    expect(mesh.nVertices).toBe(nV + 1);
    expect(mesh.nEdges).toBe(nE + 3);
    expect(mesh.nFaces).toBe(nF + 2);
    expect(mesh.nHalfedges).toBe(nH + 6);
  });

  it('flatQuad: boundary edge → +1V, +2E, +1F, +4H', () => {
    const mesh = build(flatQuad());
    const e = findBoundaryEdge(mesh);
    expect(e).toBeGreaterThanOrEqual(0);
    const nV = mesh.nVertices;
    const nE = mesh.nEdges;
    const nF = mesh.nFaces;
    const nH = mesh.nHalfedges;
    mesh.splitEdgeTriangular(e);
    expect(mesh.nVertices).toBe(nV + 1);
    expect(mesh.nEdges).toBe(nE + 2);
    expect(mesh.nFaces).toBe(nF + 1);
    expect(mesh.nHalfedges).toBe(nH + 4);
  });
});

describe('splitEdgeTriangular — manifold invariants are preserved', () => {
  it.each<[string, MeshData]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['flatGrid 3x3', flatGrid(3, 1)],
    ['flatQuad', flatQuad()],
  ])('%s: twin/next/face invariants hold post-split', (_, m) => {
    const mesh = build(m);
    const e = findInteriorEdge(mesh);
    if (e < 0) return; // nothing to test on a boundary-only mesh
    mesh.splitEdgeTriangular(e);
    checkHalfedgeInvariants(mesh);
    checkVertexHalfedgeAnchors(mesh);
    checkFaceHalfedgeAnchors(mesh);
    // Handshake lemma.
    let sumDeg = 0;
    for (let v = 0; v < mesh.nVertices; v++) sumDeg += mesh.vertexDegree(v);
    expect(sumDeg).toBe(2 * mesh.nEdges);
  });
});

describe('splitEdgeTriangular — Euler characteristic preserved', () => {
  it('cube (closed, χ=2): split preserves χ', () => {
    const mesh = build(cube());
    expect(mesh.eulerCharacteristic).toBe(2);
    const e = findInteriorEdge(mesh);
    mesh.splitEdgeTriangular(e);
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('icosahedron (closed, χ=2): split preserves χ', () => {
    const mesh = build(icosahedron());
    expect(mesh.eulerCharacteristic).toBe(2);
    mesh.splitEdgeTriangular(findInteriorEdge(mesh));
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('flatGrid 4x4 (disk-topology, χ=1): split preserves χ', () => {
    const mesh = build(flatGrid(4, 1));
    expect(mesh.eulerCharacteristic).toBe(1);
    mesh.splitEdgeTriangular(findInteriorEdge(mesh));
    expect(mesh.eulerCharacteristic).toBe(1);
  });

  it('flatQuad: boundary split preserves χ=1', () => {
    const mesh = build(flatQuad());
    expect(mesh.eulerCharacteristic).toBe(1);
    mesh.splitEdgeTriangular(findBoundaryEdge(mesh));
    expect(mesh.eulerCharacteristic).toBe(1);
  });
});

describe('splitEdgeTriangular — vertex degrees', () => {
  it('flatQuad interior diagonal: endpoints unchanged degree, opposites +1, new vertex deg 4', () => {
    const mesh = build(flatQuad());
    // Locate the interior edge first; record its endpoints + opposite vertices.
    const e = findInteriorEdge(mesh);
    const heA = mesh.edgeHalfedge(e);
    const heB = mesh.twin(heA);
    const va = mesh.vertex(heA);
    const vb = mesh.tipVertex(heA);
    const vc = mesh.vertex(mesh.next(mesh.next(heA))); // opposite in face A
    const vd = mesh.vertex(mesh.next(mesh.next(heB))); // opposite in face B

    const d0 = vertexDegrees(mesh);
    const { newVertex: vNew } = mesh.splitEdgeTriangular(e);
    const d1 = vertexDegrees(mesh);

    // Endpoints (va, vb): one original edge replaced by an edge to vNew —
    // degree unchanged.
    expect(d1[va]).toBe(d0[va]);
    expect(d1[vb]).toBe(d0[vb]);
    // Opposites (vc, vd): each gains a new edge to vNew.
    expect(d1[vc]).toBe(d0[vc]! + 1);
    expect(d1[vd]).toBe(d0[vd]! + 1);
    // New vertex degree = 4 (interior edge case).
    expect(d1[vNew]).toBe(4);
    // All other vertices unchanged.
    for (let v = 0; v < d0.length; v++) {
      if (v === va || v === vb || v === vc || v === vd) continue;
      expect(d1[v]).toBe(d0[v]);
    }
  });

  it('flatQuad: boundary edge split — new vertex degree 3', () => {
    const mesh = build(flatQuad());
    const e = findBoundaryEdge(mesh);
    const { newVertex: vNew } = mesh.splitEdgeTriangular(e);
    expect(mesh.vertexDegree(vNew)).toBe(3);
  });

  it('icosahedron: split returns 4 outgoing halfedges from the new vertex', () => {
    const mesh = build(icosahedron());
    const e = findInteriorEdge(mesh);
    const { newVertex, newHalfedgesFromNew } = mesh.splitEdgeTriangular(e);
    expect(newHalfedgesFromNew.length).toBe(4);
    for (const he of newHalfedgesFromNew) {
      expect(mesh.vertex(he)).toBe(newVertex);
    }
  });
});

describe('splitEdgeTriangular — boundary edge handling', () => {
  it('flatQuad boundary: returns 3 outgoing halfedges (2 interior + 1 boundary)', () => {
    const mesh = build(flatQuad());
    const e = findBoundaryEdge(mesh);
    const { newHalfedgesFromNew, newVertex } = mesh.splitEdgeTriangular(e);
    expect(newHalfedgesFromNew.length).toBe(3);
    // Exactly one boundary outgoing halfedge.
    let nBoundary = 0;
    for (const he of newHalfedgesFromNew) {
      expect(mesh.vertex(he)).toBe(newVertex);
      if (mesh.isBoundaryHalfedge(he)) nBoundary++;
    }
    expect(nBoundary).toBe(1);
  });
});

describe('splitFace — element-count deltas', () => {
  it.each<[string, MeshData]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['flatGrid 4x4', flatGrid(4, 1)],
    ['flatQuad', flatQuad()],
  ])('%s: insert into face 0 → +1V, +3E, +2F, +6H', (_, m) => {
    const mesh = build(m);
    const nV = mesh.nVertices;
    const nE = mesh.nEdges;
    const nF = mesh.nFaces;
    const nH = mesh.nHalfedges;
    mesh.splitFace(0);
    expect(mesh.nVertices).toBe(nV + 1);
    expect(mesh.nEdges).toBe(nE + 3);
    expect(mesh.nFaces).toBe(nF + 2);
    expect(mesh.nHalfedges).toBe(nH + 6);
  });
});

describe('splitFace — manifold invariants and degrees', () => {
  it.each<[string, MeshData]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['flatQuad', flatQuad()],
  ])('%s: twin/next invariants and handshake hold post-split', (_, m) => {
    const mesh = build(m);
    mesh.splitFace(0);
    checkHalfedgeInvariants(mesh);
    checkVertexHalfedgeAnchors(mesh);
    checkFaceHalfedgeAnchors(mesh);
    let sumDeg = 0;
    for (let v = 0; v < mesh.nVertices; v++) sumDeg += mesh.vertexDegree(v);
    expect(sumDeg).toBe(2 * mesh.nEdges);
  });

  it('cube: new vertex has degree 3', () => {
    const mesh = build(cube());
    const { newVertex } = mesh.splitFace(0);
    expect(mesh.vertexDegree(newVertex)).toBe(3);
  });

  it('flatQuad: surrounding face vertices each gain +1 degree', () => {
    const mesh = build(flatQuad());
    const f = 0;
    const corners = [...mesh.verticesOfFace(f)];
    const before = corners.map((v) => mesh.vertexDegree(v));
    mesh.splitFace(f);
    const after = corners.map((v) => mesh.vertexDegree(v));
    for (let i = 0; i < 3; i++) {
      expect(after[i]).toBe(before[i]! + 1);
    }
  });

  it('icosahedron: split returns 3 outgoing halfedges from the new vertex, all interior', () => {
    const mesh = build(icosahedron());
    const { newVertex, newHalfedgesFromNew } = mesh.splitFace(0);
    expect(newHalfedgesFromNew.length).toBe(3);
    for (const he of newHalfedgesFromNew) {
      expect(mesh.vertex(he)).toBe(newVertex);
      expect(mesh.face(he)).not.toBe(INVALID_INDEX);
    }
  });
});

describe('splitFace — Euler characteristic preserved', () => {
  it('cube (closed, χ=2): face split preserves χ', () => {
    const mesh = build(cube());
    expect(mesh.eulerCharacteristic).toBe(2);
    mesh.splitFace(0);
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('flatGrid 4x4 (disk, χ=1): face split preserves χ', () => {
    const mesh = build(flatGrid(4, 1));
    expect(mesh.eulerCharacteristic).toBe(1);
    mesh.splitFace(0);
    expect(mesh.eulerCharacteristic).toBe(1);
  });
});

describe('splitFace — three new triangles fan from the new vertex', () => {
  it('flatQuad: each outgoing halfedge from new vertex bounds a triangle face', () => {
    const mesh = build(flatQuad());
    const { newVertex, newHalfedgesFromNew } = mesh.splitFace(0);
    expect(newHalfedgesFromNew.length).toBe(3);
    for (const he of newHalfedgesFromNew) {
      const f = mesh.face(he);
      expect(f).not.toBe(INVALID_INDEX);
      expect(mesh.faceDegree(f)).toBe(3);
      // The face contains the new vertex.
      const verts = [...mesh.verticesOfFace(f)];
      expect(verts).toContain(newVertex);
    }
  });
});

describe('splitEdgeTriangular + flipEdge interplay', () => {
  it('icosahedron: flipping a non-incident edge after a split still preserves invariants', () => {
    const mesh = build(icosahedron());
    const e = findInteriorEdge(mesh);
    mesh.splitEdgeTriangular(e);
    // Find some edge that's still flippable (interior, not yet incident to
    // the new vertex). Just try them all and pick the first that flips.
    let flipped = false;
    for (let ee = 0; ee < mesh.nEdges; ee++) {
      const h = mesh.edgeHalfedge(ee);
      if (mesh.isBoundaryHalfedge(h) || mesh.isBoundaryHalfedge(mesh.twin(h))) continue;
      if (mesh.flipEdge(ee)) {
        flipped = true;
        break;
      }
    }
    expect(flipped).toBe(true);
    checkHalfedgeInvariants(mesh);
  });
});
