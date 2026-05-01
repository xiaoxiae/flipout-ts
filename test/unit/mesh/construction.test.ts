/**
 * SurfaceMesh construction.
 *
 * Builds the standard fixtures via `SurfaceMesh.fromFaces` and asserts element
 * counts, Euler characteristic, and basic accessor sanity. Non-manifold input
 * detection is also exercised here.
 */

import { describe, expect, it } from 'vitest';

import { INVALID_INDEX, SurfaceMesh, type Triangle } from '../../../src/mesh/surface-mesh.js';
import {
  cube,
  flatGrid,
  flatQuad,
  icosahedron,
  singleTriangle,
  tetrahedron,
  twoDisjointTriangles,
} from '../../_helpers/meshes.js';

describe('SurfaceMesh.fromFaces — closed sphere-topology meshes', () => {
  it('tetrahedron: 4 vertices, 4 faces, 6 edges, χ = 2', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(mesh.nVertices).toBe(4);
    expect(mesh.nFaces).toBe(4);
    expect(mesh.nEdges).toBe(6);
    expect(mesh.nHalfedges).toBe(12);
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('cube (12 triangles): 8 vertices, 12 faces, 18 edges, χ = 2', () => {
    const m = cube();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(mesh.nVertices).toBe(8);
    expect(mesh.nFaces).toBe(12);
    expect(mesh.nEdges).toBe(18);
    expect(mesh.nHalfedges).toBe(36);
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('icosahedron: 12 vertices, 20 faces, 30 edges, χ = 2', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(mesh.nVertices).toBe(12);
    expect(mesh.nFaces).toBe(20);
    expect(mesh.nEdges).toBe(30);
    expect(mesh.nHalfedges).toBe(60);
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('icosahedron: every vertex has degree 5', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      expect(mesh.vertexDegree(v)).toBe(5);
    }
  });

  it('tetrahedron: every vertex has degree 3', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      expect(mesh.vertexDegree(v)).toBe(3);
    }
  });

  it('cube: every vertex has degree 4 or 5 (depends on triangulation)', () => {
    // For the gen_fixtures triangulation, the corners (0,0,0), (1,1,1),
    // (0,1,0), (1,0,1) will have one degree, and (1,0,0), (0,1,1),
    // (0,0,1), (1,1,0) the other. We just check sum and bounds.
    const m = cube();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    let total = 0;
    for (let v = 0; v < mesh.nVertices; v++) {
      const d = mesh.vertexDegree(v);
      expect(d).toBeGreaterThanOrEqual(4);
      expect(d).toBeLessThanOrEqual(6);
      total += d;
    }
    // Handshake lemma: sum of degrees = 2 * nEdges.
    expect(total).toBe(2 * mesh.nEdges);
  });
});

describe('SurfaceMesh.fromFaces — disk-topology / boundary meshes', () => {
  it('single triangle: 3 vertices, 1 face, 3 edges, χ = 1', () => {
    const m = singleTriangle();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(mesh.nVertices).toBe(3);
    expect(mesh.nFaces).toBe(1);
    expect(mesh.nEdges).toBe(3);
    expect(mesh.nHalfedges).toBe(6);
    expect(mesh.eulerCharacteristic).toBe(1);
  });

  it('flat quad: 4 vertices, 2 faces, 5 edges, χ = 1', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(mesh.nVertices).toBe(4);
    expect(mesh.nFaces).toBe(2);
    expect(mesh.nEdges).toBe(5);
    expect(mesh.nHalfedges).toBe(10);
    expect(mesh.eulerCharacteristic).toBe(1);
  });

  it('single triangle: every halfedge is on the boundary or interior consistently', () => {
    const m = singleTriangle();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    let interior = 0;
    let boundary = 0;
    for (let h = 0; h < mesh.nHalfedges; h++) {
      if (mesh.isBoundaryHalfedge(h)) boundary++;
      else interior++;
    }
    expect(interior).toBe(3);
    expect(boundary).toBe(3);
  });

  it('flat quad: 3 boundary edges along the outer perimeter — wait, 4', () => {
    // Quad outer boundary: 4 edges. Inner shared edge is interior (1). Total 5
    // edges, of which 4 are boundary.
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    let boundaryEdges = 0;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) boundaryEdges++;
    }
    expect(boundaryEdges).toBe(4);
  });

  it('flat quad: all 4 corner vertices are on the boundary', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < 4; v++) {
      expect(mesh.isBoundaryVertex(v)).toBe(true);
    }
  });

  it('flat 4x4 grid: 16 vertices, 18 triangles, 33 edges, χ = 1', () => {
    const m = flatGrid(4, 1);
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(mesh.nVertices).toBe(16);
    expect(mesh.nFaces).toBe(18);
    // χ = 1 for a disk: V - E + F = 16 - 33 + 18 = 1
    expect(mesh.eulerCharacteristic).toBe(1);
    expect(mesh.nEdges).toBe(33);
  });

  it('flat 4x4 grid: 12 boundary vertices (the ring), 4 interior', () => {
    const m = flatGrid(4, 1);
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    let boundary = 0;
    let interior = 0;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (mesh.isBoundaryVertex(v)) boundary++;
      else interior++;
    }
    // Boundary ring of 4x4 grid = 12; interior = 4.
    expect(boundary).toBe(12);
    expect(interior).toBe(4);
  });

  it('flat 4x4 grid: handshake lemma holds', () => {
    const m = flatGrid(4, 1);
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    let sum = 0;
    for (let v = 0; v < mesh.nVertices; v++) sum += mesh.vertexDegree(v);
    expect(sum).toBe(2 * mesh.nEdges);
  });
});

describe('SurfaceMesh.fromFaces — disconnected components', () => {
  it('two disjoint triangles: builds successfully, χ = 2', () => {
    const m = twoDisjointTriangles();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(mesh.nVertices).toBe(6);
    expect(mesh.nFaces).toBe(2);
    expect(mesh.nEdges).toBe(6);
    // Two disks, χ = 2 * 1 = 2.
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('two disjoint triangles: each component has its own halfedges', () => {
    const m = twoDisjointTriangles();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const verts0 = new Set<number>();
    for (const v of mesh.verticesOfFace(0)) verts0.add(v);
    const verts1 = new Set<number>();
    for (const v of mesh.verticesOfFace(1)) verts1.add(v);
    // No shared vertex.
    for (const v of verts0) expect(verts1.has(v)).toBe(false);
  });
});

describe('SurfaceMesh.fromFaces — error cases', () => {
  it('rejects a vertex index out of range', () => {
    const faces: Triangle[] = [[0, 1, 5]];
    expect(() => SurfaceMesh.fromFaces(faces, 3)).toThrow(/outside \[0, 3\)/);
  });

  it('rejects a face with a repeated vertex (self-edge)', () => {
    const faces: Triangle[] = [[0, 1, 1]];
    expect(() => SurfaceMesh.fromFaces(faces, 3)).toThrow(/self-edge/);
  });

  it('rejects three triangles sharing edge 0-1 (non-manifold edge)', () => {
    // Vertices 0, 1, 2, 3, 4 — all three faces try to claim directed edge
    // (0,1) or (1,0) for themselves.
    const faces: Triangle[] = [
      [0, 1, 2],
      [0, 1, 3],
      [0, 1, 4],
    ];
    expect(() => SurfaceMesh.fromFaces(faces, 5)).toThrow(/non-manifold|already claimed/);
  });

  it('rejects a duplicate face (same orientation)', () => {
    const faces: Triangle[] = [
      [0, 1, 2],
      [0, 1, 2],
    ];
    expect(() => SurfaceMesh.fromFaces(faces, 3)).toThrow();
  });

  it('rejects negative vertexCount', () => {
    expect(() => SurfaceMesh.fromFaces([[0, 1, 2]], -1)).toThrow(/non-negative/);
  });

  it('rejects non-integer vertexCount', () => {
    expect(() => SurfaceMesh.fromFaces([[0, 1, 2]], 3.5)).toThrow(/non-negative/);
  });

  it('rejects a non-manifold "hourglass" vertex', () => {
    // Two triangle fans share vertex 0 only at a point — disconnected fans
    // around vertex 0. Vertex 0 is touched by faces (0,1,2) and (0,3,4) but
    // not connected by any shared edge.
    const faces: Triangle[] = [
      [0, 1, 2],
      [0, 3, 4],
    ];
    // The two fans don't share an edge through vertex 0, so vertex 0 has
    // disconnected outgoing-halfedge orbits.
    expect(() => SurfaceMesh.fromFaces(faces, 5)).toThrow(/non-manifold|disconnected/);
  });
});

describe('SurfaceMesh accessor bounds', () => {
  it('throws on out-of-range halfedge index', () => {
    const m = singleTriangle();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(() => mesh.vertex(99)).toThrow(/out of range/);
    expect(() => mesh.next(-1)).toThrow(/out of range/);
    expect(() => mesh.face(99)).toThrow(/out of range/);
  });

  it('throws on out-of-range vertex index', () => {
    const m = singleTriangle();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(() => mesh.vertexHalfedge(99)).toThrow(/out of range/);
  });

  it('throws on out-of-range face index', () => {
    const m = singleTriangle();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(() => mesh.faceHalfedge(99)).toThrow(/out of range/);
  });

  it('flipEdge throws on out-of-range edge index', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    expect(() => mesh.flipEdge(99)).toThrow(/out of range/);
  });
});

describe('SurfaceMesh.INVALID_INDEX', () => {
  it('is exported as -1', () => {
    expect(INVALID_INDEX).toBe(-1);
  });

  it('matches the boundary face sentinel', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    let saw = false;
    for (let h = 0; h < mesh.nHalfedges; h++) {
      if (mesh.face(h) === INVALID_INDEX) saw = true;
    }
    expect(saw).toBe(true);
  });
});
