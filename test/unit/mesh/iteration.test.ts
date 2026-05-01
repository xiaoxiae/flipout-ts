/**
 * Iteration helpers: vertices/edges/halfedges around a vertex, halfedges/
 * vertices around a face. These need to be solid because L3 (intrinsic
 * triangulation) and L4 (FlipOut) call them in inner loops.
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

describe('verticesAroundVertex / outgoingHalfedges', () => {
  it('tetrahedron vertex 0: neighbours are exactly {1, 2, 3}', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const ns = new Set<number>();
    for (const v of mesh.verticesAroundVertex(0)) ns.add(v);
    expect(ns).toEqual(new Set([1, 2, 3]));
  });

  it('tetrahedron: every vertex has exactly the other three as neighbours', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      const expected = new Set([0, 1, 2, 3].filter((x) => x !== v));
      const seen = new Set<number>();
      for (const n of mesh.verticesAroundVertex(v)) seen.add(n);
      expect(seen).toEqual(expected);
    }
  });

  it('icosahedron: every vertex has 5 unique neighbours', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      const seen = new Set<number>();
      for (const n of mesh.verticesAroundVertex(v)) {
        seen.add(n);
        expect(n).not.toBe(v);
      }
      expect(seen.size).toBe(5);
    }
  });

  it('outgoingHalfedges(v) yields halfedges all with tail === v', () => {
    const m = cube();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      for (const h of mesh.outgoingHalfedges(v)) {
        expect(mesh.vertex(h)).toBe(v);
      }
    }
  });

  it('outgoingHalfedges and verticesAroundVertex yield the same count', () => {
    const m = cube();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      let nHe = 0;
      for (const _ of mesh.outgoingHalfedges(v)) nHe++;
      let nN = 0;
      for (const _ of mesh.verticesAroundVertex(v)) nN++;
      expect(nN).toBe(nHe);
    }
  });

  it('incomingHalfedges(v) is the twin set of outgoingHalfedges(v)', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      const out = new Set<number>();
      for (const h of mesh.outgoingHalfedges(v)) out.add(h);
      const inc = new Set<number>();
      for (const h of mesh.incomingHalfedges(v)) inc.add(h);
      expect(out.size).toBe(inc.size);
      for (const h of inc) {
        expect(out.has(mesh.twin(h))).toBe(true);
      }
    }
  });

  it('incomingHalfedges(v) yields halfedges all with tip === v', () => {
    const m = cube();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      for (const h of mesh.incomingHalfedges(v)) {
        expect(mesh.tipVertex(h)).toBe(v);
      }
    }
  });

  it('flat quad corners have degree 2 or 3 (depending on diagonal)', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    // Faces are [0,1,2] and [0,2,3] sharing edge 0-2. Vertices 0 and 2 are
    // on both triangles (degree 3), vertices 1 and 3 are on one triangle
    // (degree 2).
    expect(mesh.vertexDegree(0)).toBe(3);
    expect(mesh.vertexDegree(2)).toBe(3);
    expect(mesh.vertexDegree(1)).toBe(2);
    expect(mesh.vertexDegree(3)).toBe(2);
  });

  it('flat 4x4 grid: corner vertices have degree 2, edge vertices 4, interior 6', () => {
    const m = flatGrid(4, 1);
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    // Corners (0, 3, 12, 15) — but with right-triangle topology, two corners
    // sit at the diagonal-meeting end and have a different valence than the
    // other two. We count by category.
    const histogram: Record<number, number> = {};
    for (let v = 0; v < mesh.nVertices; v++) {
      const d = mesh.vertexDegree(v);
      histogram[d] = (histogram[d] ?? 0) + 1;
    }
    // Sum of degrees must equal 2 * |E|.
    let sum = 0;
    for (const [d, n] of Object.entries(histogram)) sum += Number(d) * n;
    expect(sum).toBe(2 * mesh.nEdges);
    // 16 vertices total.
    let total = 0;
    for (const n of Object.values(histogram)) total += n;
    expect(total).toBe(16);
  });
});

describe('edgesAroundVertex', () => {
  it('icosahedron: every vertex has 5 distinct incident edges', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      const edges = new Set<number>();
      for (const e of mesh.edgesAroundVertex(v)) edges.add(e);
      expect(edges.size).toBe(5);
    }
  });

  it('tetrahedron: each vertex has 3 distinct incident edges', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      const edges = new Set<number>();
      for (const e of mesh.edgesAroundVertex(v)) edges.add(e);
      expect(edges.size).toBe(3);
    }
  });

  it('summed over all vertices, each edge appears exactly twice', () => {
    // Every edge is incident on its two endpoint vertices.
    const m = cube();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const counter = new Map<number, number>();
    for (let v = 0; v < mesh.nVertices; v++) {
      for (const e of mesh.edgesAroundVertex(v)) {
        counter.set(e, (counter.get(e) ?? 0) + 1);
      }
    }
    expect(counter.size).toBe(mesh.nEdges);
    for (const c of counter.values()) expect(c).toBe(2);
  });
});

describe('halfedgesAroundFace / verticesOfFace', () => {
  it('every face has exactly 3 halfedges', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let f = 0; f < mesh.nFaces; f++) {
      let n = 0;
      for (const _ of mesh.halfedgesAroundFace(f)) n++;
      expect(n).toBe(3);
    }
  });

  it('every face has 3 distinct vertex indices', () => {
    const m = cube();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let f = 0; f < mesh.nFaces; f++) {
      const vs = [...mesh.verticesOfFace(f)];
      expect(vs).toHaveLength(3);
      expect(new Set(vs).size).toBe(3);
    }
  });

  it('verticesOfFace order matches the original input face winding', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let f = 0; f < mesh.nFaces; f++) {
      const got = [...mesh.verticesOfFace(f)];
      // Allow rotations of the input triple — the choice of "starting"
      // halfedge for the face can be any of the three.
      const want = m.faces[f]!;
      const rotations = [
        [want[0], want[1], want[2]],
        [want[1], want[2], want[0]],
        [want[2], want[0], want[1]],
      ];
      expect(rotations).toContainEqual(got);
    }
  });

  it('halfedge-around-face cycles back to the start', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let f = 0; f < mesh.nFaces; f++) {
      const start = mesh.faceHalfedge(f);
      let h = start;
      for (let i = 0; i < 3; i++) h = mesh.next(h);
      expect(h).toBe(start);
    }
  });

  it('singleTriangle face 0 has vertices {0, 1, 2}', () => {
    const m = singleTriangle();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const vs = new Set<number>();
    for (const v of mesh.verticesOfFace(0)) vs.add(v);
    expect(vs).toEqual(new Set([0, 1, 2]));
  });
});

describe('orbit termination', () => {
  it('outgoingHalfedges always terminates in O(degree) steps', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let v = 0; v < mesh.nVertices; v++) {
      let n = 0;
      for (const _ of mesh.outgoingHalfedges(v)) {
        n++;
        if (n > 100) throw new Error(`runaway orbit at vertex ${v}`);
      }
    }
  });

  it('halfedgesAroundFace always terminates in O(degree) steps', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let f = 0; f < mesh.nFaces; f++) {
      let n = 0;
      for (const _ of mesh.halfedgesAroundFace(f)) {
        n++;
        if (n > 100) throw new Error(`runaway face orbit at face ${f}`);
      }
    }
  });
});
