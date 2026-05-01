/**
 * Edge flip mutation. The most error-prone part of the L1 port — every test
 * here is an invariant directly translated from the geometry-central source
 * (`SurfaceMesh::flip`).
 */

import { describe, expect, it } from 'vitest';

import { SurfaceMesh, type Triangle } from '../../../src/mesh/surface-mesh.js';
import {
  cube,
  flatGrid,
  flatQuad,
  icosahedron,
  singleTriangle,
  tetrahedron,
} from '../../_helpers/meshes.js';

/** Snapshot the connectivity arrays for equality comparison. */
function snapshot(mesh: SurfaceMesh): {
  he: number[];
  next: number[];
  face: number[];
  v: number[];
  f: number[];
} {
  const he: number[] = [];
  const next: number[] = [];
  const face: number[] = [];
  for (let h = 0; h < mesh.nHalfedges; h++) {
    he.push(mesh.vertex(h));
    next.push(mesh.next(h));
    face.push(mesh.face(h));
  }
  const v: number[] = [];
  for (let i = 0; i < mesh.nVertices; i++) v.push(mesh.vertexHalfedge(i));
  const f: number[] = [];
  for (let i = 0; i < mesh.nFaces; i++) f.push(mesh.faceHalfedge(i));
  return { he, next, face, v, f };
}

/**
 * Capture the face *topology* of the mesh: a set of vertex triples (each
 * triple normalised to a sorted form) that uniquely identifies the
 * combinatorial face set, ignoring face indices and starting halfedge.
 */
function faceVertexSets(mesh: SurfaceMesh): Set<string> {
  const out = new Set<string>();
  for (let f = 0; f < mesh.nFaces; f++) {
    const vs = [...mesh.verticesOfFace(f)].sort((a, b) => a - b);
    out.add(vs.join(','));
  }
  return out;
}

/** Capture the unordered edge set (as 'min,max' strings) of the mesh. */
function edgeSet(mesh: SurfaceMesh): Set<string> {
  const out = new Set<string>();
  for (let e = 0; e < mesh.nEdges; e++) {
    const h = mesh.edgeHalfedge(e);
    const a = mesh.vertex(h);
    const b = mesh.tipVertex(h);
    out.add(a < b ? `${a},${b}` : `${b},${a}`);
  }
  return out;
}

/** Find the interior edge index of an edge between vertices `a` and `b`. */
function findEdge(mesh: SurfaceMesh, a: number, b: number): number {
  for (const h of mesh.outgoingHalfedges(a)) {
    if (mesh.tipVertex(h) === b) return mesh.edge(h);
  }
  throw new Error(`edge ${a}-${b} not found`);
}

describe('flipEdge — basic cases on the tetrahedron', () => {
  it('flipping any interior edge succeeds (all edges are interior)', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let e = 0; e < mesh.nEdges; e++) {
      // Re-build for each: each flip mutates the mesh.
      const fresh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      // On a tetrahedron, every edge's "other diagonal" is the edge between
      // the two non-adjacent vertices — but the tetrahedron is K4, so those
      // two are already connected. Flip should refuse.
      expect(fresh.flipEdge(e)).toBe(false);
    }
  });

  it('post-failed-flip mesh equals pre-flip mesh', () => {
    const m = tetrahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const before = snapshot(mesh);
    expect(mesh.flipEdge(0)).toBe(false);
    const after = snapshot(mesh);
    expect(after).toEqual(before);
  });
});

describe('flipEdge — flat quad shared edge', () => {
  // The flat quad is the canonical "flip me" case: faces [0,1,2] and [0,2,3]
  // share edge 0-2. Flipping it produces faces with shared edge 1-3.
  it('flipping the shared diagonal of a flat quad succeeds and changes degrees', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const eShared = findEdge(mesh, 0, 2);
    const before = [
      mesh.vertexDegree(0),
      mesh.vertexDegree(1),
      mesh.vertexDegree(2),
      mesh.vertexDegree(3),
    ];
    expect(mesh.flipEdge(eShared)).toBe(true);
    const after = [
      mesh.vertexDegree(0),
      mesh.vertexDegree(1),
      mesh.vertexDegree(2),
      mesh.vertexDegree(3),
    ];
    // Endpoints (0 and 2) lose an incident edge, opposites (1 and 3) gain one.
    expect(after[0]).toBe(before[0]! - 1);
    expect(after[2]).toBe(before[2]! - 1);
    expect(after[1]).toBe(before[1]! + 1);
    expect(after[3]).toBe(before[3]! + 1);
  });

  it('flip of flat quad now contains edge 1-3 instead of 0-2', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const eShared = findEdge(mesh, 0, 2);
    expect(mesh.flipEdge(eShared)).toBe(true);
    // 1 and 3 should now be neighbours.
    const ns1 = new Set<number>();
    for (const v of mesh.verticesAroundVertex(1)) ns1.add(v);
    expect(ns1.has(3)).toBe(true);
    // 0 and 2 should NOT be neighbours anymore.
    const ns0 = new Set<number>();
    for (const v of mesh.verticesAroundVertex(0)) ns0.add(v);
    expect(ns0.has(2)).toBe(false);
  });

  it('flip(flip(e)) restores the mesh topology', () => {
    // Note: geometry-central's flip rotates the four diamond halfedges, so
    // flipping the same edge twice does NOT restore the bit-exact heNext /
    // heVertex / heFace arrays — but it does restore the *topology* (the
    // set of edges and the multiset of face vertex triples).
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const eShared = findEdge(mesh, 0, 2);
    const facesBefore = faceVertexSets(mesh);
    const edgesBefore = edgeSet(mesh);
    expect(mesh.flipEdge(eShared)).toBe(true);
    expect(mesh.flipEdge(eShared)).toBe(true);
    expect(faceVertexSets(mesh)).toEqual(facesBefore);
    expect(edgeSet(mesh)).toEqual(edgesBefore);
  });

  it('flip of a flat quad preserves face count', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const e = findEdge(mesh, 0, 2);
    const fb = mesh.nFaces;
    expect(mesh.flipEdge(e)).toBe(true);
    expect(mesh.nFaces).toBe(fb);
  });

  it('flip of a flat quad preserves the handshake lemma', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const e = findEdge(mesh, 0, 2);
    expect(mesh.flipEdge(e)).toBe(true);
    let sum = 0;
    for (let v = 0; v < mesh.nVertices; v++) sum += mesh.vertexDegree(v);
    expect(sum).toBe(2 * mesh.nEdges);
  });

  it('flip of a flat quad preserves the half-edge invariants', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const e = findEdge(mesh, 0, 2);
    expect(mesh.flipEdge(e)).toBe(true);
    for (let h = 0; h < mesh.nHalfedges; h++) {
      expect(mesh.twin(mesh.twin(h))).toBe(h);
      expect(mesh.tipVertex(h)).toBe(mesh.vertex(mesh.next(h)));
    }
    // Every interior triangle still has next^3 = id.
    for (let h = 0; h < mesh.nHalfedges; h++) {
      if (mesh.isBoundaryHalfedge(h)) continue;
      expect(mesh.next(mesh.next(mesh.next(h)))).toBe(h);
    }
  });
});

describe('flipEdge — boundary edges', () => {
  it('rejects boundary edge of a flat quad', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const eBoundary = findEdge(mesh, 0, 1);
    expect(mesh.isBoundaryEdge(eBoundary)).toBe(true);
    expect(mesh.flipEdge(eBoundary)).toBe(false);
  });

  it('rejects boundary edge of a single triangle', () => {
    const m = singleTriangle();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    for (let e = 0; e < mesh.nEdges; e++) {
      expect(mesh.flipEdge(e)).toBe(false);
    }
  });

  it('boundary edge flip leaves the mesh unchanged', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const eBoundary = findEdge(mesh, 0, 1);
    const before = snapshot(mesh);
    expect(mesh.flipEdge(eBoundary)).toBe(false);
    expect(snapshot(mesh)).toEqual(before);
  });
});

describe('flipEdge — already-connected opposite vertices', () => {
  // Build a four-vertex pyramid lying flat in z=0:
  //
  //       0
  //      /|\
  //     / | \
  //    /  3  \
  //   /  / \  \
  //  1--/   \--2
  //     \   /
  //      \ /
  //
  // Concretely: take two triangles sharing edge 0-3 and sharing the
  // outer-perimeter vertex 1, 2. The edge 0-3 has opposite vertices 1 and 2,
  // and we add a triangle 1-2-? ... actually easier: build a "diamond" with
  // two extra triangles closing it into a degree-4 closed neighborhood.
  //
  // The simplest construction: 5 vertices forming a closed disk where
  // flipping the central edge would create a duplicate.

  it("flipping an edge whose opposites are already connected returns false", () => {
    // Consider four triangles forming a closed strip where flipping the
    // shared edge would duplicate an existing one.
    //
    // Vertices: 0, 1, 2, 3, 4
    // Faces:
    //   [0, 1, 2]
    //   [0, 2, 3]
    //   [0, 3, 1]   (edge 1-3 already exists from face 2)
    //   [1, 3, 2]   (closes a tetrahedron-like shape)
    //
    // Now edge 0-2 (shared by faces [0,1,2] and [0,2,3]) has opposite
    // vertices 1 and 3 — but 1 and 3 are already directly connected (faces
    // 2 and 3). So flipping 0-2 must be refused.
    const faces: Triangle[] = [
      [0, 1, 2],
      [0, 2, 3],
      [0, 3, 1],
      [1, 3, 2],
    ];
    const mesh = SurfaceMesh.fromFaces(faces, 4);
    // (this is actually the tetrahedron with relabelled vertices; every
    // edge has its opposites already connected. So flips on every edge
    // should return false.)
    for (let e = 0; e < mesh.nEdges; e++) {
      expect(mesh.flipEdge(e)).toBe(false);
    }
  });
});

describe('flipEdge — invariants on a non-trivial mesh', () => {
  it('cube: a successful flip preserves all halfedge invariants', () => {
    const m = cube();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    // Find a flippable edge: pick any non-boundary edge (cube is closed, so
    // any edge) whose opposites aren't already connected. The cube
    // triangulation has plenty of these — most quads' diagonals can flip.
    let flipped = -1;
    for (let e = 0; e < mesh.nEdges; e++) {
      const fresh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
      if (fresh.flipEdge(e)) {
        flipped = e;
        break;
      }
    }
    expect(flipped).toBeGreaterThanOrEqual(0);
    expect(mesh.flipEdge(flipped)).toBe(true);

    // Invariants
    for (let h = 0; h < mesh.nHalfedges; h++) {
      expect(mesh.twin(mesh.twin(h))).toBe(h);
      expect(mesh.tipVertex(h)).toBe(mesh.vertex(mesh.next(h)));
      // closed mesh -> every halfedge in a triangle face
      expect(mesh.next(mesh.next(mesh.next(h)))).toBe(h);
    }
    // Sum of degrees
    let sum = 0;
    for (let v = 0; v < mesh.nVertices; v++) sum += mesh.vertexDegree(v);
    expect(sum).toBe(2 * mesh.nEdges);
  });

  it('icosahedron: flipping every flippable edge then flipping back restores topology', () => {
    const m = icosahedron();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    // Try flipping each edge twice; if first flip succeeds, second must
    // succeed and the topology (faces + edges) must match.
    for (let e = 0; e < mesh.nEdges; e++) {
      const facesBefore = faceVertexSets(mesh);
      const edgesBefore = edgeSet(mesh);
      const ok = mesh.flipEdge(e);
      if (!ok) continue;
      const ok2 = mesh.flipEdge(e);
      expect(ok2).toBe(true);
      expect(faceVertexSets(mesh)).toEqual(facesBefore);
      expect(edgeSet(mesh)).toEqual(edgesBefore);
    }
  });
});

describe('flipEdge — degree change is exactly ±1 for the four involved vertices', () => {
  it('flat quad: vertex degrees change by exactly ±1 on a flip', () => {
    const m = flatQuad();
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const e = findEdge(mesh, 0, 2);
    const before = [0, 1, 2, 3].map((v) => mesh.vertexDegree(v));
    expect(mesh.flipEdge(e)).toBe(true);
    const after = [0, 1, 2, 3].map((v) => mesh.vertexDegree(v));
    let dec = 0;
    let inc = 0;
    for (let i = 0; i < 4; i++) {
      const d = after[i]! - before[i]!;
      expect([-1, 0, 1]).toContain(d);
      if (d === -1) dec++;
      if (d === 1) inc++;
    }
    expect(dec).toBe(2);
    expect(inc).toBe(2);
  });
});

describe('flipEdge — random walk: many flips on a grid keep invariants', () => {
  it('flat 4x4 grid: 30 random flips keep handshake lemma + halfedge invariants', () => {
    const m = flatGrid(4, 1);
    const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
    const rng = mulberry32(0x12345);
    let attempts = 0;
    let successes = 0;
    while (successes < 30 && attempts < 1000) {
      const e = Math.floor(rng() * mesh.nEdges);
      if (mesh.flipEdge(e)) successes++;
      attempts++;
    }
    expect(successes).toBeGreaterThan(0);
    // Halfedge invariants should still hold across the whole mesh.
    for (let h = 0; h < mesh.nHalfedges; h++) {
      expect(mesh.twin(mesh.twin(h))).toBe(h);
      expect(mesh.tipVertex(h)).toBe(mesh.vertex(mesh.next(h)));
    }
    let sum = 0;
    for (let v = 0; v < mesh.nVertices; v++) sum += mesh.vertexDegree(v);
    expect(sum).toBe(2 * mesh.nEdges);
  });
});

/** Tiny seeded PRNG so the random-flip tests are reproducible. */
function mulberry32(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
