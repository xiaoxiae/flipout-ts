/**
 * Vertex-insertion tests for SignpostIntrinsicTriangulation.
 *
 * Mirrors gc tests around `insertVertex_face` and `insertVertex_edge`.
 * We verify both combinatorial counts (delegated to L1) and geometric
 * invariants (edge lengths, angle sums, signposts).
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { cube, flatGrid, flatQuad, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

interface MeshDataLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}

interface Built {
  geom: VertexPositionGeometry;
  sit: SignpostIntrinsicTriangulation;
}

function build(m: MeshDataLike): Built {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  const sit = new SignpostIntrinsicTriangulation(geom);
  return { geom, sit };
}

const TWO_PI = 2 * Math.PI;

function totalAngleDefect(sit: SignpostIntrinsicTriangulation): number {
  // Sum over interior vertices of (2π - angle sum); interior + boundary
  // sum gives the full Gauss-Bonnet quantity. We just sum 2π - angleSum
  // over all vertices including new ones — for a closed mesh this equals
  // 2π χ.
  let sum = 0;
  for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
    const target = sit.intrinsicMesh.isBoundaryVertex(v) ? Math.PI : TWO_PI;
    sum += target - sit.vertexAngleSums[v]!;
  }
  return sum;
}

function checkTriangleInequality(sit: SignpostIntrinsicTriangulation): void {
  const im = sit.intrinsicMesh;
  for (let f = 0; f < im.nFaces; f++) {
    const it = im.halfedgesAroundFace(f);
    const h0 = it.next().value as number;
    const h1 = it.next().value as number;
    const h2 = it.next().value as number;
    const l0 = sit.edgeLengths[im.edge(h0)]!;
    const l1 = sit.edgeLengths[im.edge(h1)]!;
    const l2 = sit.edgeLengths[im.edge(h2)]!;
    expect(l0 + l1).toBeGreaterThanOrEqual(l2 - 1e-9);
    expect(l1 + l2).toBeGreaterThanOrEqual(l0 - 1e-9);
    expect(l0 + l2).toBeGreaterThanOrEqual(l1 - 1e-9);
  }
}

function findInteriorEdge(sit: SignpostIntrinsicTriangulation): number {
  for (let e = 0; e < sit.intrinsicMesh.nEdges; e++) {
    if (!sit.intrinsicMesh.isBoundaryEdge(e)) return e;
  }
  return -1;
}

describe('insertVertex_face — basic counts', () => {
  it.each<[string, MeshDataLike]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['flatQuad', flatQuad()],
    ['flatGrid 4x4', flatGrid(4, 1)],
  ])('%s: insert into face 0 grows arrays correctly', (_, m) => {
    const { sit } = build(m);
    const nV0 = sit.intrinsicMesh.nVertices;
    const nE0 = sit.intrinsicMesh.nEdges;
    const newV = sit.insertVertex_face(0, [1 / 3, 1 / 3, 1 / 3]);
    expect(newV).toBe(nV0); // new vertex is appended at end
    expect(sit.intrinsicMesh.nVertices).toBe(nV0 + 1);
    expect(sit.intrinsicMesh.nEdges).toBe(nE0 + 3);
    expect(sit.vertexAngleSums.length).toBeGreaterThanOrEqual(nV0 + 1);
    expect(sit.edgeLengths.length).toBeGreaterThanOrEqual(nE0 + 3);
  });
});

describe('insertVertex_face — angle sum invariants', () => {
  it.each<[string, MeshDataLike]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['flatQuad', flatQuad()],
  ])('%s: inserted vertex has angle sum 2π', (_, m) => {
    const { sit } = build(m);
    const newV = sit.insertVertex_face(0, [0.4, 0.3, 0.3]);
    expect(sit.vertexAngleSums[newV]).toBeCloseTo(TWO_PI, 12);
  });

  it('cube: total angle defect preserved after face insertion', () => {
    const { sit } = build(cube());
    const before = totalAngleDefect(sit);
    sit.insertVertex_face(0, [0.4, 0.3, 0.3]);
    const after = totalAngleDefect(sit);
    expect(after).toBeCloseTo(before, 12);
  });

  it('icosahedron: total angle defect preserved after face insertion', () => {
    const { sit } = build(icosahedron());
    const before = totalAngleDefect(sit);
    sit.insertVertex_face(0, [0.4, 0.3, 0.3]);
    const after = totalAngleDefect(sit);
    expect(after).toBeCloseTo(before, 12);
  });
});

describe('insertVertex_face — edge length invariants', () => {
  it.each<[string, MeshDataLike]>([
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['flatQuad', flatQuad()],
  ])('%s: triangle inequality holds on every face after insertion', (_, m) => {
    const { sit } = build(m);
    sit.insertVertex_face(0, [0.4, 0.4, 0.2]);
    checkTriangleInequality(sit);
  });

  it('flatQuad face 0: centroid insertion produces three equal-length edges to centroid', () => {
    // flatQuad face 0 = (0,1,2) at (0,0,0), (1,0,0), (1,1,0). Centroid =
    // (2/3, 1/3, 0). Distances from centroid to the three corners:
    //   to (0,0,0): sqrt(4/9 + 1/9) = sqrt(5/9) ≈ 0.7454
    //   to (1,0,0): sqrt(1/9 + 1/9) = sqrt(2/9) ≈ 0.4714
    //   to (1,1,0): sqrt(1/9 + 4/9) = sqrt(5/9) ≈ 0.7454
    const { sit } = build(flatQuad());
    const im = sit.intrinsicMesh;
    const newV = sit.insertVertex_face(0, [1 / 3, 1 / 3, 1 / 3]);
    const lens: number[] = [];
    for (const he of im.outgoingHalfedges(newV)) {
      lens.push(sit.edgeLengths[im.edge(he)]!);
    }
    lens.sort((a, b) => a - b);
    expect(lens.length).toBe(3);
    expect(lens[0]).toBeCloseTo(Math.sqrt(2 / 9), 9);
    expect(lens[1]).toBeCloseTo(Math.sqrt(5 / 9), 9);
    expect(lens[2]).toBeCloseTo(Math.sqrt(5 / 9), 9);
  });
});

describe('insertVertex_face — signpost invariants', () => {
  it.each<[string, MeshDataLike]>([
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s: signposts at new vertex span [0, 2π) and increase CCW', (_, m) => {
    const { sit } = build(m);
    const newV = sit.insertVertex_face(0, [0.5, 0.3, 0.2]);
    const im = sit.intrinsicMesh;
    let prev = -Infinity;
    let max = 0;
    for (const he of im.outgoingHalfedges(newV)) {
      const sp = sit.halfedgeSignposts[he]!;
      expect(sp).toBeGreaterThanOrEqual(0);
      expect(sp).toBeLessThan(TWO_PI + 1e-12);
      expect(sp).toBeGreaterThanOrEqual(prev - 1e-12);
      prev = sp;
      max = Math.max(max, sp);
    }
    // Cumulative sum of corner angles around newV should be 2π.
    let totalAngle = 0;
    for (const he of im.outgoingHalfedges(newV)) {
      totalAngle += sit.cornerAngleAt(he);
    }
    expect(totalAngle).toBeCloseTo(TWO_PI, 9);
  });
});

describe('insertVertex_face — snap-to-corner', () => {
  it('flatQuad: bary (1, 0, 0) returns the existing corner vertex', () => {
    const { sit } = build(flatQuad());
    const before = sit.intrinsicMesh.nVertices;
    const v = sit.insertVertex_face(0, [1, 0, 0]);
    expect(v).toBe(0);
    expect(sit.intrinsicMesh.nVertices).toBe(before); // no insertion
  });

  it('cube: bary (0.999999999999, 0, 0) snaps to corner', () => {
    const { sit } = build(cube());
    const before = sit.intrinsicMesh.nVertices;
    const eps = SignpostIntrinsicTriangulation.SNAP_EPS;
    const v = sit.insertVertex_face(0, [1 - eps / 10, eps / 20, eps / 20]);
    expect(v).toBeLessThan(before);
    expect(sit.intrinsicMesh.nVertices).toBe(before);
  });
});

describe('insertVertex_face — invalid input', () => {
  it('rejects negative barycentric', () => {
    const { sit } = build(flatQuad());
    expect(() => sit.insertVertex_face(0, [-0.1, 0.6, 0.5])).toThrow(RangeError);
  });

  it('rejects barycentric not summing to 1', () => {
    const { sit } = build(flatQuad());
    expect(() => sit.insertVertex_face(0, [0.5, 0.5, 0.5])).toThrow(RangeError);
  });

  it('rejects invalid face index', () => {
    const { sit } = build(flatQuad());
    expect(() => sit.insertVertex_face(99, [1 / 3, 1 / 3, 1 / 3])).toThrow(RangeError);
  });
});

describe('insertVertex_edge — basic counts', () => {
  it.each<[string, MeshDataLike]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['flatGrid 3x3', flatGrid(3, 1)],
  ])('%s: interior edge insertion grows arrays correctly', (_, m) => {
    const { sit } = build(m);
    const e = findInteriorEdge(sit);
    expect(e).toBeGreaterThanOrEqual(0);
    const nV0 = sit.intrinsicMesh.nVertices;
    const nE0 = sit.intrinsicMesh.nEdges;
    const newV = sit.insertVertex_edge(e, 0.5);
    expect(newV).toBe(nV0);
    expect(sit.intrinsicMesh.nVertices).toBe(nV0 + 1);
    expect(sit.intrinsicMesh.nEdges).toBe(nE0 + 3);
  });

  it('flatQuad: boundary edge insertion grows arrays correctly', () => {
    const { sit } = build(flatQuad());
    let eBoundary = -1;
    for (let e = 0; e < sit.intrinsicMesh.nEdges; e++) {
      if (sit.intrinsicMesh.isBoundaryEdge(e)) {
        eBoundary = e;
        break;
      }
    }
    expect(eBoundary).toBeGreaterThanOrEqual(0);
    const nV0 = sit.intrinsicMesh.nVertices;
    const nE0 = sit.intrinsicMesh.nEdges;
    const newV = sit.insertVertex_edge(eBoundary, 0.4);
    expect(newV).toBe(nV0);
    expect(sit.intrinsicMesh.nVertices).toBe(nV0 + 1);
    expect(sit.intrinsicMesh.nEdges).toBe(nE0 + 2);
  });
});

describe('insertVertex_edge — angle sum', () => {
  it.each<[string, MeshDataLike]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s: interior edge insertion → new vertex angle sum = 2π', (_, m) => {
    const { sit } = build(m);
    const e = findInteriorEdge(sit);
    const newV = sit.insertVertex_edge(e, 0.5);
    expect(sit.vertexAngleSums[newV]).toBeCloseTo(TWO_PI, 12);
  });

  it('flatQuad: boundary edge insertion → new vertex angle sum = π', () => {
    const { sit } = build(flatQuad());
    let eBoundary = -1;
    for (let e = 0; e < sit.intrinsicMesh.nEdges; e++) {
      if (sit.intrinsicMesh.isBoundaryEdge(e)) {
        eBoundary = e;
        break;
      }
    }
    const newV = sit.insertVertex_edge(eBoundary, 0.5);
    expect(sit.vertexAngleSums[newV]).toBeCloseTo(Math.PI, 12);
  });
});

describe('insertVertex_edge — edge length geometry', () => {
  it('flatQuad interior diagonal at t=0.5: new edges have lengths sqrt(2)/2 each, plus midline', () => {
    // flatQuad diagonal: 0 (0,0,0) — 2 (1,1,0). Length = sqrt(2). At t=0.5,
    // newV is at (0.5, 0.5, 0). Halves: sqrt(2)/2 each.
    // The two cross edges go to v=1 (1,0,0) and v=3 (0,1,0). Both at
    // distance sqrt(0.5) = sqrt(2)/2.
    const { sit } = build(flatQuad());
    const im = sit.intrinsicMesh;
    let eDiag = -1;
    for (let e = 0; e < im.nEdges; e++) {
      const h = im.edgeHalfedge(e);
      const va = im.vertex(h);
      const vb = im.tipVertex(h);
      if ((va === 0 && vb === 2) || (va === 2 && vb === 0)) {
        eDiag = e;
        break;
      }
    }
    expect(eDiag).toBeGreaterThanOrEqual(0);
    const newV = sit.insertVertex_edge(eDiag, 0.5);

    const lens: number[] = [];
    for (const he of im.outgoingHalfedges(newV)) {
      lens.push(sit.edgeLengths[im.edge(he)]!);
    }
    expect(lens.length).toBe(4);
    for (const l of lens) {
      expect(l).toBeCloseTo(Math.SQRT2 / 2, 12);
    }
  });

  it('cube interior edge at t=0.5: two halves are equal', () => {
    const { sit } = build(cube());
    const e = findInteriorEdge(sit);
    const lE = sit.edgeLengths[e]!;
    const im = sit.intrinsicMesh;
    const oldHe = im.edgeHalfedge(e);
    const va = im.vertex(oldHe);
    const vb = im.tipVertex(oldHe);
    const newV = sit.insertVertex_edge(e, 0.5);

    // After insertion, the two "halves" are the edges from newV to va and
    // newV to vb.
    let toA = -1;
    let toB = -1;
    for (const he of im.outgoingHalfedges(newV)) {
      const tip = im.tipVertex(he);
      if (tip === va) toA = sit.edgeLengths[im.edge(he)]!;
      if (tip === vb) toB = sit.edgeLengths[im.edge(he)]!;
    }
    expect(toA).toBeCloseTo(lE / 2, 12);
    expect(toB).toBeCloseTo(lE / 2, 12);
  });

  it.each<[string, MeshDataLike]>([
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s: triangle inequality holds on every face after edge insertion', (_, m) => {
    const { sit } = build(m);
    const e = findInteriorEdge(sit);
    sit.insertVertex_edge(e, 0.4);
    checkTriangleInequality(sit);
  });
});

describe('insertVertex_edge — signposts span 2π', () => {
  it('cube interior edge at t=0.5: signposts at new vertex sum corner angles to 2π', () => {
    const { sit } = build(cube());
    const e = findInteriorEdge(sit);
    const newV = sit.insertVertex_edge(e, 0.5);
    const im = sit.intrinsicMesh;
    let totalAngle = 0;
    for (const he of im.outgoingHalfedges(newV)) {
      totalAngle += sit.cornerAngleAt(he);
    }
    expect(totalAngle).toBeCloseTo(TWO_PI, 9);
  });
});

describe('insertVertex_edge — snap-to-endpoint', () => {
  it('flatQuad: t=0 returns first endpoint, no insertion', () => {
    const { sit } = build(flatQuad());
    const e = findInteriorEdge(sit);
    if (e < 0) return;
    const before = sit.intrinsicMesh.nVertices;
    const v = sit.insertVertex_edge(e, 0);
    expect(v).toBeLessThan(before);
    expect(sit.intrinsicMesh.nVertices).toBe(before);
  });

  it('cube: t=1e-12 snaps to endpoint', () => {
    const { sit } = build(cube());
    const e = findInteriorEdge(sit);
    const before = sit.intrinsicMesh.nVertices;
    const v = sit.insertVertex_edge(e, 1e-12);
    expect(v).toBeLessThan(before);
    expect(sit.intrinsicMesh.nVertices).toBe(before);
  });
});

describe('insertVertex_edge — invalid input', () => {
  it('rejects t < 0 and t > 1', () => {
    const { sit } = build(flatQuad());
    const e = findInteriorEdge(sit);
    if (e < 0) return;
    expect(() => sit.insertVertex_edge(e, -0.1)).toThrow(RangeError);
    expect(() => sit.insertVertex_edge(e, 1.1)).toThrow(RangeError);
  });

  it('rejects invalid edge index', () => {
    const { sit } = build(flatQuad());
    expect(() => sit.insertVertex_edge(999, 0.5)).toThrow(RangeError);
  });
});

describe('insertVertex_edge — surface-point bookkeeping', () => {
  it('records inserted vertex location for face insertions', () => {
    const { sit } = build(cube());
    const newV = sit.insertVertex_face(0, [0.4, 0.3, 0.3]);
    const loc = sit.insertedVertexLocations.get(newV);
    expect(loc).toBeDefined();
    expect(loc!.kind).toBe('face');
  });

  it('records inserted vertex location for edge insertions', () => {
    const { sit } = build(cube());
    const e = findInteriorEdge(sit);
    const newV = sit.insertVertex_edge(e, 0.4);
    const loc = sit.insertedVertexLocations.get(newV);
    expect(loc).toBeDefined();
    expect(loc!.kind).toBe('edge');
  });
});
