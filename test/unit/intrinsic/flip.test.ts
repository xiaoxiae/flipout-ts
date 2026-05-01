/**
 * Edge-flip invariants for SignpostIntrinsicTriangulation.
 *
 * The most error-prone area: every flip must (1) preserve the underlying
 * mesh's combinatorial roundtrip exactly, (2) restore the per-edge intrinsic
 * lengths to FP precision after a double flip, (3) preserve total angle
 * defect, and (4) keep every face's three lengths satisfying the strict
 * triangle inequality.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import {
  SignpostIntrinsicTriangulation,
  layoutTriangleVertex,
} from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { cube, flatGrid, flatQuad, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

interface Built {
  sit: SignpostIntrinsicTriangulation;
}

interface MeshDataLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function build(meshData: MeshDataLike): Built {
  const mesh = SurfaceMesh.fromFaces(meshData.faces, meshData.vertices.length);
  const geom = new VertexPositionGeometry(mesh, meshData.vertices);
  const sit = new SignpostIntrinsicTriangulation(geom);
  return { sit };
}

interface TopologySnapshot {
  faceSets: string[];   // unordered face vertex triples (sorted)
  edgeSets: string[];   // unordered edge endpoint pairs (sorted)
  vertexHe: number[];   // per-vertex tail-vertex check via halfedge array
  vertexDegrees: number[];
}

/**
 * Snapshot the *topology* (face vertex triples + edge endpoint pairs +
 * per-vertex degree). L1's `flipEdge` is not byte-level idempotent because
 * it permutes face-id assignments to half-edges (`heFaceArr`) and may
 * reassign `vertexHalfedge` representatives. The topology, however, IS
 * preserved — that's what we check for the "flip(flip(e)) restores" test.
 */
function topologySnapshot(sit: SignpostIntrinsicTriangulation): TopologySnapshot {
  const m = sit.intrinsicMesh;
  const faceSets: string[] = [];
  for (let f = 0; f < m.nFaces; f++) {
    const tri = [...m.verticesOfFace(f)].sort((a, b) => a - b);
    faceSets.push(tri.join(','));
  }
  faceSets.sort();
  const edgeSets: string[] = [];
  for (let e = 0; e < m.nEdges; e++) {
    const h = m.edgeHalfedge(e);
    const a = m.vertex(h);
    const b = m.tipVertex(h);
    edgeSets.push(a < b ? `${a},${b}` : `${b},${a}`);
  }
  edgeSets.sort();
  const vertexHe: number[] = [];
  for (let v = 0; v < m.nVertices; v++) vertexHe.push(m.vertex(m.vertexHalfedge(v)));
  const vertexDegrees: number[] = [];
  for (let v = 0; v < m.nVertices; v++) vertexDegrees.push(m.vertexDegree(v));
  return { faceSets, edgeSets, vertexHe, vertexDegrees };
}

function lengthsSnapshot(sit: SignpostIntrinsicTriangulation): number[] {
  return Array.from(sit.edgeLengths);
}

function totalAngleDefect(sit: SignpostIntrinsicTriangulation): number {
  // Recompute angle sums at each vertex from intrinsic lengths via
  // cornerAngleAt, then sum 2π - sum (interior) and π - sum (boundary).
  const m = sit.intrinsicMesh;
  let total = 0;
  for (let v = 0; v < m.nVertices; v++) {
    let s = 0;
    for (const h of m.outgoingHalfedges(v)) {
      if (m.face(h) === -1) continue;
      s += sit.cornerAngleAt(h);
    }
    const target = m.isBoundaryVertex(v) ? Math.PI : 2 * Math.PI;
    total += target - s;
  }
  return total;
}

describe('flipEdge — boundary edges are rejected', () => {
  it('flatQuad: a boundary edge cannot be flipped', () => {
    const { sit } = build(flatQuad());
    const m = sit.intrinsicMesh;
    let foundBoundary = false;
    for (let e = 0; e < m.nEdges; e++) {
      if (m.isBoundaryEdge(e)) {
        foundBoundary = true;
        const before = topologySnapshot(sit);
        expect(sit.flipEdge(e)).toBe(false);
        expect(topologySnapshot(sit)).toEqual(before);
      }
    }
    expect(foundBoundary).toBe(true);
  });

  it('flatGrid 4x4: boundary edges all rejected', () => {
    const { sit } = build(flatGrid(4));
    const m = sit.intrinsicMesh;
    for (let e = 0; e < m.nEdges; e++) {
      if (m.isBoundaryEdge(e)) {
        expect(sit.flipEdge(e)).toBe(false);
      }
    }
  });
});

describe('flipEdge — flat quad has the analytic flip length', () => {
  it('flatQuad: the diagonal flip swaps a unit-length axes diagonal for the other (both √2)', () => {
    const { sit } = build(flatQuad());
    const m = sit.intrinsicMesh;
    // The shared interior edge runs between vertices 0 and 2 (the (0,0)-(1,1) diagonal).
    // Find it.
    let sharedEdge = -1;
    for (let e = 0; e < m.nEdges; e++) {
      if (!m.isBoundaryEdge(e)) {
        sharedEdge = e;
        break;
      }
    }
    expect(sharedEdge).toBeGreaterThanOrEqual(0);
    const before = sit.edgeLengths[sharedEdge]!;
    expect(before).toBeCloseTo(Math.sqrt(2), 10);
    const ok = sit.flipEdge(sharedEdge);
    expect(ok).toBe(true);
    expect(sit.edgeLengths[sharedEdge]!).toBeCloseTo(Math.sqrt(2), 10);
  });
});

describe('flipEdge — combinatorial round-trip restores topology and intrinsic lengths', () => {
  it.each<[string, MeshDataLike]>([
    ['tetrahedron', tetrahedron()],
    ['icosahedron', icosahedron()],
    ['cube', cube()],
  ])('%s: flip(flip(e)) restores topology, lengths, and per-vertex signpost multiset', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) continue;
      const before = topologySnapshot(sit);
      const lensBefore = lengthsSnapshot(sit);
      // For each vertex, snapshot the signpost multiset (sorted).
      const sigsByVertexBefore: number[][] = [];
      for (let v = 0; v < mesh.nVertices; v++) {
        const arr: number[] = [];
        for (const h of mesh.outgoingHalfedges(v)) arr.push(sit.halfedgeSignposts[h]!);
        arr.sort((a, b) => a - b);
        sigsByVertexBefore.push(arr);
      }

      const first = sit.flipEdge(e);
      if (!first) continue;
      const second = sit.flipEdge(e);
      expect(second).toBe(true);

      // Topology unchanged
      expect(topologySnapshot(sit)).toEqual(before);
      // Edge lengths restored
      const lensAfter = lengthsSnapshot(sit);
      for (let i = 0; i < lensAfter.length; i++) {
        expect(lensAfter[i]!).toBeCloseTo(lensBefore[i]!, 10);
      }
      // Per-vertex signpost CYCLIC structure restored. Halfedge IDs may
      // swap tails (the L1 flip swaps the two halfedges of an edge),
      // AND `vertexHalfedge(v)` may shift to a different anchor, which
      // rotates all signposts at v by a constant. We compare the sorted
      // wedge-gap multiset (consecutive sorted-signpost differences plus
      // the wraparound gap), which is rotation-invariant.
      for (let v = 0; v < mesh.nVertices; v++) {
        const sum = sit.vertexAngleSums[v]!;
        const arrAfter: number[] = [];
        for (const h of mesh.outgoingHalfedges(v)) arrAfter.push(sit.halfedgeSignposts[h]!);
        arrAfter.sort((a, b) => a - b);
        const arrBefore = sigsByVertexBefore[v]!;
        expect(arrAfter.length).toBe(arrBefore.length);
        const gapsAfter: number[] = [];
        const gapsBefore: number[] = [];
        for (let i = 0; i < arrAfter.length; i++) {
          const j = (i + 1) % arrAfter.length;
          const gA = i + 1 === arrAfter.length ? sum - arrAfter[i]! + arrAfter[j]! : arrAfter[j]! - arrAfter[i]!;
          gapsAfter.push(gA);
          const gB = i + 1 === arrBefore.length ? sum - arrBefore[i]! + arrBefore[j]! : arrBefore[j]! - arrBefore[i]!;
          gapsBefore.push(gB);
        }
        gapsAfter.sort((a, b) => a - b);
        gapsBefore.sort((a, b) => a - b);
        for (let i = 0; i < gapsAfter.length; i++) {
          expect(gapsAfter[i]!).toBeCloseTo(gapsBefore[i]!, 10);
        }
      }
    }
  });
});

describe('flipEdge — preserves total angle defect', () => {
  it.each<[string, MeshDataLike, number]>([
    ['tetrahedron', tetrahedron(), 4 * Math.PI],
    ['icosahedron', icosahedron(), 4 * Math.PI],
    ['cube', cube(), 4 * Math.PI],
  ])('%s: total angle defect is 4π and preserved by every flip', (_, m, expected) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    expect(totalAngleDefect(sit)).toBeCloseTo(expected, 8);
    for (let e = 0; e < mesh.nEdges; e++) {
      if (!mesh.isBoundaryEdge(e)) {
        if (sit.flipEdge(e)) {
          expect(totalAngleDefect(sit)).toBeCloseTo(expected, 8);
          // restore
          sit.flipEdge(e);
        }
      }
    }
  });
});

describe('flipEdge — every face still satisfies the strict triangle inequality', () => {
  it.each<[string, MeshDataLike]>([
    ['icosahedron', icosahedron()],
    ['cube', cube()],
  ])('%s: after each flip, all 3 lengths a < b + c', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) continue;
      const ok = sit.flipEdge(e);
      if (!ok) continue;
      for (let f = 0; f < mesh.nFaces; f++) {
        const it = mesh.halfedgesAroundFace(f);
        const h0 = it.next().value as number;
        const h1 = it.next().value as number;
        const h2 = it.next().value as number;
        const a = sit.edgeLengths[mesh.edge(h0)]!;
        const b = sit.edgeLengths[mesh.edge(h1)]!;
        const c = sit.edgeLengths[mesh.edge(h2)]!;
        const eps = 1e-9;
        expect(a).toBeLessThan(b + c + eps);
        expect(b).toBeLessThan(a + c + eps);
        expect(c).toBeLessThan(a + b + eps);
      }
      sit.flipEdge(e); // restore
    }
  });
});

describe('flipEdge — analytic length on a flat quad (paper math)', () => {
  it('flat quad with vertices on (0,0),(1,0),(1,1),(0,1) — flip along (0,0)-(1,1) gives sqrt(2)', () => {
    const { sit } = build(flatQuad());
    const mesh = sit.intrinsicMesh;
    let sharedEdge = -1;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (!mesh.isBoundaryEdge(e)) sharedEdge = e;
    }
    expect(sharedEdge).toBeGreaterThanOrEqual(0);
    const beforeLen = sit.edgeLengths[sharedEdge]!;
    expect(beforeLen).toBeCloseTo(Math.sqrt(2), 12);
    sit.flipEdge(sharedEdge);
    expect(sit.edgeLengths[sharedEdge]!).toBeCloseTo(Math.sqrt(2), 10);
  });

  it('layoutTriangleVertex sanity — equilateral triangle', () => {
    const a: [number, number] = [0, 0];
    const b: [number, number] = [1, 0];
    const c = layoutTriangleVertex(a, b, 1, 1);
    expect(c[0]).toBeCloseTo(0.5, 12);
    expect(c[1]).toBeCloseTo(Math.sqrt(3) / 2, 12);
  });

  it('layoutTriangleVertex sanity — 3-4-5 right triangle', () => {
    const a: [number, number] = [0, 0];
    const b: [number, number] = [3, 0];
    const c = layoutTriangleVertex(a, b, 4, 5);
    // c lies above AB; project: distance from A = 5, from B = 4, AB = 3.
    expect(Math.hypot(c[0], c[1])).toBeCloseTo(5, 10);
    expect(Math.hypot(c[0] - 3, c[1])).toBeCloseTo(4, 10);
    expect(c[1]).toBeGreaterThan(0);
  });
});

describe('flipEdge — out-of-range edge throws', () => {
  it('flipping edge -1 throws', () => {
    const { sit } = build(tetrahedron());
    expect(() => sit.flipEdge(-1)).toThrow(RangeError);
  });

  it('flipping edge >= nEdges throws', () => {
    const { sit } = build(tetrahedron());
    expect(() => sit.flipEdge(sit.intrinsicMesh.nEdges)).toThrow(RangeError);
  });
});

describe('flipEdge — corner angle sum at each vertex still equals vertexAngleSums', () => {
  it('icosahedron: after each flip, sum of corner angles at each vertex (computed from intrinsic lengths) equals the constant `vertexAngleSums[v]`', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) continue;
      const ok = sit.flipEdge(e);
      if (!ok) continue;
      for (let v = 0; v < mesh.nVertices; v++) {
        let s = 0;
        for (const h of mesh.outgoingHalfedges(v)) {
          if (mesh.face(h) === -1) continue;
          s += sit.cornerAngleAt(h);
        }
        // Intrinsic flips preserve corner angle sums (the intrinsic
        // triangulation is just a different way of triangulating the same
        // intrinsic surface; the sum of angles around v is its angle sum
        // in the intrinsic metric, which is invariant under retriangulation).
        expect(s).toBeCloseTo(sit.vertexAngleSums[v]!, 8);
      }
      sit.flipEdge(e);
    }
  });
});
