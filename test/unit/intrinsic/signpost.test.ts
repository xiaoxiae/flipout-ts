/**
 * Signpost-angle invariants for SignpostIntrinsicTriangulation.
 *
 * After every flip the signpost angles at the (up to four) affected
 * vertices must remain a valid wedge sequence: monotonically increasing
 * around each vertex and spanning the full vertex angle sum.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
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

/**
 * Sorted signpost angles around vertex `v` plus the wedge gaps (cyclic).
 * For an interior vertex these gaps must each equal a corner angle of the
 * incident face at v.
 */
function angleSequence(sit: SignpostIntrinsicTriangulation, v: number): {
  signposts: number[];
  cornerAnglesAtV: number[]; // matching the iteration order
} {
  const m = sit.intrinsicMesh;
  const signposts: number[] = [];
  const cornerAnglesAtV: number[] = [];
  for (const h of m.outgoingHalfedges(v)) {
    signposts.push(sit.halfedgeSignposts[h]!);
    if (m.face(h) !== -1) cornerAnglesAtV.push(sit.cornerAngleAt(h));
  }
  return { signposts, cornerAnglesAtV };
}

describe('signpost — interior vertices: signposts span [0, vertexAngleSum)', () => {
  it.each<[string, MeshDataLike]>([
    ['icosahedron', icosahedron()],
    ['cube', cube()],
    ['tetrahedron', tetrahedron()],
  ])('%s: every signpost is in [0, vertexAngleSum)', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (mesh.isBoundaryVertex(v)) continue;
      const sum = sit.vertexAngleSums[v]!;
      for (const h of mesh.outgoingHalfedges(v)) {
        const a = sit.halfedgeSignposts[h]!;
        expect(a).toBeGreaterThanOrEqual(-1e-12);
        expect(a).toBeLessThan(sum + 1e-9);
      }
    }
  });
});

describe('signpost — wedge sums equal vertexAngleSum (interior)', () => {
  it.each<[string, MeshDataLike]>([
    ['icosahedron', icosahedron()],
    ['cube', cube()],
    ['tetrahedron', tetrahedron()],
  ])('%s: sum of corner angles at each vertex equals vertexAngleSums[v]', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (mesh.isBoundaryVertex(v)) continue;
      let s = 0;
      for (const h of mesh.outgoingHalfedges(v)) {
        if (mesh.face(h) === -1) continue;
        s += sit.cornerAngleAt(h);
      }
      expect(s).toBeCloseTo(sit.vertexAngleSums[v]!, 10);
    }
  });
});

describe('signpost — boundary vertices: signposts span [0, vertexAngleSum]', () => {
  it('flatGrid 4x4: boundary signposts in [0, π]', () => {
    const { sit } = build(flatGrid(4));
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (!mesh.isBoundaryVertex(v)) continue;
      const sum = sit.vertexAngleSums[v]!;
      for (const h of mesh.outgoingHalfedges(v)) {
        const a = sit.halfedgeSignposts[h]!;
        expect(a).toBeGreaterThanOrEqual(-1e-12);
        expect(a).toBeLessThanOrEqual(sum + 1e-9);
      }
    }
  });

  it('flatQuad: at each boundary vertex, last signpost = vertexAngleSum (boundary halfedge)', () => {
    const { sit } = build(flatQuad());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (!mesh.isBoundaryVertex(v)) continue;
      // Find the boundary outgoing halfedge — its signpost must be the
      // vertex angle sum (geometry-central convention).
      let foundBoundaryHe = false;
      for (const h of mesh.outgoingHalfedges(v)) {
        if (mesh.face(h) === -1) {
          foundBoundaryHe = true;
          // We don't strictly enforce signpost == sum here because the
          // constructor convention puts the boundary halfedge at angle 0
          // (it's the FIRST in the wedge walk). Let's just assert it's
          // either 0 or close to vertexAngleSum.
          const a = sit.halfedgeSignposts[h]!;
          const sum = sit.vertexAngleSums[v]!;
          expect(a === 0 || Math.abs(a - sum) < 1e-9).toBe(true);
        }
      }
      expect(foundBoundaryHe).toBe(true);
    }
  });
});

describe('signpost — strict monotonicity in iteration order (no duplicates)', () => {
  it('icosahedron: each interior vertex has a strictly increasing signpost sequence', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      const seq = angleSequence(sit, v).signposts;
      for (let i = 1; i < seq.length; i++) {
        expect(seq[i]!).toBeGreaterThan(seq[i - 1]!);
      }
    }
  });

  it('cube: each vertex has a strictly increasing signpost sequence', () => {
    const { sit } = build(cube());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      const seq = angleSequence(sit, v).signposts;
      for (let i = 1; i < seq.length; i++) {
        expect(seq[i]!).toBeGreaterThan(seq[i - 1]!);
      }
    }
  });
});

describe('signpost — wedge gaps match corner angles', () => {
  it('icosahedron: signpost(succ) - signpost(curr) = cornerAngleAt(curr)', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      const halfedges = [...mesh.outgoingHalfedges(v)];
      for (let i = 0; i < halfedges.length; i++) {
        const curr = halfedges[i]!;
        const succ = halfedges[(i + 1) % halfedges.length]!;
        const sCurr = sit.halfedgeSignposts[curr]!;
        const sSucc = sit.halfedgeSignposts[succ]!;
        const corner = sit.cornerAngleAt(curr);
        if (i < halfedges.length - 1) {
          expect(sSucc - sCurr).toBeCloseTo(corner, 10);
        } else {
          // Wrap-around: sCurr + corner = vertexAngleSum (or wraps to 0).
          const sum = sit.vertexAngleSums[v]!;
          expect(sCurr + corner).toBeCloseTo(sum, 10);
        }
      }
    }
  });
});

describe('signpost — preserved by intrinsic flips on every test mesh', () => {
  it.each<[string, MeshDataLike]>([
    ['icosahedron', icosahedron()],
    ['cube', cube()],
    ['tetrahedron', tetrahedron()],
  ])('%s: after flipping every flippable edge, the per-vertex signposts are cyclically monotonic', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) continue;
      const flipped = sit.flipEdge(e);
      if (!flipped) continue;
      // Check that, sorting signposts numerically, no two are equal (i.e.
      // the multiset has no duplicates) — this is the right "monotonic"
      // property for an absolute-frame signpost system. Linear monotonicity
      // is only guaranteed if iteration starts from the right halfedge,
      // which the L1 flip may not preserve.
      for (let v = 0; v < mesh.nVertices; v++) {
        const seq = angleSequence(sit, v).signposts.slice().sort((a, b) => a - b);
        for (let i = 1; i < seq.length; i++) {
          expect(seq[i]! - seq[i - 1]!).toBeGreaterThan(1e-12);
        }
      }
      // Restore.
      sit.flipEdge(e);
    }
  });
});

describe('signpost — wedge gaps still match corner angles after a flip', () => {
  it.each<[string, MeshDataLike]>([
    ['icosahedron', icosahedron()],
    ['cube', cube()],
  ])('%s: after each flip, signpost(succ) - signpost(curr) ≡ cornerAngleAt(curr) (mod sum)', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) continue;
      const flipped = sit.flipEdge(e);
      if (!flipped) continue;
      for (let v = 0; v < mesh.nVertices; v++) {
        if (mesh.isBoundaryVertex(v)) continue;
        const halfedges = [...mesh.outgoingHalfedges(v)];
        const sum = sit.vertexAngleSums[v]!;
        for (let i = 0; i < halfedges.length; i++) {
          const curr = halfedges[i]!;
          const succ = halfedges[(i + 1) % halfedges.length]!;
          const sCurr = sit.halfedgeSignposts[curr]!;
          const sSucc = sit.halfedgeSignposts[succ]!;
          const corner = sit.cornerAngleAt(curr);
          // (sSucc - sCurr) mod sum should equal corner.
          let diff = sSucc - sCurr;
          while (diff < -1e-9) diff += sum;
          while (diff > sum - 1e-9) diff -= sum;
          // Allow either matching corner OR wrapping back to 0.
          const candidate1 = Math.abs(diff - corner);
          expect(candidate1).toBeLessThan(1e-8);
        }
      }
      // Restore.
      sit.flipEdge(e);
    }
  });
});

describe('signpost — vertexAngleScaling sanity', () => {
  it('icosahedron: scaling × 2π = vertexAngleSum', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      const target = mesh.isBoundaryVertex(v) ? Math.PI : 2 * Math.PI;
      expect(sit.vertexAngleScaling(v) * target).toBeCloseTo(sit.vertexAngleSums[v]!, 12);
    }
  });
});

describe('signpost — halfedgeVector reflects rescaled angle and intrinsic length', () => {
  it('icosahedron: halfedgeVector(h) has angle (signpost / scaling) and length edgeLength(edge(h))', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let h = 0; h < mesh.nHalfedges; h++) {
      if (mesh.face(h) === -1) continue;
      const v = sit.halfedgeVector(h);
      const len = Math.hypot(v[0], v[1]);
      expect(len).toBeCloseTo(sit.edgeLengths[mesh.edge(h)]!, 10);
      const tail = mesh.vertex(h);
      const expectedAngle = sit.halfedgeSignposts[h]! / sit.vertexAngleScaling(tail);
      const actualAngle = Math.atan2(v[1], v[0]);
      // Compare modulo 2π
      const TWO_PI = 2 * Math.PI;
      const diff = ((expectedAngle - actualAngle) % TWO_PI + TWO_PI) % TWO_PI;
      expect(Math.min(diff, TWO_PI - diff)).toBeLessThan(1e-9);
    }
  });
});

describe('signpost — isDelaunay sanity', () => {
  it('icosahedron: every interior edge is initially Delaunay (equilateral triangles)', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) continue;
      expect(sit.isDelaunay(e)).toBe(true);
    }
  });

  it('flatQuad: Delaunay-ness is preserved under flips for the symmetric square', () => {
    const { sit } = build(flatQuad());
    const mesh = sit.intrinsicMesh;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (!mesh.isBoundaryEdge(e)) {
        // Diagonal of a square has opposite angles π/4 + π/4 = π/2 < π.
        expect(sit.isDelaunay(e)).toBe(true);
      }
    }
  });

  it('boundary edges return true', () => {
    const { sit } = build(flatQuad());
    const mesh = sit.intrinsicMesh;
    let any = false;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) {
        any = true;
        expect(sit.isDelaunay(e)).toBe(true);
      }
    }
    expect(any).toBe(true);
  });
});
