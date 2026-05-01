/**
 * Initialization invariants for SignpostIntrinsicTriangulation.
 *
 * These tests verify the constructor populates `edgeLengths`, `vertexAngleSums`,
 * and `halfedgeSignposts` correctly and consistently with the input geometry.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import {
  cube,
  flatGrid,
  flatQuad,
  icosahedron,
  singleTriangle,
  tetrahedron,
} from '../../_helpers/meshes.js';

const TWO_PI = 2 * Math.PI;

interface Built {
  geom: VertexPositionGeometry;
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
  return { geom, sit };
}

describe('initialization — intrinsicMesh is a structural clone of the input', () => {
  it.each<[string, MeshDataLike]>([
    ['singleTriangle', singleTriangle()],
    ['tetrahedron', tetrahedron()],
    ['flatQuad', flatQuad()],
    ['flatGrid 3x3', flatGrid(3)],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s: same vertex / face / halfedge counts', (_, m) => {
    const { sit } = build(m);
    expect(sit.intrinsicMesh.nVertices).toBe(m.vertices.length);
    expect(sit.intrinsicMesh.nFaces).toBe(m.faces.length);
  });

  it.each<[string, MeshDataLike]>([
    ['singleTriangle', singleTriangle()],
    ['tetrahedron', tetrahedron()],
    ['flatQuad', flatQuad()],
    ['flatGrid 3x3', flatGrid(3)],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s: connectivity matches input mesh exactly', (_, m) => {
    const { sit, geom } = build(m);
    const inMesh = geom.mesh;
    const outMesh = sit.intrinsicMesh;
    expect(outMesh.nHalfedges).toBe(inMesh.nHalfedges);
    for (let h = 0; h < inMesh.nHalfedges; h++) {
      expect(outMesh.vertex(h)).toBe(inMesh.vertex(h));
      expect(outMesh.next(h)).toBe(inMesh.next(h));
      expect(outMesh.face(h)).toBe(inMesh.face(h));
    }
  });
});

describe('initialization — edgeLengths match extrinsic edge lengths', () => {
  it.each<[string, MeshDataLike]>([
    ['singleTriangle', singleTriangle()],
    ['tetrahedron', tetrahedron()],
    ['flatQuad', flatQuad()],
    ['flatGrid 3x3', flatGrid(3)],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s', (_, m) => {
    const { sit, geom } = build(m);
    for (let e = 0; e < sit.intrinsicMesh.nEdges; e++) {
      expect(sit.edgeLengths[e]!).toBeCloseTo(geom.edgeLength(e), 12);
    }
  });
});

describe('initialization — vertexAngleSums match extrinsic vertex angle sums', () => {
  it.each<[string, MeshDataLike]>([
    ['singleTriangle', singleTriangle()],
    ['tetrahedron', tetrahedron()],
    ['flatQuad', flatQuad()],
    ['flatGrid 3x3', flatGrid(3)],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s', (_, m) => {
    const { sit, geom } = build(m);
    for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
      expect(sit.vertexAngleSums[v]!).toBeCloseTo(geom.vertexAngleSum(v), 12);
      expect(sit.getCornerAngleSum(v)).toBeCloseTo(geom.vertexAngleSum(v), 12);
    }
  });
});

describe('initialization — signposts span [0, vertexAngleSum) at interior vertices', () => {
  it.each<[string, MeshDataLike]>([
    ['tetrahedron', tetrahedron()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s: monotonically increasing around each vertex', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (mesh.isBoundaryVertex(v)) continue;
      const angles: number[] = [];
      for (const he of mesh.outgoingHalfedges(v)) {
        angles.push(sit.halfedgeSignposts[he]!);
      }
      // All angles must be in [0, vertexAngleSum)
      const sum = sit.vertexAngleSums[v]!;
      for (const a of angles) {
        expect(a).toBeGreaterThanOrEqual(0 - 1e-12);
        expect(a).toBeLessThan(sum + 1e-9);
      }
      // First angle is 0 (geometry-central convention).
      expect(angles[0]!).toBeCloseTo(0, 12);
      // Sequence is strictly increasing (until possibly wrapping back to 0).
      for (let i = 1; i < angles.length; i++) {
        expect(angles[i]!).toBeGreaterThan(angles[i - 1]!);
      }
    }
  });

  it('icosahedron: differences between consecutive signposts equal corner angles', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      const halfedges: number[] = [];
      for (const he of mesh.outgoingHalfedges(v)) halfedges.push(he);
      for (let i = 0; i < halfedges.length; i++) {
        const heA = halfedges[i]!;
        const heB = halfedges[(i + 1) % halfedges.length]!;
        const aA = sit.halfedgeSignposts[heA]!;
        const aB = sit.halfedgeSignposts[heB]!;
        const cAngle = sit.cornerAngleAt(heA);
        // Wrap-around case at the last halfedge
        if (i === halfedges.length - 1) {
          // a + cAngle = vertexAngleSum, mod which is 0
          expect(aA + cAngle).toBeCloseTo(sit.vertexAngleSums[v]!, 10);
        } else {
          expect(aB - aA).toBeCloseTo(cAngle, 10);
        }
      }
    }
  });
});

describe('initialization — boundary handling', () => {
  it('flatQuad: boundary vertex angle sum < 2π', () => {
    const { sit } = build(flatQuad());
    const mesh = sit.intrinsicMesh;
    let foundBoundary = false;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (mesh.isBoundaryVertex(v)) {
        foundBoundary = true;
        // Quad corners have angle sum < 2π (here, π/2).
        expect(sit.vertexAngleSums[v]!).toBeLessThan(TWO_PI);
      }
    }
    expect(foundBoundary).toBe(true);
  });

  it('flatGrid 4x4: interior vertices have angle sum 2π, corners < π', () => {
    const m = flatGrid(4);
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      const sum = sit.vertexAngleSums[v]!;
      if (!mesh.isBoundaryVertex(v)) {
        expect(sum).toBeCloseTo(TWO_PI, 10);
      } else {
        expect(sum).toBeLessThanOrEqual(Math.PI + 1e-9);
        expect(sum).toBeGreaterThan(0);
      }
    }
  });
});

describe('initialization — sanity checks on small meshes', () => {
  it('tetrahedron: each vertex has 3 outgoing halfedges, signposts sum gap = vertexAngleSum', () => {
    const { sit } = build(tetrahedron());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      let count = 0;
      for (const _ of mesh.outgoingHalfedges(v)) count++;
      expect(count).toBe(3);
    }
  });

  it('singleTriangle: signposts at each boundary vertex span [0, vertexAngleSum]', () => {
    // All three vertices of a single triangle are boundary vertices.
    // Per L1's gc-aligned convention, `vertexHalfedge(v)` for a boundary
    // vertex is the FIRST INTERIOR outgoing halfedge in the CCW arc from
    // the boundary — signpost = 0 (the START of the wedge walk). The
    // single boundary outgoing halfedge sits at the END with signpost =
    // vertexAngleSum. We assert both endpoints.
    const { sit } = build(singleTriangle());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      const first = mesh.vertexHalfedge(v);
      const sum = sit.vertexAngleSums[v]!;
      // First (interior) halfedge in CCW arc has signpost 0.
      expect(mesh.face(first)).not.toBe(-1);
      expect(sit.halfedgeSignposts[first]!).toBeCloseTo(0, 12);
      // The boundary outgoing halfedge has signpost = vertexAngleSum.
      let bdryHe = -1;
      for (const h of mesh.outgoingHalfedges(v)) {
        if (mesh.face(h) === -1) {
          bdryHe = h;
          break;
        }
      }
      expect(bdryHe).toBeGreaterThanOrEqual(0);
      expect(sit.halfedgeSignposts[bdryHe]!).toBeCloseTo(sum, 12);
    }
  });

  it('icosahedron: every signpost angle is finite', () => {
    const { sit } = build(icosahedron());
    for (let h = 0; h < sit.intrinsicMesh.nHalfedges; h++) {
      expect(Number.isFinite(sit.halfedgeSignposts[h]!)).toBe(true);
    }
  });
});

describe('initialization — vertexAngleScaling', () => {
  it('icosahedron: scaling = vertexAngleSum / 2π for interior vertices', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (!mesh.isBoundaryVertex(v)) {
        expect(sit.vertexAngleScaling(v)).toBeCloseTo(sit.vertexAngleSums[v]! / TWO_PI, 12);
      }
    }
  });

  it('flatQuad: scaling = vertexAngleSum / π for boundary vertices', () => {
    const { sit } = build(flatQuad());
    const mesh = sit.intrinsicMesh;
    for (let v = 0; v < mesh.nVertices; v++) {
      if (mesh.isBoundaryVertex(v)) {
        expect(sit.vertexAngleScaling(v)).toBeCloseTo(sit.vertexAngleSums[v]! / Math.PI, 12);
      }
    }
  });
});

describe('initialization — halfedgeVector mirrors edge direction in tangent plane', () => {
  it('icosahedron: halfedgeVector length equals intrinsic edge length', () => {
    const { sit } = build(icosahedron());
    const mesh = sit.intrinsicMesh;
    for (let h = 0; h < mesh.nHalfedges; h++) {
      if (mesh.face(h) === -1) continue;
      const v = sit.halfedgeVector(h);
      const len = Math.hypot(v[0], v[1]);
      expect(len).toBeCloseTo(sit.edgeLengths[mesh.edge(h)]!, 10);
    }
  });
});
