/**
 * Dijkstra (shortestEdgePath) unit tests.
 *
 * Mirrors `tools/gen_fixtures.py`'s expected initial path on small graphs
 * we can hand-verify, plus invariants that must hold for any input.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { shortestEdgePath } from '../../../src/flipout/dijkstra.js';
import { cube, flatGrid, flatQuad, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

interface MeshLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function build(m: MeshLike): SignpostIntrinsicTriangulation {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  return new SignpostIntrinsicTriangulation(geom);
}

describe('shortestEdgePath — basics', () => {
  it('returns null for src === dst', () => {
    const sit = build(flatQuad());
    expect(shortestEdgePath(sit, 0, 0)).toBeNull();
  });

  it('flatQuad 0 -> 1: single direct edge (length 1)', () => {
    const sit = build(flatQuad());
    const path = shortestEdgePath(sit, 0, 1);
    expect(path).not.toBeNull();
    expect(path!).toHaveLength(1);
    const m = sit.intrinsicMesh;
    const he = path![0]!;
    expect(m.vertex(he)).toBe(0);
    expect(m.tipVertex(he)).toBe(1);
  });

  it('flatQuad 0 -> 2: direct diagonal (length √2)', () => {
    const sit = build(flatQuad());
    const path = shortestEdgePath(sit, 0, 2);
    expect(path).not.toBeNull();
    expect(path!).toHaveLength(1);
    const m = sit.intrinsicMesh;
    const he = path![0]!;
    expect(m.vertex(he)).toBe(0);
    expect(m.tipVertex(he)).toBe(2);
  });

  it('cube 0 -> 6 (space diagonal): finds a path along mesh edges', () => {
    const sit = build(cube());
    const path = shortestEdgePath(sit, 0, 6);
    expect(path).not.toBeNull();
    // Cube faces are triangulated; the SP travels along face diagonals
    // (length √2) and unit edges. Two face diagonals = 2√2 ≈ 2.83.
    let len = 0;
    for (const he of path!) {
      len += sit.edgeLengths[sit.intrinsicMesh.edge(he)]!;
    }
    expect(len).toBeLessThanOrEqual(2 * Math.sqrt(2) + 1e-9);
    expect(len).toBeGreaterThan(0);
  });

  it('produces tip-to-tail connected halfedge sequences (general invariant)', () => {
    const sit = build(icosahedron());
    const path = shortestEdgePath(sit, 0, 3);
    expect(path).not.toBeNull();
    const m = sit.intrinsicMesh;
    expect(m.vertex(path![0]!)).toBe(0);
    for (let i = 0; i + 1 < path!.length; i++) {
      expect(m.tipVertex(path![i]!)).toBe(m.vertex(path![i + 1]!));
    }
    expect(m.tipVertex(path![path!.length - 1]!)).toBe(3);
  });

  it('grid 0 -> 15 (corner to corner): produces a path of finite length', () => {
    const sit = build(flatGrid(4, 1));
    const path = shortestEdgePath(sit, 0, 15);
    expect(path).not.toBeNull();
    let len = 0;
    for (const he of path!) {
      len += sit.edgeLengths[sit.intrinsicMesh.edge(he)]!;
    }
    expect(Number.isFinite(len)).toBe(true);
    expect(len).toBeGreaterThan(0);
  });

  it('tetrahedron 0 -> 1: single edge', () => {
    const sit = build(tetrahedron());
    const path = shortestEdgePath(sit, 0, 1);
    expect(path).not.toBeNull();
    expect(path!).toHaveLength(1);
  });

  it('throws on out-of-range vertex', () => {
    const sit = build(flatQuad());
    expect(() => shortestEdgePath(sit, -1, 0)).toThrow();
    expect(() => shortestEdgePath(sit, 0, 100)).toThrow();
  });
});
