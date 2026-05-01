import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { FlipEdgeNetwork, flipOutPath } from '../../../src/flipout/index.js';
import { flatQuad, tetrahedron, cube, flatGrid } from '../../_helpers/meshes.js';

interface MeshLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function build(m: MeshLike): SignpostIntrinsicTriangulation {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  return new SignpostIntrinsicTriangulation(geom);
}

describe('FlipEdgeNetwork — smoke', () => {
  it('flatQuad 0->2: direct diagonal already geodesic, length √2', () => {
    const sit = build(flatQuad());
    const result = flipOutPath(sit, 0, 2);
    expect(result.converged).toBe(true);
    expect(result.length).toBeCloseTo(Math.sqrt(2), 9);
  });

  it('flatQuad 1->3: forces a flip across the diagonal', () => {
    const sit = build(flatQuad());
    // The initial path must visit either v=0 or v=2; FlipOut should straighten
    // it to length √2.
    const result = flipOutPath(sit, 1, 3);
    expect(result.converged).toBe(true);
    expect(result.length).toBeCloseTo(Math.sqrt(2), 6);
  });

  it('tetrahedron 0->1: just one edge, no flips needed', () => {
    const sit = build(tetrahedron());
    const result = flipOutPath(sit, 0, 1);
    expect(result.converged).toBe(true);
    expect(result.length).toBeCloseTo(2 * Math.sqrt(2), 6);
  });

  it('tetrahedron 0->3: shortest path of one edge length 2√2', () => {
    const sit = build(tetrahedron());
    const result = flipOutPath(sit, 0, 3);
    expect(result.converged).toBe(true);
    expect(result.length).toBeCloseTo(2 * Math.sqrt(2), 6);
  });

  it('cube 0->1: straight along an edge, length 1', () => {
    const sit = build(cube());
    const result = flipOutPath(sit, 0, 1);
    expect(result.converged).toBe(true);
    expect(result.length).toBeCloseTo(1, 6);
  });

  it('cube 0->6: cube space diagonal — geodesic √5 (across two adjacent faces)', () => {
    const sit = build(cube());
    const result = flipOutPath(sit, 0, 6);
    expect(result.converged).toBe(true);
    // Unfolding two adjacent faces: distance = sqrt(1^2 + 2^2) = sqrt(5).
    expect(result.length).toBeCloseTo(Math.sqrt(5), 4);
  });

  it('flatGrid(4,1) 0->15: diagonal of unit square = √2', () => {
    const sit = build(flatGrid(4, 1));
    const result = flipOutPath(sit, 0, 15);
    expect(result.converged).toBe(true);
    expect(result.length).toBeCloseTo(Math.sqrt(2), 4);
  });

  it('FlipOut on already-straight path: zero flips', () => {
    const sit = build(flatQuad());
    // Initial path = the direct diagonal halfedge from 0 to 2.
    const m = sit.intrinsicMesh;
    let diag = -1;
    for (const he of m.outgoingHalfedges(0)) {
      if (m.tipVertex(he) === 2) {
        diag = he;
        break;
      }
    }
    expect(diag).not.toBe(-1);
    const network = new FlipEdgeNetwork(sit, [diag]);
    const result = network.flipOut();
    expect(result.converged).toBe(true);
    expect(network.nFlips).toBe(0);
    expect(network.pathLength()).toBeCloseTo(Math.sqrt(2), 9);
  });
});
