import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { FlipEdgeNetwork, shortestEdgePath } from '../../../src/flipout/index.js';
import { flatQuad } from '../../_helpers/meshes.js';

interface MeshLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function buildSit(m: MeshLike): SignpostIntrinsicTriangulation {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  return new SignpostIntrinsicTriangulation(geom);
}

describe('FlipEdgeNetwork — marked vertices', () => {
  it('isMarkedVertex defaults to false for every vertex', () => {
    const sit = buildSit(flatQuad());
    const path = shortestEdgePath(sit, 1, 3)!;
    const net = new FlipEdgeNetwork(sit, path);
    for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
      expect(net.isMarkedVertex(v)).toBe(false);
    }
  });

  it('setMarkedVertex round-trips', () => {
    const sit = buildSit(flatQuad());
    const path = shortestEdgePath(sit, 1, 3)!;
    const net = new FlipEdgeNetwork(sit, path);
    net.setMarkedVertex(0, true);
    expect(net.isMarkedVertex(0)).toBe(true);
    net.setMarkedVertex(0, false);
    expect(net.isMarkedVertex(0)).toBe(false);
  });

  it('isMarkedVertex returns false for indices past the array', () => {
    const sit = buildSit(flatQuad());
    const path = shortestEdgePath(sit, 1, 3)!;
    const net = new FlipEdgeNetwork(sit, path);
    expect(net.isMarkedVertex(10_000_000)).toBe(false);
  });

  it('setMarkedVertex grows storage to accommodate large indices', () => {
    const sit = buildSit(flatQuad());
    const path = shortestEdgePath(sit, 1, 3)!;
    const net = new FlipEdgeNetwork(sit, path);
    const big = sit.intrinsicMesh.nVertices + 5000;
    net.setMarkedVertex(big, true);
    expect(net.isMarkedVertex(big)).toBe(true);
    expect(net.isMarkedVertex(big - 1)).toBe(false);
  });

  it('default (straightenAroundMarkedVertices=true) — flag is no-op even with marked junction', () => {
    // flatQuad 1->3 forces a flip across the diagonal. Marking the
    // junction vertex (0 or 2) shouldn't matter when the flag is true:
    // the path still straightens to the diagonal.
    const sit = buildSit(flatQuad());
    const path = shortestEdgePath(sit, 1, 3)!;
    const net = new FlipEdgeNetwork(sit, path);
    expect(net.straightenAroundMarkedVertices).toBe(true);
    // Mark whichever junction is on the initial Dijkstra path (0 or 2).
    const m = sit.intrinsicMesh;
    for (const he of path) {
      const tail = m.vertex(he);
      if (tail !== 1 && tail !== 3) net.setMarkedVertex(tail, true);
    }
    const r = net.flipOut(10_000);
    expect(r.converged).toBe(true);
    expect(net.pathLength()).toBeCloseTo(Math.sqrt(2), 9);
  });

  it('straightenAroundMarkedVertices=false — pinned junction blocks the flip', () => {
    // Same setup, but with the flag off and the junction marked, the
    // flip is blocked: pathLength stays at the Dijkstra distance (= 2).
    const sit = buildSit(flatQuad());
    const path = shortestEdgePath(sit, 1, 3)!;
    const m = sit.intrinsicMesh;
    const net = new FlipEdgeNetwork(sit, path);
    for (const he of path) {
      const tail = m.vertex(he);
      if (tail !== 1 && tail !== 3) net.setMarkedVertex(tail, true);
    }
    net.straightenAroundMarkedVertices = false;
    const lengthBefore = net.pathLength();
    expect(lengthBefore).toBeCloseTo(2, 9); // 1->0->3 or 1->2->3
    const r = net.flipOut(10_000);
    expect(r.converged).toBe(true);
    expect(net.pathLength()).toBeCloseTo(lengthBefore, 9);
    expect(net.pathLength()).toBeGreaterThan(Math.sqrt(2) + 1e-3);
  });
});
