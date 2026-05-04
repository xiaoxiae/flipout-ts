import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import {
  flipEdgeNetworkFromControlPath,
  shortestEdgePath,
} from '../../../src/flipout/index.js';
import { flatGrid, icosahedron } from '../../_helpers/meshes.js';

interface MeshLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function buildSit(m: MeshLike): SignpostIntrinsicTriangulation {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  return new SignpostIntrinsicTriangulation(geom);
}

describe('flipEdgeNetworkFromControlPath', () => {
  it('throws on fewer than 2 control vertices', () => {
    const sit = buildSit(flatGrid(3));
    expect(() => flipEdgeNetworkFromControlPath(sit, [])).toThrow();
    expect(() => flipEdgeNetworkFromControlPath(sit, [0])).toThrow();
  });

  it('two-vertex case matches a single shortestEdgePath call', () => {
    const sit = buildSit(flatGrid(3));
    const expected = shortestEdgePath(sit, 0, 8)!;
    const net = flipEdgeNetworkFromControlPath(sit, [0, 8])!;
    expect(net).not.toBeNull();
    expect(net.pathHalfedges()).toEqual(expected);
  });

  it('three-vertex case concatenates the two Dijkstra segments', () => {
    const sit = buildSit(flatGrid(3));
    const segA = shortestEdgePath(sit, 0, 4)!;
    const segB = shortestEdgePath(sit, 4, 8)!;
    const net = flipEdgeNetworkFromControlPath(sit, [0, 4, 8], {
      markInterior: true,
    })!;
    expect(net).not.toBeNull();
    expect(net.pathHalfedges()).toEqual([...segA, ...segB]);
  });

  it('markInterior=true marks every control vertex', () => {
    const sit = buildSit(flatGrid(3));
    const ctrl = [0, 4, 8];
    const net = flipEdgeNetworkFromControlPath(sit, ctrl, {
      markInterior: true,
    })!;
    for (const v of ctrl) {
      expect(net.isMarkedVertex(v), `vertex ${v} should be marked`).toBe(true);
    }
  });

  it('markInterior=false leaves all vertices unmarked', () => {
    const sit = buildSit(flatGrid(3));
    const net = flipEdgeNetworkFromControlPath(sit, [0, 4, 8])!;
    for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
      expect(net.isMarkedVertex(v)).toBe(false);
    }
  });

  it('markInterior=false applies back-and-forth cleanup', () => {
    // Pick three colinear control points on a flat grid such that the
    // Dijkstra path A→B→A reverses and cancels (paranoia test: even
    // minimal redundancy gets collapsed).
    const sit = buildSit(flatGrid(3));
    const net = flipEdgeNetworkFromControlPath(sit, [0, 4, 0])!;
    // After cleanup the path should be empty (every step gets cancelled)
    // — but we return null when the path is empty after cleanup.
    expect(net).toBeNull();
  });

  it('disconnected pair returns null', () => {
    // Same source and dest = degenerate Dijkstra (returns null upstream).
    const sit = buildSit(flatGrid(3));
    expect(flipEdgeNetworkFromControlPath(sit, [3, 3, 5])).toBeNull();
  });

  it('post-construction flipOut on bezier-style setup pins control points', () => {
    // 3 control vertices on the icosahedron, marked, flag off → flipOut is
    // a no-op (every junction is a control point) — pathLength is preserved.
    const sit = buildSit(icosahedron());
    const net = flipEdgeNetworkFromControlPath(sit, [0, 5, 11], {
      markInterior: true,
    })!;
    net.straightenAroundMarkedVertices = false;
    const before = net.pathLength();
    const r = net.flipOut(10_000);
    expect(r.converged).toBe(true);
    expect(net.pathLength()).toBeCloseTo(before, 9);
  });
});
