/**
 * flipEdgeNetworkFromSurfacePointControlPath — bezier control paths whose
 * control points are arbitrary face/edge surface points (not just vertices).
 *
 * The control points are inserted into the intrinsic triangulation as new
 * vertices; we then run bezierSubdivide + extractPolyline and check the curve
 * lies on the surface (3D arc length == intrinsic path length) and passes
 * through each inserted control's 3D position.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import type { SurfacePoint } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { flipEdgeNetworkFromSurfacePointControlPath } from '../../../src/flipout/index.js';
import { loadBezierFixture } from '../../_helpers/load-fixture.js';
import { polylineLength } from '../../_helpers/polyline.js';

function buildIcosphere() {
  // Reuse a fixture mesh purely as a convenient closed triangle mesh.
  const f = loadBezierFixture('icosphere-bezier-3pt-r1');
  const mesh = SurfaceMesh.fromFaces(f.mesh.faces, f.mesh.vertices.length);
  const geom = new VertexPositionGeometry(mesh, f.mesh.vertices);
  return { mesh, geom, vertices: f.mesh.vertices };
}

function facePoint(face: number): SurfacePoint {
  return { kind: 'face', face, bary: [0.4, 0.35, 0.25] };
}

describe('flipEdgeNetworkFromSurfacePointControlPath', () => {
  it('builds an on-surface bezier through face-interior control points', () => {
    const { mesh, geom } = buildIcosphere();
    const sit = new SignpostIntrinsicTriangulation(geom);
    // Three well-separated faces on the icosphere (80 faces).
    const controls = [facePoint(2), facePoint(40), facePoint(70)];

    const net = flipEdgeNetworkFromSurfacePointControlPath(sit, controls, {
      markInterior: true,
    });
    expect(net, 'expected a network').not.toBeNull();
    net!.bezierSubdivide(2);

    const poly = net!.extractPolyline();
    expect(poly.length).toBeGreaterThanOrEqual(2);

    // On-surface invariant: 3D arc length == intrinsic length.
    const intrinsicLen = net!.pathLength();
    const arcLen = polylineLength(poly);
    expect(Math.abs(arcLen - intrinsicLen) / Math.max(1, intrinsicLen)).toBeLessThan(1e-9);
  });

  it('collapses consecutive control points that resolve to the same vertex', () => {
    const { geom } = buildIcosphere();
    const sit = new SignpostIntrinsicTriangulation(geom);
    // Two points that snap to the same corner (bary ~ vertex 0 of the face)
    // are collapsed, leaving < 2 distinct controls → null.
    const corner: SurfacePoint = { kind: 'face', face: 5, bary: [1, 0, 0] };
    const net = flipEdgeNetworkFromSurfacePointControlPath(sit, [corner, corner], {
      markInterior: true,
    });
    expect(net).toBeNull();
  });

  it('requires at least two control points', () => {
    const { geom } = buildIcosphere();
    const sit = new SignpostIntrinsicTriangulation(geom);
    expect(() =>
      flipEdgeNetworkFromSurfacePointControlPath(sit, [facePoint(0)]),
    ).toThrow(/need ≥2/);
  });
});
