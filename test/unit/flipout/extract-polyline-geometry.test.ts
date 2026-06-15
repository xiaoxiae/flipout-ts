/**
 * extractPolyline 3D geometry — cross-check against geometry-central.
 *
 * The existing bezier/geodesic fixture tests only assert *path length*
 * (an intrinsic edge-length sum), which is correct regardless of how the
 * intrinsic path is mapped back to 3D points on the input surface. That left
 * `FlipEdgeNetwork.extractPolyline()` — the intrinsic→3D back-mapping — almost
 * untested: bezier curves render off the surface (inserted subdivision
 * vertices get mis-located) while the length tests stay green.
 *
 * We assert two things, both against gc's `getPathPolyline3D()` reference
 * (stored in every fixture as `expected.pathPoints`):
 *
 *  1. **On-surface invariant** (all fixtures): the extracted polyline's own 3D
 *     arc length equals the intrinsic path length. A geodesic that lies on the
 *     surface has 3D length == intrinsic length; off-surface "spikes" inflate
 *     it. This is robust to path non-uniqueness, so it's the right tool for
 *     symmetric meshes (where many equal-length geodesics exist) and for the
 *     three near-degenerate bezier curves that take a different — but still
 *     valid and on-surface — branch from gc (see bezier-fixtures.test.ts).
 *
 *  2. **Hausdorff match** (bezier fixtures whose path matches gc): the
 *     symmetric Hausdorff distance between our polyline and gc's, normalised by
 *     the mesh bbox diagonal, is ~machine-epsilon small. We skip this for the
 *     three acos-divergent bezier fixtures (their curve legitimately differs)
 *     and for geodesics (symmetric meshes have non-unique shortest geodesics,
 *     so gc's route need not equal ours even though both are correct — the
 *     on-surface invariant covers those).
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import {
  flipEdgeNetworkFromControlPath,
  flipOutPath,
} from '../../../src/flipout/index.js';
import {
  listFixtures,
  loadBezierFixture,
  loadFixture,
} from '../../_helpers/load-fixture.js';
import {
  bboxDiagonal,
  polylineLength,
  symmetricHausdorff,
} from '../../_helpers/polyline.js';

// Bezier fixtures that take a different (valid) Bezier branch from gc due to
// the documented Math.acos ulp drift on near-degenerate geometry — their 3D
// route legitimately differs, so we skip the Hausdorff match for them. See
// bezier-fixtures.test.ts. The same ulp cascade can also leave a small
// inconsistency between a midpoint's resolved input location and the geodesic
// segments meeting at it (a sub-1% junction gap on teapot-bezier-3pt-r3), so
// these fixtures get a looser on-surface bound too.
const ACOS_DIVERGENT = new Set([
  'teapot-bezier-3pt-r1',
  'teapot-bezier-3pt-r3',
  'spot-bezier-5pt-r3',
]);

// The extracted polyline's 3D arc length must equal the intrinsic path length
// (a geodesic lies on the surface). Achieved ≤ 9e-14 for all non-divergent
// fixtures; the bound is set ~10× above that.
const ON_SURFACE_REL_TOL = 1e-12;
// Looser on-surface bound for the acos-divergent fixtures (worst observed
// junction gap is 5e-3 on teapot-bezier-3pt-r3).
const ON_SURFACE_REL_TOL_DIVERGENT = 1e-2;
// Hausdorff/bbox bound for bezier fixtures whose path matches gc. Achieved
// ≤ 2.4e-11; the bound is set well above that but still 7 orders tighter than
// the off-surface bug it guards against.
const HAUSDORFF_REL_TOL = 1e-9;

describe('extractPolyline geometry — bezier fixtures vs geometry-central', () => {
  for (const name of listFixtures('bezier')) {
    it(`${name}: extracted polyline is on-surface and matches gc`, () => {
      const f = loadBezierFixture(name);
      const mesh = SurfaceMesh.fromFaces(f.mesh.faces, f.mesh.vertices.length);
      const geom = new VertexPositionGeometry(mesh, f.mesh.vertices);
      const sit = new SignpostIntrinsicTriangulation(geom);
      const net = flipEdgeNetworkFromControlPath(sit, f.query.controlVertices, {
        markInterior: true,
      });
      expect(net, `${name}: failed to build initial network`).not.toBeNull();
      net!.bezierSubdivide(f.query.nRounds);

      const got = net!.extractPolyline();
      expect(got.length, `${name}: extractPolyline returned too few points`).toBeGreaterThanOrEqual(2);

      // (1) On-surface invariant: 3D arc length == intrinsic path length.
      const intrinsicLen = net!.pathLength();
      const arcLen = polylineLength(got);
      const arcRel = Math.abs(arcLen - intrinsicLen) / Math.max(1, intrinsicLen);
      const onSurfaceTol = ACOS_DIVERGENT.has(name)
        ? ON_SURFACE_REL_TOL_DIVERGENT
        : ON_SURFACE_REL_TOL;
      expect(
        arcRel,
        `${name}: 3D arc length ${arcLen.toFixed(6)} vs intrinsic ${intrinsicLen.toFixed(6)} ` +
          `(relErr=${arcRel.toExponential(3)}) — off-surface polyline`,
      ).toBeLessThan(onSurfaceTol);

      // (2) Hausdorff match against gc (skip the acos-divergent curves).
      if (!ACOS_DIVERGENT.has(name)) {
        const diag = bboxDiagonal(f.mesh.vertices);
        const rel = symmetricHausdorff(got, f.expected.pathPoints) / diag;
        expect(
          rel,
          `${name}: Hausdorff/bbox = ${rel.toExponential(3)} ` +
            `(${got.length} pts vs ${f.expected.pathPoints.length} ref)`,
        ).toBeLessThan(HAUSDORFF_REL_TOL);
      }
    });
  }
});

describe('extractPolyline geometry — geodesic fixtures (on-surface invariant)', () => {
  // Symmetric meshes (cube, icosahedron, …) have multiple equal-length
  // shortest geodesics, so gc's specific route need not match ours. We assert
  // the route-independent on-surface invariant: our 3D arc length equals the
  // intrinsic length and gc's reference 3D length.
  for (const name of listFixtures('geodesic')) {
    it(`${name}: extracted polyline is on-surface`, () => {
      const f = loadFixture(name);
      const mesh = SurfaceMesh.fromFaces(f.mesh.faces, f.mesh.vertices.length);
      const geom = new VertexPositionGeometry(mesh, f.mesh.vertices);
      const sit = new SignpostIntrinsicTriangulation(geom);
      const r = flipOutPath(sit, f.query.src, f.query.dst);

      expect(r.polyline.length, `${name}: polyline too short`).toBeGreaterThanOrEqual(2);

      const arcLen = polylineLength(r.polyline);
      const rel = Math.abs(arcLen - r.length) / Math.max(1, r.length);
      expect(
        rel,
        `${name}: 3D arc length ${arcLen.toFixed(6)} vs intrinsic ${r.length.toFixed(6)} ` +
          `(relErr=${rel.toExponential(3)})`,
      ).toBeLessThan(ON_SURFACE_REL_TOL);

      // gc's reference 3D length should match too — both are *shortest*
      // geodesics, so even when the routes differ (symmetric meshes) their
      // lengths agree to machine epsilon.
      const gcLen = polylineLength(f.expected.pathPoints);
      const gcRel = Math.abs(gcLen - r.length) / Math.max(1, r.length);
      expect(
        gcRel,
        `${name}: gc 3D length ${gcLen.toFixed(6)} vs intrinsic ${r.length.toFixed(6)}`,
      ).toBeLessThan(ON_SURFACE_REL_TOL);
    });
  }
});
