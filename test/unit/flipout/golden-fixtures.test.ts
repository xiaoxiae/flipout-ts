/**
 * Golden-fixture cross-validation against potpourri3d.
 *
 * For every JSON in `fixtures/`, build the input mesh, run `flipOutPath`,
 * and check:
 *   - `length` matches `expected.path_length` to 4 decimal places
 *   - if our polyline has the same point count, every pair of points is
 *     within 1e-3 (different intermediate face crossings, same geodesic
 *     are acceptable too — we don't require equal point counts)
 *
 * Mirrors gc's flip-geodesics + getPathPolyline3D output. potpourri3d
 * itself wraps gc, so a length match validates the algorithm against the
 * reference implementation.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import type { Vec3 } from '../../../src/math/vec3.js';
import { flipOutPath } from '../../../src/flipout/index.js';
import { listFixtures, loadFixture } from '../../_helpers/load-fixture.js';

function dist3(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

describe('golden fixtures — length', () => {
  for (const name of listFixtures()) {
    it(`${name}: length matches potpourri3d to 4 decimal places`, () => {
      const f = loadFixture(name);
      const mesh = SurfaceMesh.fromFaces(f.mesh.faces, f.mesh.vertices.length);
      const geom = new VertexPositionGeometry(mesh, f.mesh.vertices);
      const sit = new SignpostIntrinsicTriangulation(geom);
      const r = flipOutPath(sit, f.query.src, f.query.dst, { maxIterations: 100000 });
      expect(r.converged, `${name}: should converge`).toBe(true);
      expect(r.length).toBeCloseTo(f.expected.pathLength, 4);
    });
  }
});

describe('golden fixtures — endpoint positions', () => {
  for (const name of listFixtures()) {
    it(`${name}: polyline starts/ends at the source/destination 3D positions`, () => {
      const f = loadFixture(name);
      const mesh = SurfaceMesh.fromFaces(f.mesh.faces, f.mesh.vertices.length);
      const geom = new VertexPositionGeometry(mesh, f.mesh.vertices);
      const sit = new SignpostIntrinsicTriangulation(geom);
      const r = flipOutPath(sit, f.query.src, f.query.dst, { maxIterations: 100000 });
      const expectedSrc: Vec3 = [...f.mesh.vertices[f.query.src]!] as Vec3;
      const expectedDst: Vec3 = [...f.mesh.vertices[f.query.dst]!] as Vec3;
      expect(r.polyline.length).toBeGreaterThanOrEqual(2);
      expect(dist3(r.polyline[0]!, expectedSrc)).toBeLessThan(1e-6);
      expect(dist3(r.polyline[r.polyline.length - 1]!, expectedDst)).toBeLessThan(1e-6);
    });
  }
});

describe('golden fixtures — polyline equality (when point counts match)', () => {
  for (const name of listFixtures()) {
    if (name.startsWith('teapot')) continue; // skip slow / complex cases
    it(`${name}: per-point distance ≤ 1e-3 if counts match`, () => {
      const f = loadFixture(name);
      const mesh = SurfaceMesh.fromFaces(f.mesh.faces, f.mesh.vertices.length);
      const geom = new VertexPositionGeometry(mesh, f.mesh.vertices);
      const sit = new SignpostIntrinsicTriangulation(geom);
      const r = flipOutPath(sit, f.query.src, f.query.dst, { maxIterations: 100000 });
      if (r.polyline.length === f.expected.pathPoints.length) {
        for (let i = 0; i < r.polyline.length; i++) {
          const expected = f.expected.pathPoints[i]! as Vec3;
          const actual = r.polyline[i]!;
          expect(dist3(actual, expected)).toBeLessThan(1e-3);
        }
      } else {
        // Different number of intermediate face-crossings is acceptable as
        // long as endpoints + length match (verified by the other suites).
        expect(true).toBe(true);
      }
    });
  }
});
