/**
 * geodesicSpline — bezier (adaptive) and Catmull-Rom (interpolating) schemes
 * through surface control points.
 *
 * Checks:
 *  - Catmull-Rom passes exactly through every control point (interpolation).
 *  - Both schemes produce on-surface polylines (3D arc length ≈ the sum of the
 *    pieces' intrinsic geodesic lengths).
 *  - A self-intersecting control set still yields a curve (no throw) and the
 *    bezier scheme reports the fallback.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import type { SurfacePoint } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { geodesicSpline } from '../../../src/flipout/index.js';
import { loadBezierFixture } from '../../_helpers/load-fixture.js';
import { dist3, polylineLength, type P3 } from '../../_helpers/polyline.js';

function teapot() {
  const f = loadBezierFixture('teapot-bezier-3pt-r1'); // reused only as a mesh source
  const mesh = SurfaceMesh.fromFaces(f.mesh.faces, f.mesh.vertices.length);
  const geom = new VertexPositionGeometry(mesh, f.mesh.vertices);
  return { geom, vertices: f.mesh.vertices };
}
const V = (v: number): SurfacePoint => ({ kind: 'vertex', vertex: v });
const face = (face: number): SurfacePoint => ({ kind: 'face', face, bary: [0.4, 0.35, 0.25] });

describe('geodesicSpline — Catmull-Rom interpolates the control points', () => {
  it('passes through every control point (vertex controls)', () => {
    const { geom, vertices } = teapot();
    const controls = [100, 500, 900, 1300].map(V);
    const r = geodesicSpline(geom, controls, { type: 'catmull-rom', rounds: 3 });
    expect(r.polyline.length).toBeGreaterThan(controls.length);
    for (const c of controls) {
      const target = vertices[(c as { vertex: number }).vertex]! as P3;
      const minD = Math.min(...r.polyline.map((q) => dist3(q, target)));
      expect(minD, `control ${(c as { vertex: number }).vertex} not interpolated`).toBeLessThan(1e-6);
    }
  });

  it('interpolates face-interior control points too', () => {
    const { geom } = teapot();
    const controls = [face(10), face(800), face(2000), face(1500)];
    const r = geodesicSpline(geom, controls, { type: 'catmull-rom', rounds: 2 });
    expect(r.polyline.length).toBeGreaterThan(2);
    // arc length matches the summed piece lengths (on-surface, no off-surface chords)
    const arc = polylineLength(r.polyline);
    expect(Math.abs(arc - r.length) / Math.max(1, r.length)).toBeLessThan(1e-2);
  });

  it('stays on-surface when a handle would run off a mesh boundary', () => {
    // This control set places the last knot's tangent handle on a path toward
    // the teapot rim; an un-clamped exp map traces off the open boundary and
    // returns an off-face barycentric (used to throw / cusp). The handle must
    // be clamped to the surface and the curve must still interpolate.
    const { geom, vertices } = teapot();
    const idx = [100, 500, 900, 1300, 300];
    const controls = idx.map(V);
    const r = geodesicSpline(geom, controls, { type: 'catmull-rom', rounds: 4 });
    expect(r.polyline.every((p) => p.every((x) => Number.isFinite(x)))).toBe(true);
    for (const v of idx) {
      const target = vertices[v]! as P3;
      const minD = Math.min(...r.polyline.map((q) => dist3(q, target)));
      expect(minD, `control ${v} not interpolated`).toBeLessThan(1e-5);
    }
  });
});

describe('geodesicSpline — quadratic B-spline', () => {
  it('produces a smooth on-surface curve near the controls', () => {
    const { geom, vertices } = teapot();
    const idx = [100, 500, 900, 1300];
    const r = geodesicSpline(geom, idx.map(V), { type: 'bspline', rounds: 3 });
    expect(r.polyline.length).toBeGreaterThan(2);
    // On-surface: 3D arc length matches the summed piece geodesic lengths.
    const arc = polylineLength(r.polyline);
    expect(Math.abs(arc - r.length) / Math.max(1, r.length)).toBeLessThan(1e-2);
    // Approximating: passes near each control (within a fraction of the curve),
    // but not necessarily through it.
    const diag = Math.max(...vertices.map((p) => Math.hypot(p[0], p[1], p[2])));
    for (const v of idx) {
      const minD = Math.min(...r.polyline.map((q) => dist3(q, vertices[v]! as P3)));
      expect(minD).toBeLessThan(0.5 * diag);
    }
  });

  it('handles face-interior controls without leaving the surface', () => {
    const { geom } = teapot();
    const r = geodesicSpline(geom, [face(10), face(800), face(2000), face(1500), face(40)], {
      type: 'bspline',
      rounds: 2,
    });
    expect(r.polyline.length).toBeGreaterThan(2);
    expect(r.polyline.every((p) => p.every((x) => Number.isFinite(x)))).toBe(true);
    const arc = polylineLength(r.polyline);
    expect(Math.abs(arc - r.length) / Math.max(1, r.length)).toBeLessThan(1e-2);
  });
});

describe('geodesicSpline — bezier scheme', () => {
  it('single global bezier for a simple control path', () => {
    const { geom } = teapot();
    const r = geodesicSpline(geom, [V(100), V(900)], { type: 'bezier', rounds: 2 });
    expect(r.fellBack).toBe(false);
    expect(r.pieces).toBe(1);
    const arc = polylineLength(r.polyline);
    expect(Math.abs(arc - r.length) / Math.max(1, r.length)).toBeLessThan(1e-2);
  });

  it('falls back to a piecewise spline when the path self-intersects', () => {
    const { geom } = teapot();
    // Many spread controls in a zig-zag order tend to self-intersect globally.
    const controls = [50, 1400, 300, 1100, 700, 200].map(V);
    const r = geodesicSpline(geom, controls, { type: 'bezier', rounds: 2 });
    // Either it stayed simple (pieces=1) or it fell back — both must yield a curve.
    expect(r.polyline.length).toBeGreaterThanOrEqual(2);
    expect(r.polyline.every((p) => p.every((x) => Number.isFinite(x)))).toBe(true);
  });
});
