// Ported from geometry-central:
//   include/geometrycentral/utilities/utilities.h            (Heron / cosine rule)
//   src/surface/intrinsic_geometry_interface.cpp             (corner angles)
//   src/surface/vertex_position_geometry.cpp                 (face areas)
//
// Triangle math used throughout the intrinsic-triangulation layer. Length-only
// formulations are critical: intrinsic triangulations only carry edge lengths,
// not embedded vertex positions, so face areas and corner angles must be
// computable from `(l_ab, l_bc, l_ca)` alone.

import type { Vec3 } from './vec3.js';
import { cross, dot, norm, sub } from './vec3.js';

/**
 * Area of the triangle with the given embedded vertex positions, computed via
 * the cross-product magnitude `|AB × AC| / 2`. Returns `0` for degenerate
 * (collinear) inputs.
 */
export function triangleArea(a: Vec3, b: Vec3, c: Vec3): number {
  const ab = sub(b, a);
  const ac = sub(c, a);
  return 0.5 * norm(cross(ab, ac));
}

/**
 * Area of a triangle from its three side lengths, using Kahan's numerically
 * stable rearrangement of Heron's formula:
 *
 *   sort the lengths so a ≥ b ≥ c, then
 *   area = sqrt( (a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c)) ) / 4
 *
 * The classic Heron formula `sqrt(s(s-a)(s-b)(s-c))` loses catastrophic digits
 * for sliver triangles; this variant is well-conditioned for all inputs that
 * satisfy the triangle inequality. Slightly negative radicands (caused by FP
 * round-off on near-degenerate triangles) are clamped to 0.
 *
 * Reference: W. Kahan, "Miscalculating Area and Angles of a Needle-like
 * Triangle" (2014).
 */
export function triangleAreaFromLengths(lAB: number, lBC: number, lCA: number): number {
  // Sort so a ≥ b ≥ c without mutation in the public API.
  const sorted = [lAB, lBC, lCA].slice().sort((x, y) => y - x);
  const a = sorted[0]!;
  const b = sorted[1]!;
  const c = sorted[2]!;

  const radicand = (a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c));
  if (radicand <= 0) return 0;
  return Math.sqrt(radicand) * 0.25;
}

/**
 * Interior angle, in radians, at the vertex between the two sides of length
 * `lA` and `lB`, opposite the third side of length `lOpposite`. Standard
 * cosine rule:
 *
 *   cos(theta) = (lA^2 + lB^2 - lOpposite^2) / (2 * lA * lB)
 *
 * The acos input is clamped to `[-1, 1]` to absorb floating-point excursions
 * that would otherwise produce NaN on degenerate or near-degenerate triangles
 * (`lOpposite ≈ |lA - lB|` or `lOpposite ≈ lA + lB`).
 *
 * If either adjacent side is zero, the angle is undefined; we return 0 by
 * convention to keep callers branch-free.
 */
export function cornerAngleFromLengths(lOpposite: number, lA: number, lB: number): number {
  if (lA === 0 || lB === 0) return 0;
  const cosTheta = (lA * lA + lB * lB - lOpposite * lOpposite) / (2 * lA * lB);
  const clamped = cosTheta < -1 ? -1 : cosTheta > 1 ? 1 : cosTheta;
  return Math.acos(clamped);
}

/**
 * Convert barycentric coordinates `(u, v, w)` (in that order, weighting
 * `pA`, `pB`, `pC`) to a Cartesian point. No normalisation is performed —
 * caller is responsible for `u + v + w == 1` if that's required.
 */
export function barycentricToCartesian(b: Vec3, pA: Vec3, pB: Vec3, pC: Vec3): Vec3 {
  const u = b[0];
  const v = b[1];
  const w = b[2];
  return [
    u * pA[0] + v * pB[0] + w * pC[0],
    u * pA[1] + v * pB[1] + w * pC[1],
    u * pA[2] + v * pB[2] + w * pC[2],
  ];
}

/**
 * Barycentric coordinates of the projection of `p` onto the plane of triangle
 * `(pA, pB, pC)`. The standard area-ratio formulation:
 *
 *   total = (AB × AC) ⋅ n
 *   v = ((AP × AC) ⋅ n) / total
 *   w = ((AB × AP) ⋅ n) / total
 *   u = 1 - v - w
 *
 * where `n = normalize(AB × AC)` is the unit triangle normal. Returns
 * `[1, 0, 0]` for a degenerate (zero-area) triangle.
 */
export function cartesianToBarycentric(p: Vec3, pA: Vec3, pB: Vec3, pC: Vec3): Vec3 {
  const ab = sub(pB, pA);
  const ac = sub(pC, pA);
  const ap = sub(p, pA);
  const n = cross(ab, ac);
  const denom = dot(n, n);
  if (denom === 0) return [1, 0, 0];

  const v = dot(cross(ap, ac), n) / denom;
  const w = dot(cross(ab, ap), n) / denom;
  const u = 1 - v - w;
  return [u, v, w];
}
