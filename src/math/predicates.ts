// Ported from geometry-central:
//   include/geometrycentral/utilities/utilities.h  (orient2d-style helpers)
//
// Geometric predicates. FlipOut does not need Shewchuk's adaptive exact
// predicates — every flip decision in the algorithm operates on intrinsic
// edge lengths via the cosine rule, never on raw 2D coordinates of vertices
// in a global frame. We provide only the simple non-robust determinant.

import type { Vec2 } from './vec2.js';

/**
 * Sign-bearing twice-the-signed-area of triangle `(a, b, c)`.
 *
 *   orient2d(a, b, c) = (b - a) × (c - a)
 *                     = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
 *
 * Conventions (right-handed 2D coordinate system):
 *   > 0  ⇒ `c` lies to the left of directed edge `a → b`  (CCW)
 *   < 0  ⇒ `c` lies to the right                          (CW)
 *   = 0  ⇒ all three points are collinear
 *
 * The raw determinant is returned (not just the sign) so callers can use it
 * as a magnitude where useful. Note this is *not* a robust predicate; for
 * collinear points stored as integers, JavaScript's `number` is IEEE-754
 * Float64 and the result is exactly 0 provided each operand fits in 2^26.
 */
export function orient2d(a: Vec2, b: Vec2, c: Vec2): number {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}
