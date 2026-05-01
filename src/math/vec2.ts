// Ported from geometry-central:
//   include/geometrycentral/utilities/vector2.h
//   src/utilities/vector2.cpp
//
// `Vec2` is an immutable readonly tuple. All operations return new tuples; no
// in-place mutation. The 2D type is used heavily by intrinsic-triangulation
// code (tangent planes, signpost angle bookkeeping), so the API mirrors
// geometry-central's `Vector2` rather closely.

/**
 * Two-dimensional vector, stored as an immutable `[x, y]` tuple.
 */
export type Vec2 = readonly [number, number];

/** Construct a `Vec2` from two scalar components. */
export function vec2(x: number, y: number): Vec2 {
  return [x, y];
}

/** Component-wise sum `a + b`. */
export function add(a: Vec2, b: Vec2): Vec2 {
  return [a[0] + b[0], a[1] + b[1]];
}

/** Component-wise difference `a - b`. */
export function sub(a: Vec2, b: Vec2): Vec2 {
  return [a[0] - b[0], a[1] - b[1]];
}

/** Scalar multiplication `s * a`. */
export function scale(a: Vec2, s: number): Vec2 {
  return [a[0] * s, a[1] * s];
}

/** Additive inverse `-a`. */
export function neg(a: Vec2): Vec2 {
  return [-a[0], -a[1]];
}

/** Standard 2D dot product. */
export function dot(a: Vec2, b: Vec2): number {
  return a[0] * b[0] + a[1] * b[1];
}

/**
 * Scalar 2D "cross product" — the z-component of the 3D cross of
 * `(a.x, a.y, 0)` and `(b.x, b.y, 0)`. Positive when `b` lies CCW from `a`.
 */
export function cross2(a: Vec2, b: Vec2): number {
  return a[0] * b[1] - a[1] * b[0];
}

/** Squared Euclidean length `|a|^2`. Avoids a sqrt when only ordering matters. */
export function norm2(a: Vec2): number {
  return a[0] * a[0] + a[1] * a[1];
}

/** Euclidean length `|a|`. */
export function norm(a: Vec2): number {
  return Math.sqrt(norm2(a));
}

/**
 * Unit-length vector parallel to `a`. The zero vector is returned unchanged
 * (matching geometry-central's behaviour, which produces NaNs but does not
 * throw — we choose to be slightly more forgiving).
 */
export function normalize(a: Vec2): Vec2 {
  const n = norm(a);
  if (n === 0) return [0, 0];
  return [a[0] / n, a[1] / n];
}

/** Euclidean distance `|a - b|`. */
export function distance(a: Vec2, b: Vec2): number {
  const dx = a[0] - b[0];
  const dy = a[1] - b[1];
  return Math.sqrt(dx * dx + dy * dy);
}

/**
 * Angle of `a` measured from the +x axis, in radians, in `(-pi, pi]`.
 * Returns `0` for the zero vector (matches `Math.atan2(0, 0)`).
 */
export function angle(a: Vec2): number {
  return Math.atan2(a[1], a[0]);
}

/**
 * Rotate `a` by `theta` radians (CCW for positive `theta` in a right-handed
 * 2D coordinate system).
 */
export function rotate(a: Vec2, theta: number): Vec2 {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [c * a[0] - s * a[1], s * a[0] + c * a[1]];
}

/**
 * Unit-length vector at angle `theta` from the +x axis (CCW). Mirrors
 * geometry-central's `Vector2::fromAngle(theta) = {cos(theta), sin(theta)}`,
 * used heavily by the intrinsic-triangulation layer for converting
 * signpost angles to tangent-plane vectors.
 */
export function fromAngle(theta: number): Vec2 {
  return [Math.cos(theta), Math.sin(theta)];
}

/**
 * Reduce `angle` to the canonical range `[0, modulus)` via
 * `((angle % modulus) + modulus) % modulus`. Differs from `Math.fmod` /
 * `std::fmod` in that the result is always non-negative — matching the
 * angle wrapping used by signpost bookkeeping (`standardizeAngle` in
 * geometry-central uses `std::fmod`, but for non-negative running sums
 * the two agree; we use the symmetric form to be defensive).
 */
export function modPositive(angle: number, modulus: number): number {
  if (modulus <= 0) return angle;
  const m = angle % modulus;
  return m < 0 ? m + modulus : m;
}
