// Ported from geometry-central:
//   include/geometrycentral/utilities/vector3.h
//   src/utilities/vector3.cpp
//
// `Vec3` is an immutable readonly tuple. All operations return new tuples; no
// in-place mutation. The function set mirrors geometry-central's `Vector3`,
// trimmed to what FlipOut and the intrinsic-triangulation layer actually need.

/**
 * Three-dimensional vector, stored as an immutable `[x, y, z]` tuple.
 */
export type Vec3 = readonly [number, number, number];

/** Construct a `Vec3` from three scalar components. */
export function vec3(x: number, y: number, z: number): Vec3 {
  return [x, y, z];
}

/** Component-wise sum `a + b`. */
export function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

/** Component-wise difference `a - b`. */
export function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

/** Scalar multiplication `s * a`. */
export function scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

/** Additive inverse `-a`. */
export function neg(a: Vec3): Vec3 {
  return [-a[0], -a[1], -a[2]];
}

/** Standard Euclidean dot product. */
export function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/** Right-handed cross product `a × b`. */
export function cross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

/** Squared Euclidean length `|a|^2`. */
export function norm2(a: Vec3): number {
  return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}

/** Euclidean length `|a|`. */
export function norm(a: Vec3): number {
  return Math.sqrt(norm2(a));
}

/**
 * Unit-length vector parallel to `a`. The zero vector is returned unchanged
 * (geometry-central yields NaNs; we prefer to be defensive at this layer).
 */
export function normalize(a: Vec3): Vec3 {
  const n = norm(a);
  if (n === 0) return [0, 0, 0];
  return [a[0] / n, a[1] / n, a[2] / n];
}

/** Euclidean distance `|a - b|`. */
export function distance(a: Vec3, b: Vec3): number {
  const dx = a[0] - b[0];
  const dy = a[1] - b[1];
  const dz = a[2] - b[2];
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}
