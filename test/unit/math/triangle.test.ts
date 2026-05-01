import { describe, expect, it } from 'vitest';

import {
  barycentricToCartesian,
  cartesianToBarycentric,
  cornerAngleFromLengths,
  triangleArea,
  triangleAreaFromLengths,
} from '../../../src/math/triangle.js';
import { distance, type Vec3 } from '../../../src/math/vec3.js';

describe('triangle.triangleArea', () => {
  it('right triangle with legs 3 and 4 in xy-plane has area 6', () => {
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [3, 0, 0];
    const c: Vec3 = [0, 4, 0];
    expect(triangleArea(a, b, c)).toBeCloseTo(6, 12);
  });

  it('equilateral triangle of side s has area sqrt(3)/4 * s^2', () => {
    const s = 2.5;
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [s, 0, 0];
    const c: Vec3 = [s / 2, (s * Math.sqrt(3)) / 2, 0];
    const expected = (Math.sqrt(3) / 4) * s * s;
    expect(triangleArea(a, b, c)).toBeCloseTo(expected, 12);
  });

  it('degenerate (collinear) triangle has area 0', () => {
    expect(triangleArea([0, 0, 0], [1, 0, 0], [2, 0, 0])).toBe(0);
  });

  it('zero-side (two coincident vertices) triangle has area 0', () => {
    expect(triangleArea([1, 2, 3], [1, 2, 3], [4, 5, 6])).toBe(0);
  });

  it('area is invariant under vertex re-ordering', () => {
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [3, 0, 0];
    const c: Vec3 = [0, 4, 0];
    const A = triangleArea(a, b, c);
    expect(triangleArea(b, c, a)).toBeCloseTo(A, 12);
    expect(triangleArea(c, a, b)).toBeCloseTo(A, 12);
    expect(triangleArea(b, a, c)).toBeCloseTo(A, 12);
  });

  it('area is invariant under translation', () => {
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [3, 0, 0];
    const c: Vec3 = [0, 4, 0];
    const t: Vec3 = [10, -5, 7];
    const a2: Vec3 = [a[0] + t[0], a[1] + t[1], a[2] + t[2]];
    const b2: Vec3 = [b[0] + t[0], b[1] + t[1], b[2] + t[2]];
    const c2: Vec3 = [c[0] + t[0], c[1] + t[1], c[2] + t[2]];
    expect(triangleArea(a2, b2, c2)).toBeCloseTo(triangleArea(a, b, c), 12);
  });

  it('non-axis-aligned triangle in 3-space', () => {
    // Triangle with one vertex out of the xy-plane.
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [1, 0, 0];
    const c: Vec3 = [0, 0, 1];
    expect(triangleArea(a, b, c)).toBeCloseTo(0.5, 12);
  });
});

describe('triangle.triangleAreaFromLengths', () => {
  it('3-4-5 right triangle has area 6', () => {
    expect(triangleAreaFromLengths(3, 4, 5)).toBeCloseTo(6, 12);
  });

  it('equilateral triangle of side s has area sqrt(3)/4 * s^2', () => {
    const s = 2.5;
    expect(triangleAreaFromLengths(s, s, s)).toBeCloseTo((Math.sqrt(3) / 4) * s * s, 12);
  });

  it('degenerate triangle (one length == sum of other two) has area 0', () => {
    expect(triangleAreaFromLengths(2, 3, 5)).toBe(0);
  });

  it('zero-length triangle has area 0', () => {
    expect(triangleAreaFromLengths(0, 0, 0)).toBe(0);
  });

  it('agrees with cross-product area on a random non-degenerate triangle', () => {
    const a: Vec3 = [0.1, 0.2, 0.3];
    const b: Vec3 = [1.7, -0.4, 0.5];
    const c: Vec3 = [0.6, 1.1, -0.8];
    const lab = distance(a, b);
    const lbc = distance(b, c);
    const lca = distance(c, a);
    expect(triangleAreaFromLengths(lab, lbc, lca)).toBeCloseTo(triangleArea(a, b, c), 12);
  });

  it('agrees with cross-product area on a sliver triangle (relative error)', () => {
    // Very thin: heights are small relative to side length. This is the case
    // where naive Heron loses catastrophic digits but Kahan's formulation
    // remains accurate to within a small relative error. We compare on
    // *relative* error (1e-3 here) because the absolute area is tiny and
    // the lengths derived from the sliver geometry already accumulate FP
    // error before they reach Heron's formula.
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [1, 0, 0];
    const c: Vec3 = [0.5, 1e-7, 0];
    const lab = distance(a, b);
    const lbc = distance(b, c);
    const lca = distance(c, a);
    const expected = triangleArea(a, b, c);
    const got = triangleAreaFromLengths(lab, lbc, lca);
    expect(Math.abs(got - expected) / Math.abs(expected)).toBeLessThan(1e-3);
  });

  it('lengths argument order does not matter', () => {
    const A = triangleAreaFromLengths(3, 4, 5);
    expect(triangleAreaFromLengths(4, 3, 5)).toBeCloseTo(A, 12);
    expect(triangleAreaFromLengths(5, 4, 3)).toBeCloseTo(A, 12);
    expect(triangleAreaFromLengths(5, 3, 4)).toBeCloseTo(A, 12);
  });
});

describe('triangle.cornerAngleFromLengths', () => {
  it('right angle in a 3-4-5 triangle is pi/2', () => {
    // The right angle is opposite the hypotenuse (length 5), between the
    // legs of length 3 and 4.
    expect(cornerAngleFromLengths(5, 3, 4)).toBeCloseTo(Math.PI / 2, 12);
  });

  it('non-right angles in a 3-4-5 triangle', () => {
    // Angle opposite side 3 is between sides 4 and 5: arcsin(3/5).
    expect(cornerAngleFromLengths(3, 4, 5)).toBeCloseTo(Math.asin(3 / 5), 12);
    // Angle opposite side 4 is between sides 3 and 5: arcsin(4/5).
    expect(cornerAngleFromLengths(4, 3, 5)).toBeCloseTo(Math.asin(4 / 5), 12);
  });

  it('the three corners of any triangle sum to pi', () => {
    const lAB = 1.7;
    const lBC = 2.3;
    const lCA = 1.4;
    const angA = cornerAngleFromLengths(lBC, lAB, lCA);
    const angB = cornerAngleFromLengths(lCA, lAB, lBC);
    const angC = cornerAngleFromLengths(lAB, lBC, lCA);
    expect(angA + angB + angC).toBeCloseTo(Math.PI, 12);
  });

  it('equilateral triangle has all corners pi/3', () => {
    expect(cornerAngleFromLengths(1, 1, 1)).toBeCloseTo(Math.PI / 3, 12);
  });

  it('isosceles right triangle has angles pi/2, pi/4, pi/4', () => {
    // Legs of length 1, hypotenuse sqrt(2).
    const h = Math.sqrt(2);
    expect(cornerAngleFromLengths(h, 1, 1)).toBeCloseTo(Math.PI / 2, 12);
    expect(cornerAngleFromLengths(1, 1, h)).toBeCloseTo(Math.PI / 4, 12);
    expect(cornerAngleFromLengths(1, h, 1)).toBeCloseTo(Math.PI / 4, 12);
  });

  it('argument order in (lA, lB) does not matter', () => {
    expect(cornerAngleFromLengths(5, 3, 4)).toBeCloseTo(cornerAngleFromLengths(5, 4, 3), 12);
  });

  it('clamps when lOpposite ≈ lA + lB (degenerate flat)', () => {
    // Theoretical: angle = pi (the triangle has degenerated to a line).
    const eps = 1e-15;
    expect(cornerAngleFromLengths(2 + eps, 1, 1)).toBeCloseTo(Math.PI, 6);
  });

  it('clamps when lOpposite ≈ |lA - lB| (zero angle)', () => {
    const eps = 1e-15;
    // |3 - 4| = 1; the third side is just below this lower bound.
    expect(cornerAngleFromLengths(1 - eps, 3, 4)).toBeCloseTo(0, 6);
  });

  it('does not return NaN on slight FP overshoot', () => {
    // Construct an input that, without clamping, would push acos out of
    // [-1, 1] by a small amount.
    const v = cornerAngleFromLengths(2.0000000001, 1, 1);
    expect(Number.isNaN(v)).toBe(false);
    expect(v).toBeCloseTo(Math.PI, 6);
  });

  it('zero-length adjacent side returns 0 (no NaN)', () => {
    expect(cornerAngleFromLengths(1, 0, 1)).toBe(0);
    expect(cornerAngleFromLengths(1, 1, 0)).toBe(0);
  });
});

describe('triangle.barycentricToCartesian / cartesianToBarycentric', () => {
  const pA: Vec3 = [0, 0, 0];
  const pB: Vec3 = [4, 0, 0];
  const pC: Vec3 = [0, 3, 0];

  it('vertex A has barycentric (1, 0, 0)', () => {
    expect(barycentricToCartesian([1, 0, 0], pA, pB, pC)).toEqual(pA);
  });

  it('vertex B has barycentric (0, 1, 0)', () => {
    expect(barycentricToCartesian([0, 1, 0], pA, pB, pC)).toEqual(pB);
  });

  it('vertex C has barycentric (0, 0, 1)', () => {
    expect(barycentricToCartesian([0, 0, 1], pA, pB, pC)).toEqual(pC);
  });

  it('centroid is the average of the three vertices', () => {
    const cart = barycentricToCartesian([1 / 3, 1 / 3, 1 / 3], pA, pB, pC);
    const expected: Vec3 = [
      (pA[0] + pB[0] + pC[0]) / 3,
      (pA[1] + pB[1] + pC[1]) / 3,
      (pA[2] + pB[2] + pC[2]) / 3,
    ];
    expect(cart[0]).toBeCloseTo(expected[0], 12);
    expect(cart[1]).toBeCloseTo(expected[1], 12);
    expect(cart[2]).toBeCloseTo(expected[2], 12);
  });

  it('inverse: cartesian -> barycentric -> cartesian round-trips', () => {
    const points: Vec3[] = [
      [1, 1, 0],
      [0.5, 0.5, 0],
      [2, 0.5, 0],
      [0, 0, 0],
      [4, 0, 0],
      [0, 3, 0],
    ];
    for (const p of points) {
      const bary = cartesianToBarycentric(p, pA, pB, pC);
      const back = barycentricToCartesian(bary, pA, pB, pC);
      expect(back[0]).toBeCloseTo(p[0], 12);
      expect(back[1]).toBeCloseTo(p[1], 12);
      expect(back[2]).toBeCloseTo(p[2], 12);
    }
  });

  it('barycentric coordinates of a point in the triangle plane sum to 1', () => {
    const p: Vec3 = [1.2, 0.8, 0];
    const bary = cartesianToBarycentric(p, pA, pB, pC);
    expect(bary[0] + bary[1] + bary[2]).toBeCloseTo(1, 12);
  });

  it('barycentric coordinates of vertices are basis vectors', () => {
    const ba = cartesianToBarycentric(pA, pA, pB, pC);
    expect(ba[0]).toBeCloseTo(1, 12);
    expect(ba[1]).toBeCloseTo(0, 12);
    expect(ba[2]).toBeCloseTo(0, 12);

    const bb = cartesianToBarycentric(pB, pA, pB, pC);
    expect(bb[0]).toBeCloseTo(0, 12);
    expect(bb[1]).toBeCloseTo(1, 12);
    expect(bb[2]).toBeCloseTo(0, 12);

    const bc = cartesianToBarycentric(pC, pA, pB, pC);
    expect(bc[0]).toBeCloseTo(0, 12);
    expect(bc[1]).toBeCloseTo(0, 12);
    expect(bc[2]).toBeCloseTo(1, 12);
  });

  it('non-axis-aligned triangle in 3-space round-trips', () => {
    const qA: Vec3 = [1, 1, 1];
    const qB: Vec3 = [2, 0, -1];
    const qC: Vec3 = [-1, 2, 0.5];
    // Build a point as a known barycentric combination, then invert.
    const wantBary: Vec3 = [0.2, 0.3, 0.5];
    const cart = barycentricToCartesian(wantBary, qA, qB, qC);
    const gotBary = cartesianToBarycentric(cart, qA, qB, qC);
    expect(gotBary[0]).toBeCloseTo(wantBary[0], 12);
    expect(gotBary[1]).toBeCloseTo(wantBary[1], 12);
    expect(gotBary[2]).toBeCloseTo(wantBary[2], 12);
  });

  it('degenerate triangle returns (1, 0, 0) without NaN', () => {
    const dA: Vec3 = [0, 0, 0];
    const dB: Vec3 = [1, 0, 0];
    const dC: Vec3 = [2, 0, 0];
    const bary = cartesianToBarycentric([0.5, 0, 0], dA, dB, dC);
    expect(bary).toEqual([1, 0, 0]);
  });

  it('barycentricToCartesian does not require coords to sum to 1', () => {
    // Linearity check: scaling all barycentric weights scales the position
    // proportionally if A is the origin.
    const result = barycentricToCartesian([2, 0, 0], pA, pB, pC);
    expect(result).toEqual([0, 0, 0]);
  });
});
