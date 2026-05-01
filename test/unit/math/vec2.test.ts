import { describe, expect, it } from 'vitest';

import {
  add,
  angle,
  cross2,
  distance,
  dot,
  neg,
  norm,
  norm2,
  normalize,
  rotate,
  scale,
  sub,
  vec2,
} from '../../../src/math/vec2.js';

const TAU = Math.PI * 2;

describe('vec2.vec2 / add / sub / scale / neg', () => {
  it('vec2 packs components into a tuple', () => {
    expect(vec2(1, 2)).toEqual([1, 2]);
  });

  it('add of zero returns the original', () => {
    expect(add([1, 2], [0, 0])).toEqual([1, 2]);
  });

  it('add is component-wise', () => {
    expect(add([1, 2], [3, 4])).toEqual([4, 6]);
  });

  it('sub of self is the zero vector', () => {
    expect(sub([7, -3], [7, -3])).toEqual([0, 0]);
  });

  it('sub is component-wise', () => {
    expect(sub([4, 6], [1, 2])).toEqual([3, 4]);
  });

  it('scale multiplies both components', () => {
    expect(scale([2, -3], 4)).toEqual([8, -12]);
  });

  it('scale by 0 returns the zero vector', () => {
    expect(scale([5, 5], 0)).toEqual([0, 0]);
  });

  it('neg flips both signs', () => {
    expect(neg([1, -2])).toEqual([-1, 2]);
  });

  it('neg of zero is zero', () => {
    expect(neg([0, 0])).toEqual([-0, -0]);
  });
});

describe('vec2.dot / cross2', () => {
  it('dot of perpendicular axis vectors is 0', () => {
    expect(dot([1, 0], [0, 1])).toBe(0);
  });

  it('dot of parallel vectors is the product of magnitudes', () => {
    expect(dot([2, 0], [3, 0])).toBe(6);
  });

  it('dot is symmetric', () => {
    const a: [number, number] = [1.5, -2.7];
    const b: [number, number] = [3.0, 4.1];
    expect(dot(a, b)).toBe(dot(b, a));
  });

  it('dot(a, a) === norm2(a)', () => {
    const a: [number, number] = [3, 4];
    expect(dot(a, a)).toBe(norm2(a));
  });

  it('cross2 of +x and +y is +1 (CCW)', () => {
    expect(cross2([1, 0], [0, 1])).toBe(1);
  });

  it('cross2 of +y and +x is -1 (CW)', () => {
    expect(cross2([0, 1], [1, 0])).toBe(-1);
  });

  it('cross2 is anti-symmetric', () => {
    const a: [number, number] = [1.5, -2.7];
    const b: [number, number] = [3.0, 4.1];
    expect(cross2(a, b)).toBe(-cross2(b, a));
  });

  it('cross2(a, a) === 0', () => {
    expect(cross2([3, 4], [3, 4])).toBe(0);
  });
});

describe('vec2.norm / norm2 / normalize / distance', () => {
  it('norm of zero is 0', () => {
    expect(norm([0, 0])).toBe(0);
  });

  it('norm of (3, 4) is 5', () => {
    expect(norm([3, 4])).toBe(5);
  });

  it('norm2 is the square of norm', () => {
    const v: [number, number] = [3, 4];
    expect(norm2(v)).toBeCloseTo(norm(v) ** 2, 12);
  });

  it('normalize of an axis vector is itself', () => {
    expect(normalize([1, 0])).toEqual([1, 0]);
    expect(normalize([0, 1])).toEqual([0, 1]);
  });

  it('normalize gives a unit-length result', () => {
    const v = normalize([3, 4]);
    expect(norm(v)).toBeCloseTo(1, 12);
  });

  it('normalize of zero returns zero (no NaN)', () => {
    expect(normalize([0, 0])).toEqual([0, 0]);
  });

  it('distance is symmetric', () => {
    expect(distance([1, 2], [4, 6])).toBe(distance([4, 6], [1, 2]));
  });

  it('distance of self is 0', () => {
    expect(distance([1.5, -2.7], [1.5, -2.7])).toBe(0);
  });

  it('distance reduces to the (3-4-5) Pythagorean case', () => {
    expect(distance([0, 0], [3, 4])).toBe(5);
  });
});

describe('vec2.angle', () => {
  it('+x axis has angle 0', () => {
    expect(angle([1, 0])).toBe(0);
  });

  it('+y axis has angle pi/2', () => {
    expect(angle([0, 1])).toBeCloseTo(Math.PI / 2, 12);
  });

  it('-x axis has angle pi', () => {
    expect(angle([-1, 0])).toBeCloseTo(Math.PI, 12);
  });

  it('-y axis has angle -pi/2', () => {
    expect(angle([0, -1])).toBeCloseTo(-Math.PI / 2, 12);
  });

  it('zero vector returns 0 (atan2(0, 0))', () => {
    expect(angle([0, 0])).toBe(0);
  });

  it('matches atan2 for arbitrary inputs', () => {
    const v: [number, number] = [-2, 3];
    expect(angle(v)).toBe(Math.atan2(v[1], v[0]));
  });
});

describe('vec2.rotate', () => {
  it('rotate by 0 is identity', () => {
    const v: [number, number] = [1.5, -2.7];
    const r = rotate(v, 0);
    expect(r[0]).toBeCloseTo(v[0], 12);
    expect(r[1]).toBeCloseTo(v[1], 12);
  });

  it('rotate +x by pi/2 gives +y', () => {
    const r = rotate([1, 0], Math.PI / 2);
    expect(r[0]).toBeCloseTo(0, 12);
    expect(r[1]).toBeCloseTo(1, 12);
  });

  it('rotate +y by pi/2 gives -x', () => {
    const r = rotate([0, 1], Math.PI / 2);
    expect(r[0]).toBeCloseTo(-1, 12);
    expect(r[1]).toBeCloseTo(0, 12);
  });

  it('rotate is additive: rotate(rotate(v, a), b) = rotate(v, a+b)', () => {
    const v: [number, number] = [1.234, -5.678];
    const a = 0.7;
    const b = 1.3;
    const r1 = rotate(rotate(v, a), b);
    const r2 = rotate(v, a + b);
    expect(r1[0]).toBeCloseTo(r2[0], 12);
    expect(r1[1]).toBeCloseTo(r2[1], 12);
  });

  it('rotate by 2π returns the original (within FP)', () => {
    const v: [number, number] = [1.234, -5.678];
    const r = rotate(v, TAU);
    expect(r[0]).toBeCloseTo(v[0], 12);
    expect(r[1]).toBeCloseTo(v[1], 12);
  });

  it('rotate preserves length', () => {
    const v: [number, number] = [1.234, -5.678];
    expect(norm(rotate(v, 0.913))).toBeCloseTo(norm(v), 12);
  });

  it('rotate(angle(v)) places v on the +x axis at distance |v|', () => {
    const v: [number, number] = [3, 4];
    const r = rotate(v, -angle(v));
    expect(r[0]).toBeCloseTo(norm(v), 12);
    expect(r[1]).toBeCloseTo(0, 12);
  });
});
