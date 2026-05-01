import { describe, expect, it } from 'vitest';

import {
  add,
  cross,
  distance,
  dot,
  neg,
  norm,
  norm2,
  normalize,
  scale,
  sub,
  vec3,
} from '../../../src/math/vec3.js';

describe('vec3.vec3 / add / sub / scale / neg', () => {
  it('vec3 packs components into a tuple', () => {
    expect(vec3(1, 2, 3)).toEqual([1, 2, 3]);
  });

  it('add of zero returns the original', () => {
    expect(add([1, 2, 3], [0, 0, 0])).toEqual([1, 2, 3]);
  });

  it('add is component-wise', () => {
    expect(add([1, 2, 3], [4, 5, 6])).toEqual([5, 7, 9]);
  });

  it('add is commutative', () => {
    const a: [number, number, number] = [1.1, 2.2, 3.3];
    const b: [number, number, number] = [4.4, 5.5, 6.6];
    expect(add(a, b)).toEqual(add(b, a));
  });

  it('sub of self is zero', () => {
    expect(sub([7, -3, 2], [7, -3, 2])).toEqual([0, 0, 0]);
  });

  it('sub is component-wise', () => {
    expect(sub([5, 7, 9], [1, 2, 3])).toEqual([4, 5, 6]);
  });

  it('scale by 0 yields zero', () => {
    expect(scale([1, 2, 3], 0)).toEqual([0, 0, 0]);
  });

  it('scale by 1 is identity', () => {
    expect(scale([1.5, -2.5, 3.5], 1)).toEqual([1.5, -2.5, 3.5]);
  });

  it('scale by -1 equals neg', () => {
    expect(scale([1, 2, 3], -1)).toEqual(neg([1, 2, 3]));
  });

  it('neg flips all signs', () => {
    expect(neg([1, -2, 3])).toEqual([-1, 2, -3]);
  });
});

describe('vec3.dot / cross', () => {
  it('dot of perpendicular axes is 0', () => {
    expect(dot([1, 0, 0], [0, 1, 0])).toBe(0);
    expect(dot([1, 0, 0], [0, 0, 1])).toBe(0);
    expect(dot([0, 1, 0], [0, 0, 1])).toBe(0);
  });

  it('dot of parallel axes is the product of magnitudes', () => {
    expect(dot([2, 0, 0], [3, 0, 0])).toBe(6);
  });

  it('dot is symmetric', () => {
    const a: [number, number, number] = [1.5, -2.7, 0.3];
    const b: [number, number, number] = [3.0, 4.1, -1.2];
    expect(dot(a, b)).toBe(dot(b, a));
  });

  it('dot(a, a) === norm2(a)', () => {
    const a: [number, number, number] = [3, 4, 12];
    expect(dot(a, a)).toBe(norm2(a));
  });

  it('cross of +x and +y is +z (right-handed)', () => {
    expect(cross([1, 0, 0], [0, 1, 0])).toEqual([0, 0, 1]);
  });

  it('cross of +y and +z is +x', () => {
    expect(cross([0, 1, 0], [0, 0, 1])).toEqual([1, 0, 0]);
  });

  it('cross of +z and +x is +y', () => {
    expect(cross([0, 0, 1], [1, 0, 0])).toEqual([0, 1, 0]);
  });

  it('cross is anti-symmetric: cross(a, b) === -cross(b, a)', () => {
    const a: [number, number, number] = [1.5, -2.7, 0.3];
    const b: [number, number, number] = [3.0, 4.1, -1.2];
    const ab = cross(a, b);
    const ba = cross(b, a);
    expect(ab[0]).toBeCloseTo(-ba[0], 12);
    expect(ab[1]).toBeCloseTo(-ba[1], 12);
    expect(ab[2]).toBeCloseTo(-ba[2], 12);
  });

  it('cross(a, a) === 0', () => {
    expect(cross([1, 2, 3], [1, 2, 3])).toEqual([0, 0, 0]);
  });

  it('cross(a, b) is orthogonal to both a and b', () => {
    const a: [number, number, number] = [1.5, -2.7, 0.3];
    const b: [number, number, number] = [3.0, 4.1, -1.2];
    const c = cross(a, b);
    expect(dot(a, c)).toBeCloseTo(0, 10);
    expect(dot(b, c)).toBeCloseTo(0, 10);
  });
});

describe('vec3.norm / norm2 / normalize / distance', () => {
  it('norm of zero is 0', () => {
    expect(norm([0, 0, 0])).toBe(0);
  });

  it('norm of (3, 4, 0) is 5', () => {
    expect(norm([3, 4, 0])).toBe(5);
  });

  it('norm of (1, 2, 2) is 3', () => {
    expect(norm([1, 2, 2])).toBe(3);
  });

  it('norm of (3, 4, 12) is 13', () => {
    expect(norm([3, 4, 12])).toBe(13);
  });

  it('norm2 == norm**2', () => {
    const v: [number, number, number] = [1.5, -2.7, 0.3];
    expect(norm2(v)).toBeCloseTo(norm(v) ** 2, 12);
  });

  it('normalize of unit axis is itself', () => {
    expect(normalize([1, 0, 0])).toEqual([1, 0, 0]);
    expect(normalize([0, 1, 0])).toEqual([0, 1, 0]);
    expect(normalize([0, 0, 1])).toEqual([0, 0, 1]);
  });

  it('normalize gives unit length', () => {
    const v = normalize([3, 4, 12]);
    expect(norm(v)).toBeCloseTo(1, 12);
  });

  it('normalize of zero returns zero (no NaN)', () => {
    expect(normalize([0, 0, 0])).toEqual([0, 0, 0]);
  });

  it('distance is symmetric', () => {
    expect(distance([1, 2, 3], [4, 6, 15])).toBe(distance([4, 6, 15], [1, 2, 3]));
  });

  it('distance of self is 0', () => {
    expect(distance([1.5, -2.7, 0.3], [1.5, -2.7, 0.3])).toBe(0);
  });

  it('distance ((0,0,0), (3,4,12)) is 13', () => {
    expect(distance([0, 0, 0], [3, 4, 12])).toBe(13);
  });
});
