import { describe, expect, it } from 'vitest';

import { orient2d } from '../../../src/math/predicates.js';

describe('predicates.orient2d', () => {
  it('CCW triangle returns positive', () => {
    expect(orient2d([0, 0], [1, 0], [0, 1])).toBeGreaterThan(0);
  });

  it('CW triangle returns negative', () => {
    expect(orient2d([0, 0], [0, 1], [1, 0])).toBeLessThan(0);
  });

  it('collinear points return exactly 0 (integer coords)', () => {
    expect(orient2d([0, 0], [1, 1], [2, 2])).toBe(0);
    expect(orient2d([0, 0], [1, 0], [5, 0])).toBe(0);
    expect(orient2d([0, 0], [-1, -1], [3, 3])).toBe(0);
  });

  it('twice the signed area of a unit triangle is +1', () => {
    expect(orient2d([0, 0], [1, 0], [0, 1])).toBe(1);
  });

  it('twice the signed area of a 3-4 right triangle is 12', () => {
    expect(orient2d([0, 0], [3, 0], [0, 4])).toBe(12);
  });

  it('sign flips when two points are swapped', () => {
    const a: [number, number] = [0, 0];
    const b: [number, number] = [3, 1];
    const c: [number, number] = [1, 4];
    expect(orient2d(a, b, c)).toBe(-orient2d(a, c, b));
    expect(orient2d(a, b, c)).toBe(-orient2d(b, a, c));
  });

  it('cyclic permutation preserves the value', () => {
    const a: [number, number] = [0, 0];
    const b: [number, number] = [3, 1];
    const c: [number, number] = [1, 4];
    const o = orient2d(a, b, c);
    expect(orient2d(b, c, a)).toBe(o);
    expect(orient2d(c, a, b)).toBe(o);
  });

  it('translation invariance', () => {
    const a: [number, number] = [0, 0];
    const b: [number, number] = [3, 1];
    const c: [number, number] = [1, 4];
    const o = orient2d(a, b, c);

    const t: [number, number] = [100, -50];
    const at: [number, number] = [a[0] + t[0], a[1] + t[1]];
    const bt: [number, number] = [b[0] + t[0], b[1] + t[1]];
    const ct: [number, number] = [c[0] + t[0], c[1] + t[1]];
    expect(orient2d(at, bt, ct)).toBe(o);
  });

  it('large integer collinear points still produce exactly 0', () => {
    // Within Float64 exact-integer range (< 2^53), but well below the
    // 2^26-scale subtraction-product threshold the doc-comment mentions.
    expect(orient2d([0, 0], [1000000, 1000000], [2000000, 2000000])).toBe(0);
  });

  it('detects strict left/right of a directed edge', () => {
    // Edge from (0,0) -> (1,0); positive y is "left", negative y is "right".
    expect(orient2d([0, 0], [1, 0], [0.5, 0.001])).toBeGreaterThan(0);
    expect(orient2d([0, 0], [1, 0], [0.5, -0.001])).toBeLessThan(0);
  });

  it('matches the manual determinant formula', () => {
    // Random sample.
    const a: [number, number] = [1.2, 3.4];
    const b: [number, number] = [-0.5, 2.7];
    const c: [number, number] = [4.0, -1.1];
    const expected = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    expect(orient2d(a, b, c)).toBe(expected);
  });

  it('degenerate (two coincident vertices) returns 0', () => {
    expect(orient2d([1, 2], [1, 2], [3, 4])).toBe(0);
    expect(orient2d([1, 2], [3, 4], [1, 2])).toBe(0);
    expect(orient2d([3, 4], [1, 2], [1, 2])).toBe(0);
  });
});
