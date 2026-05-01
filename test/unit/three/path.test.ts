/**
 * Path → Three.js adapters: `pathToVector3Array` and `pathToBufferGeometry`.
 *
 * The latter is sized for `THREE.Line` semantics (`N` points → `N - 1`
 * segments), *not* `THREE.LineSegments`. We don't actually instantiate a
 * renderer here; we only verify the geometry's shape and contents.
 */

import * as THREE from 'three';
import { describe, expect, it } from 'vitest';

import type { Vec3 } from '../../../src/math/vec3.js';
import { pathToBufferGeometry, pathToVector3Array } from '../../../src/three/index.js';

describe('pathToVector3Array', () => {
  it('maps 3 input points to 3 THREE.Vector3 instances', () => {
    const path: Vec3[] = [
      [0, 0, 0],
      [1, 2, 3],
      [-1, -2, -3],
    ];
    const out = pathToVector3Array(path);
    expect(out.length).toBe(3);
    expect(out[0]).toBeInstanceOf(THREE.Vector3);
    expect(out[0]!.x).toBeCloseTo(0);
    expect(out[1]!.x).toBeCloseTo(1);
    expect(out[1]!.y).toBeCloseTo(2);
    expect(out[1]!.z).toBeCloseTo(3);
    expect(out[2]!.x).toBeCloseTo(-1);
  });

  it('returns an empty array for an empty path', () => {
    const out = pathToVector3Array([]);
    expect(out.length).toBe(0);
  });

  it('preserves order', () => {
    const path: Vec3[] = [];
    for (let i = 0; i < 50; i++) path.push([i, i * 2, i * 3]);
    const out = pathToVector3Array(path);
    expect(out.length).toBe(50);
    for (let i = 0; i < 50; i++) {
      expect(out[i]!.x).toBe(i);
      expect(out[i]!.y).toBe(i * 2);
      expect(out[i]!.z).toBe(i * 3);
    }
  });

  it('each result is an independent THREE.Vector3 (no shared storage)', () => {
    const path: Vec3[] = [
      [1, 2, 3],
      [4, 5, 6],
    ];
    const out = pathToVector3Array(path);
    out[0]!.x = 999;
    expect(out[1]!.x).toBe(4);
  });
});

describe('pathToBufferGeometry', () => {
  it('5-point path: position attribute count is 5, itemSize is 3, no index', () => {
    const path: Vec3[] = [
      [0, 0, 0],
      [1, 0, 0],
      [1, 1, 0],
      [0, 1, 0],
      [0, 0, 0],
    ];
    const g = pathToBufferGeometry(path);
    const pos = g.getAttribute('position');
    expect(pos.count).toBe(5);
    expect(pos.itemSize).toBe(3);
    expect(g.getIndex()).toBeNull();
  });

  it('matches THREE.Line semantics: N points produce N-1 segments', () => {
    // Don't render — just assert the geometry shape would yield N-1 line
    // segments under THREE.Line: one drawcall iterating consecutive pairs.
    const path: Vec3[] = [
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
      [3, 0, 0],
    ];
    const g = pathToBufferGeometry(path);
    expect(g.getAttribute('position').count).toBe(4);
    // For THREE.LineSegments we'd need to emit pairs (0,1,1,2,2,3) → 6 verts.
    // We don't.
    expect(g.getAttribute('position').count).not.toBe((path.length - 1) * 2);
  });

  it('position values match the input, x/y/z preserved', () => {
    const path: Vec3[] = [
      [0.1, 0.2, 0.3],
      [-1, -2, -3],
      [42, 43, 44],
    ];
    const g = pathToBufferGeometry(path);
    const pos = g.getAttribute('position');
    for (let i = 0; i < path.length; i++) {
      const p = path[i]!;
      expect(pos.getX(i)).toBeCloseTo(p[0], 5);
      expect(pos.getY(i)).toBeCloseTo(p[1], 5);
      expect(pos.getZ(i)).toBeCloseTo(p[2], 5);
    }
  });

  it('empty path produces an empty geometry', () => {
    const g = pathToBufferGeometry([]);
    expect(g.getAttribute('position').count).toBe(0);
  });

  it('single-point path produces a 1-vertex geometry (no segments)', () => {
    const g = pathToBufferGeometry([[1, 2, 3]]);
    expect(g.getAttribute('position').count).toBe(1);
  });

  it('underlying array is Float32 (consistent with Three.js conventions)', () => {
    const g = pathToBufferGeometry([
      [0, 0, 0],
      [1, 0, 0],
    ]);
    const arr = g.getAttribute('position').array;
    expect(arr).toBeInstanceOf(Float32Array);
  });

  it('returned geometry can be wrapped in a THREE.Line without throwing', () => {
    // Sanity: assemble the standard Line scene-graph node and check the
    // Line's geometry references our attribute.
    const path: Vec3[] = [
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ];
    const g = pathToBufferGeometry(path);
    const line = new THREE.Line(g, new THREE.LineBasicMaterial());
    expect(line.geometry).toBe(g);
    expect(line.geometry.getAttribute('position').count).toBe(3);
  });
});
