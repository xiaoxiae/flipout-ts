/**
 * `meshFromBufferGeometry` — both indexed and non-indexed input paths,
 * plus the welding behaviour and error contract.
 */

import * as THREE from 'three';
import { describe, expect, it } from 'vitest';

import type { Vec3 } from '../../../src/math/vec3.js';
import { meshFromBufferGeometry } from '../../../src/three/index.js';
import { cube, tetrahedron } from '../../_helpers/meshes.js';
import { loadFixture } from '../../_helpers/load-fixture.js';

// ---------------------------------------------------------------------------
// Indexed geometries — passed through, no welding.
// ---------------------------------------------------------------------------

describe('meshFromBufferGeometry — indexed BufferGeometry', () => {
  it('THREE.BoxGeometry: vertex count matches position attribute count (24)', () => {
    const g = new THREE.BoxGeometry();
    const { mesh, positions } = meshFromBufferGeometry(g);
    expect(mesh.nVertices).toBe(g.getAttribute('position').count);
    expect(mesh.nVertices).toBe(24);
    expect(positions.length).toBe(24);
  });

  it('THREE.BoxGeometry: face count matches index.count / 3 (12)', () => {
    const g = new THREE.BoxGeometry();
    const { mesh } = meshFromBufferGeometry(g);
    const idx = g.getIndex();
    expect(idx).not.toBeNull();
    expect(mesh.nFaces).toBe(idx!.count / 3);
    expect(mesh.nFaces).toBe(12);
  });

  it('positions array is a parallel Vec3[] of the position attribute', () => {
    const g = new THREE.BoxGeometry();
    const { positions } = meshFromBufferGeometry(g);
    const pa = g.getAttribute('position');
    for (let i = 0; i < pa.count; i++) {
      expect(positions[i]).toEqual([pa.getX(i), pa.getY(i), pa.getZ(i)]);
    }
  });

  it('THREE.TetrahedronGeometry (non-indexed) welds to 4 vertices, 4 faces', () => {
    // TetrahedronGeometry is non-indexed — corners are duplicated per face for
    // normals/uv. Welding by position should collapse to the 4 corners.
    const g = new THREE.TetrahedronGeometry();
    expect(g.getIndex()).toBeNull();
    const { mesh } = meshFromBufferGeometry(g);
    expect(mesh.nFaces).toBe(4);
    expect(mesh.nVertices).toBe(4);
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('hand-built indexed cube from the L1 helper has matching vertex/face counts', () => {
    const m = cube();
    const flat = new Float32Array(m.vertices.length * 3);
    for (let i = 0; i < m.vertices.length; i++) {
      const v = m.vertices[i]!;
      flat[i * 3 + 0] = v[0];
      flat[i * 3 + 1] = v[1];
      flat[i * 3 + 2] = v[2];
    }
    const indices = new Uint32Array(m.faces.length * 3);
    for (let i = 0; i < m.faces.length; i++) {
      const f = m.faces[i]!;
      indices[i * 3 + 0] = f[0];
      indices[i * 3 + 1] = f[1];
      indices[i * 3 + 2] = f[2];
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(flat, 3));
    g.setIndex(new THREE.BufferAttribute(indices, 1));

    const { mesh, positions } = meshFromBufferGeometry(g);
    expect(mesh.nVertices).toBe(8);
    expect(mesh.nFaces).toBe(12);
    expect(mesh.nEdges).toBe(18);
    expect(mesh.eulerCharacteristic).toBe(2);
    expect(positions.length).toBe(8);
  });

  it('round-tripped mesh supports flipEdge (half-edge structure is sound)', () => {
    const m = cube();
    const flat = new Float32Array(m.vertices.flat());
    const indices = new Uint32Array(m.faces.flat());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(flat, 3));
    g.setIndex(new THREE.BufferAttribute(indices, 1));

    const { mesh } = meshFromBufferGeometry(g);
    // Find a flippable edge and flip it; the mesh should accept the flip.
    let flipped = false;
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.flipEdge(e)) {
        flipped = true;
        break;
      }
    }
    expect(flipped).toBe(true);
    // Counts unchanged.
    expect(mesh.nVertices).toBe(8);
    expect(mesh.nFaces).toBe(12);
    expect(mesh.nEdges).toBe(18);
  });

  it('hand-built indexed tetrahedron round-trips with χ=2', () => {
    const m = tetrahedron();
    const flat = new Float32Array(m.vertices.flat());
    const indices = new Uint32Array(m.faces.flat());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(flat, 3));
    g.setIndex(new THREE.BufferAttribute(indices, 1));

    const { mesh, positions } = meshFromBufferGeometry(g);
    expect(mesh.nVertices).toBe(4);
    expect(mesh.nFaces).toBe(4);
    expect(mesh.eulerCharacteristic).toBe(2);
    expect(positions[0]).toEqual([1, 1, 1]);
  });
});

// ---------------------------------------------------------------------------
// Non-indexed welding.
// ---------------------------------------------------------------------------

describe('meshFromBufferGeometry — non-indexed welding', () => {
  it('non-indexed BoxGeometry welds to 8 corner vertices', () => {
    const g = new THREE.BoxGeometry().toNonIndexed();
    const { mesh, positions } = meshFromBufferGeometry(g);
    // BoxGeometry indexed has 24 vertices, but 8 unique positions; after
    // welding the non-indexed (36-vertex) form, the underlying corners
    // collapse to 8.
    expect(mesh.nVertices).toBe(8);
    expect(positions.length).toBe(8);
    expect(mesh.nFaces).toBe(12);
    expect(mesh.eulerCharacteristic).toBe(2);
  });

  it('welding tolerance: vertices closer than eps merge', () => {
    const g = new THREE.BufferGeometry();
    // Two triangles sharing the edge (0,0,0)-(1,0,0) with consistent
    // (manifold) winding: tri1 = [a, b, c] CCW, tri2 = [b, a, d] so the
    // shared edge runs (a→b) in tri1 and (b→a) in tri2. Tri2's first
    // vertex is offset by 1e-10 from `b` so welding must merge it.
    const positions = new Float32Array([
      // tri 1: a=(0,0,0), b=(1,0,0), c=(0,1,0)
      0, 0, 0, 1, 0, 0, 0, 1, 0,
      // tri 2: b'=(1+ε,0,0), a'=(0,0,0), d=(1,1,0) — shared edge b→a
      1 + 1e-10, 0, 0, 0, 0, 0, 1, 1, 0,
    ]);
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const { mesh } = meshFromBufferGeometry(g, { weldEpsilon: 1e-6 });
    // Expect 4 unique vertices: a, b, c, d.
    expect(mesh.nVertices).toBe(4);
    expect(mesh.nFaces).toBe(2);
  });

  it('welding tolerance: with tighter eps, near-duplicates stay separate', () => {
    // Two fully-disjoint triangles whose only "shared" content is a near-
    // duplicate vertex pair: tri1 has a vertex at (1, 0, 0); tri2 has a
    // vertex at (1+1e-3, 0, 0). With eps=1e-6 they stay separate, leaving
    // 6 unique positions across 2 disjoint triangles.
    const g = new THREE.BufferGeometry();
    const positions = new Float32Array([
      0, 0, 0, 1, 0, 0, 0, 1, 0,
      1 + 1e-3, 0, 5, 2, 0, 5, 1 + 1e-3, 1, 5,
    ]);
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const { mesh } = meshFromBufferGeometry(g, { weldEpsilon: 1e-6 });
    // 6 unique positions, 2 disjoint triangles.
    expect(mesh.nVertices).toBe(6);
    expect(mesh.nFaces).toBe(2);
  });

  it('a single triangle with all 3 vertices identical is dropped (degenerate face)', () => {
    const g = new THREE.BufferGeometry();
    const positions = new Float32Array([0, 0, 0, 0, 0, 0, 0, 0, 0]);
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const { mesh, positions: pos } = meshFromBufferGeometry(g);
    expect(pos.length).toBe(1);
    // Degenerate triangle is silently dropped (otherwise SurfaceMesh.fromFaces
    // would reject the self-edge).
    expect(mesh.nFaces).toBe(0);
    expect(mesh.nVertices).toBe(1);
  });

  it('default eps scales with bbox diagonal: large-coordinate mesh welds correctly', () => {
    // Two triangles meeting along a shared edge, then we shift one corner
    // of the *second* triangle by a sub-meter amount. With absolute eps
    // (1e-7) the corners would stay separate; with the bbox-scaled
    // default (1e-7 × 1e6 ≈ 0.1) they merge.
    //
    // tri 1: a=(0,0,0), b=(1e6,0,0), c=(0,1e6,0)  CCW
    // tri 2: b=(1e6,0,0), a'=(0,0,0)+jitter, d=(1e6,1e6,0)  with shared
    // edge (b → a) opposite to (a → b) in tri 1.
    const g = new THREE.BufferGeometry();
    const positions = new Float32Array([
      0, 0, 0, 1e6, 0, 0, 0, 1e6, 0,
      1e6, 0, 0, 1e-3, 1e-3, 1e-3, 1e6, 1e6, 0,
    ]);
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const { mesh } = meshFromBufferGeometry(g);
    // Bbox diag ≈ sqrt(2) × 1e6; default eps ≈ 0.14. 1e-3 jitter folds in.
    expect(mesh.nVertices).toBe(4);
    expect(mesh.nFaces).toBe(2);
  });

  it('default eps falls back to absolute floor when bbox is degenerate', () => {
    // Three identical vertices → bbox diag is 0 → eps falls back to 1e-12.
    // Two vertices that differ by 1e-9 should stay separate under that floor.
    const g = new THREE.BufferGeometry();
    const positions = new Float32Array([
      // tri 1: collapsed
      0, 0, 0, 0, 0, 0, 0, 0, 0,
      // tri 2: includes a vertex at 1e-9, also collapsed but separate from origin under eps=1e-12
      1e-9, 0, 0, 1e-9, 0, 0, 1e-9, 0, 0,
    ]);
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const { mesh } = meshFromBufferGeometry(g);
    expect(mesh.nVertices).toBe(2);
    expect(mesh.nFaces).toBe(0);
  });

  it('user-provided weldEpsilon overrides the automatic default', () => {
    const g = new THREE.BufferGeometry();
    const positions = new Float32Array([
      0, 0, 0, 1, 0, 0, 0, 1, 0, 0.5, 0, 0, 1, 0, 0, 1, 1, 0,
    ]);
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    // With eps = 1.0 every vertex falls inside the same neighbourhood as
    // (0,0,0); after the first dedup pass we end up with the 1 vertex they
    // were all assigned to → exact count depends on grid placement, but it
    // must drop below the raw count.
    const { mesh } = meshFromBufferGeometry(g, { weldEpsilon: 1.0 });
    expect(mesh.nVertices).toBeLessThan(6);
  });

  it('non-indexed input keeps face winding (CCW from outside) on cube', () => {
    const g = new THREE.BoxGeometry().toNonIndexed();
    const { mesh, positions } = meshFromBufferGeometry(g);
    // For a unit-centred cube, sum of (v0 - centroid) · normal across faces
    // should be positive — sanity that the welded faces still wind outwards.
    let totalSignedArea = 0;
    for (let f = 0; f < mesh.nFaces; f++) {
      const verts = [...mesh.verticesOfFace(f)];
      const a = positions[verts[0]!]!;
      const b = positions[verts[1]!]!;
      const c = positions[verts[2]!]!;
      const ux = b[0] - a[0],
        uy = b[1] - a[1],
        uz = b[2] - a[2];
      const vx = c[0] - a[0],
        vy = c[1] - a[1],
        vz = c[2] - a[2];
      // Cross product gives 2× face normal weighted by area.
      const nx = uy * vz - uz * vy;
      const ny = uz * vx - ux * vz;
      const nz = ux * vy - uy * vx;
      // Centroid of cube is (0,0,0); a is on the boundary.
      totalSignedArea += a[0] * nx + a[1] * ny + a[2] * nz;
    }
    expect(totalSignedArea).toBeGreaterThan(0);
  });
});

// ---------------------------------------------------------------------------
// Smoke test against the largest fixture mesh we keep around (the Utah teapot).
// ---------------------------------------------------------------------------

describe('meshFromBufferGeometry — fixture smoke tests', () => {
  it('teapot fixture (1601 vertices) round-trips via indexed BufferGeometry', () => {
    const fix = loadFixture('teapot-near');
    const flat = new Float32Array(fix.mesh.vertices.flat());
    const indices = new Uint32Array(fix.mesh.faces.flat());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(flat, 3));
    g.setIndex(new THREE.BufferAttribute(indices, 1));
    const { mesh, positions } = meshFromBufferGeometry(g);
    expect(mesh.nVertices).toBe(fix.mesh.vertices.length);
    expect(mesh.nFaces).toBe(fix.mesh.faces.length);
    expect(positions.length).toBe(fix.mesh.vertices.length);
  });

  it('icosahedron fixture round-trips', () => {
    const fix = loadFixture('icosahedron-edge');
    const flat = new Float32Array(fix.mesh.vertices.flat());
    const indices = new Uint32Array(fix.mesh.faces.flat());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(flat, 3));
    g.setIndex(new THREE.BufferAttribute(indices, 1));
    const { mesh } = meshFromBufferGeometry(g);
    expect(mesh.nVertices).toBe(12);
    expect(mesh.nFaces).toBe(20);
  });
});

// ---------------------------------------------------------------------------
// Error contract.
// ---------------------------------------------------------------------------

describe('meshFromBufferGeometry — error cases', () => {
  it('throws when position attribute is missing', () => {
    const g = new THREE.BufferGeometry();
    expect(() => meshFromBufferGeometry(g)).toThrow(/position/);
  });

  it('throws when position itemSize is not 3', () => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array([0, 0, 0, 0]), 2));
    expect(() => meshFromBufferGeometry(g)).toThrow(/itemSize/);
  });

  it('throws when indexed geometry has index count not divisible by 3', () => {
    const g = new THREE.BufferGeometry();
    g.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]), 3),
    );
    g.setIndex(new THREE.BufferAttribute(new Uint16Array([0, 1, 2, 0]), 1));
    expect(() => meshFromBufferGeometry(g)).toThrow(/divisible by 3/);
  });

  it('throws when non-indexed position count is not divisible by 3', () => {
    const g = new THREE.BufferGeometry();
    g.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0]), 3),
    );
    expect(() => meshFromBufferGeometry(g)).toThrow(/divisible by 3/);
  });

  it('throws when any position is NaN', () => {
    const g = new THREE.BufferGeometry();
    g.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array([0, 0, 0, NaN, 0, 0, 0, 1, 0]), 3),
    );
    expect(() => meshFromBufferGeometry(g)).toThrow(/non-finite/);
  });

  it('throws when any position is Infinity', () => {
    const g = new THREE.BufferGeometry();
    g.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array([0, 0, 0, Infinity, 0, 0, 0, 1, 0]), 3),
    );
    expect(() => meshFromBufferGeometry(g)).toThrow(/non-finite/);
  });
});

// ---------------------------------------------------------------------------
// Internal sanity: the welded mesh has positions at the expected places.
// ---------------------------------------------------------------------------

describe('meshFromBufferGeometry — welded positions land at canonical corners', () => {
  it('non-indexed cube: the 8 welded positions are the 8 cube corners', () => {
    const g = new THREE.BoxGeometry().toNonIndexed();
    const { positions } = meshFromBufferGeometry(g);
    // Each component should be ±0.5 (BoxGeometry default size = 1).
    const set = new Set<string>();
    for (const p of positions) {
      const [x, y, z] = p as Vec3;
      expect(Math.abs(Math.abs(x) - 0.5)).toBeLessThan(1e-6);
      expect(Math.abs(Math.abs(y) - 0.5)).toBeLessThan(1e-6);
      expect(Math.abs(Math.abs(z) - 0.5)).toBeLessThan(1e-6);
      set.add(`${Math.sign(x)},${Math.sign(y)},${Math.sign(z)}`);
    }
    expect(set.size).toBe(8);
  });
});
