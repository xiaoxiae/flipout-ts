// New code (no geometry-central counterpart — geometry-central has no
// Three.js dependency).
//
// L5 — Three.js adapter.
//
// Two thin convenience surfaces:
//
//   1. `meshFromBufferGeometry(geom, opts?)` reads a `THREE.BufferGeometry`
//      and produces a `{ mesh, positions }` pair: a connectivity-only
//      `SurfaceMesh` plus a parallel `Vec3[]`. Indexed geometries are
//      consumed verbatim; non-indexed geometries are welded by position
//      using a quantized hash-grid (default tolerance: `1e-7` × bbox
//      diagonal, with a `1e-12` fallback when the bbox is degenerate).
//
//   2. `pathToVector3Array(path)` / `pathToBufferGeometry(path)` lift a
//      `Vec3[]` polyline back into Three.js. The geometry is non-indexed
//      and matches `THREE.Line` semantics (`N` points → `N - 1` segments).
//
// This module is the *only* place in `src/` allowed to import `three`.
// The core mesh / math / geometry / intrinsic / flipout layers operate on
// plain `Vec3` tuples and typed arrays, leaving rendering concerns here.

import * as THREE from 'three';

import type { Vec3 } from '../math/vec3.js';
import { SurfaceMesh, type Triangle } from '../mesh/surface-mesh.js';

/** Result of {@link meshFromBufferGeometry}: connectivity + welded positions. */
export interface MeshFromBufferGeometryResult {
  /** Half-edge mesh built from the (welded) face list. */
  mesh: SurfaceMesh;
  /** Vertex positions in the same order as the mesh's vertex indices. */
  positions: Vec3[];
}

/** Options for {@link meshFromBufferGeometry}. */
export interface MeshFromBufferGeometryOptions {
  /**
   * Absolute distance below which two non-indexed positions are considered
   * the same vertex. Ignored when `geom.index` is set (indexed geometries
   * are consumed verbatim — Three.js already has them deduped).
   *
   * Default: `1e-7 * bboxDiagonal`, falling back to `1e-12` if the bbox is
   * degenerate (all vertices coincident).
   */
  weldEpsilon?: number;
}

const DEFAULT_RELATIVE_EPS = 1e-7;
const ABSOLUTE_EPS_FALLBACK = 1e-12;

/**
 * Convert a `THREE.BufferGeometry` into a {@link SurfaceMesh} and parallel
 * `Vec3[]` positions array. Triangle-only.
 *
 * Indexed geometries are passed through unchanged: Three.js already shares
 * vertices via the index buffer. Non-indexed geometries are welded by
 * position using a hash-grid with cell size `2 * weldEpsilon`, which makes
 * dedup linear in the vertex count. The teapot fixture (1601 vertices) is
 * the tightest realistic case the test suite covers; larger meshes scale
 * the same way.
 *
 * @throws if `geom.attributes.position` is missing
 * @throws if the position attribute has `itemSize !== 3`
 * @throws if the (post-indexing) face list length is not a multiple of 3
 * @throws if any position component is not finite (NaN / Infinity)
 */
export function meshFromBufferGeometry(
  geom: THREE.BufferGeometry,
  options: MeshFromBufferGeometryOptions = {},
): MeshFromBufferGeometryResult {
  const positionAttr = geom.getAttribute('position') as THREE.BufferAttribute | undefined;
  if (positionAttr === undefined) {
    throw new Error(
      'meshFromBufferGeometry: BufferGeometry has no `position` attribute. ' +
        'Pass a populated geometry (e.g. THREE.BoxGeometry, or one with `setAttribute(\'position\', ...)`).',
    );
  }
  if (positionAttr.itemSize !== 3) {
    throw new Error(
      `meshFromBufferGeometry: position attribute itemSize must be 3, got ${positionAttr.itemSize}.`,
    );
  }

  const rawCount = positionAttr.count;
  const rawPositions: Vec3[] = new Array(rawCount);
  for (let i = 0; i < rawCount; i++) {
    const x = positionAttr.getX(i);
    const y = positionAttr.getY(i);
    const z = positionAttr.getZ(i);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      throw new Error(
        `meshFromBufferGeometry: vertex ${i} has non-finite position [${x}, ${y}, ${z}].`,
      );
    }
    rawPositions[i] = [x, y, z];
  }

  const index = geom.getIndex();

  if (index !== null) {
    // Indexed geometry: consume directly. Three.js already shares vertices.
    if (index.count % 3 !== 0) {
      throw new Error(
        `meshFromBufferGeometry: index count ${index.count} is not divisible by 3 ` +
          '(only triangle meshes are supported).',
      );
    }
    const faces: Triangle[] = new Array(index.count / 3);
    for (let f = 0; f < faces.length; f++) {
      const a = index.getX(f * 3 + 0);
      const b = index.getX(f * 3 + 1);
      const c = index.getX(f * 3 + 2);
      faces[f] = [a, b, c];
    }
    const mesh = SurfaceMesh.fromFaces(faces, rawCount);
    return { mesh, positions: rawPositions };
  }

  // Non-indexed: weld by position.
  if (rawCount % 3 !== 0) {
    throw new Error(
      `meshFromBufferGeometry: non-indexed position count ${rawCount} is not divisible by 3 ` +
        '(only triangle meshes are supported).',
    );
  }

  const eps = options.weldEpsilon ?? defaultWeldEpsilon(rawPositions);
  const { positions, remap } = weldByPosition(rawPositions, eps);

  const nFaces = rawCount / 3;
  const faces: Triangle[] = new Array(nFaces);
  for (let f = 0; f < nFaces; f++) {
    const a = remap[f * 3 + 0]!;
    const b = remap[f * 3 + 1]!;
    const c = remap[f * 3 + 2]!;
    faces[f] = [a, b, c];
  }

  // Drop fully-degenerate triangles (all three vertices welded together).
  // Without this `SurfaceMesh.fromFaces` rejects the input (self-edge), but
  // for non-indexed input the user usually wants the geometry consumed
  // anyway — the duplicate triangle was just a render-time artifact.
  const nonDegenerateFaces: Triangle[] = [];
  for (const tri of faces) {
    const [a, b, c] = tri;
    if (a !== b && b !== c && c !== a) nonDegenerateFaces.push(tri);
  }

  const mesh = SurfaceMesh.fromFaces(nonDegenerateFaces, positions.length);
  return { mesh, positions };
}

/** Compute the default weld epsilon as a fraction of the bounding-box diagonal. */
function defaultWeldEpsilon(positions: readonly Vec3[]): number {
  if (positions.length === 0) return ABSOLUTE_EPS_FALLBACK;
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;
  for (const p of positions) {
    if (p[0] < minX) minX = p[0];
    if (p[1] < minY) minY = p[1];
    if (p[2] < minZ) minZ = p[2];
    if (p[0] > maxX) maxX = p[0];
    if (p[1] > maxY) maxY = p[1];
    if (p[2] > maxZ) maxZ = p[2];
  }
  const dx = maxX - minX;
  const dy = maxY - minY;
  const dz = maxZ - minZ;
  const diag = Math.sqrt(dx * dx + dy * dy + dz * dz);
  if (diag === 0) return ABSOLUTE_EPS_FALLBACK;
  return DEFAULT_RELATIVE_EPS * diag;
}

/**
 * Weld duplicate positions using a hash-grid with cell size `2 * eps`. Each
 * input vertex is mapped to one of (potentially) eight neighbouring cells
 * to avoid edge-of-cell false negatives, then matched against bucket
 * occupants by Euclidean distance.
 *
 * Worst case is `O(n * k)` where `k` is the average number of vertices in
 * the 27 neighbouring cells — `O(1)` in practice for well-spread meshes.
 */
function weldByPosition(
  raw: readonly Vec3[],
  eps: number,
): { positions: Vec3[]; remap: number[] } {
  const cellSize = Math.max(eps * 2, Number.MIN_VALUE);
  const eps2 = eps * eps;
  const buckets = new Map<string, number[]>();
  const positions: Vec3[] = [];
  const remap = new Array<number>(raw.length);

  const cellKey = (cx: number, cy: number, cz: number): string => `${cx},${cy},${cz}`;

  for (let i = 0; i < raw.length; i++) {
    const p = raw[i]!;
    const cx = Math.floor(p[0] / cellSize);
    const cy = Math.floor(p[1] / cellSize);
    const cz = Math.floor(p[2] / cellSize);

    let merged = -1;
    outer: for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        for (let dz = -1; dz <= 1; dz++) {
          const bucket = buckets.get(cellKey(cx + dx, cy + dy, cz + dz));
          if (bucket === undefined) continue;
          for (const j of bucket) {
            const q = positions[j]!;
            const ex = p[0] - q[0];
            const ey = p[1] - q[1];
            const ez = p[2] - q[2];
            if (ex * ex + ey * ey + ez * ez <= eps2) {
              merged = j;
              break outer;
            }
          }
        }
      }
    }

    if (merged >= 0) {
      remap[i] = merged;
    } else {
      const newIdx = positions.length;
      positions.push(p);
      const key = cellKey(cx, cy, cz);
      const bucket = buckets.get(key);
      if (bucket === undefined) buckets.set(key, [newIdx]);
      else bucket.push(newIdx);
      remap[i] = newIdx;
    }
  }

  return { positions, remap };
}

// ---------------------------------------------------------------------------
// Path → Three.js
// ---------------------------------------------------------------------------

/**
 * Convert a path (sequence of `Vec3` points) into an array of
 * `THREE.Vector3` instances. Trivial map; one allocation per point.
 */
export function pathToVector3Array(path: readonly Vec3[]): THREE.Vector3[] {
  const out: THREE.Vector3[] = new Array(path.length);
  for (let i = 0; i < path.length; i++) {
    const p = path[i]!;
    out[i] = new THREE.Vector3(p[0], p[1], p[2]);
  }
  return out;
}

/**
 * Convert a path to a non-indexed `THREE.BufferGeometry` carrying just a
 * `position` attribute. The result is meant to be wrapped in `THREE.Line`
 * (or `THREE.LineLoop` / `MeshLine`); `N` points produce `N - 1` segments
 * under standard `THREE.Line` semantics.
 *
 * The vertices are *not* pre-duplicated as line segments — use this with
 * `THREE.Line`, not `THREE.LineSegments`.
 */
export function pathToBufferGeometry(path: readonly Vec3[]): THREE.BufferGeometry {
  const arr = new Float32Array(path.length * 3);
  for (let i = 0; i < path.length; i++) {
    const p = path[i]!;
    arr[i * 3 + 0] = p[0];
    arr[i * 3 + 1] = p[1];
    arr[i * 3 + 2] = p[2];
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(arr, 3));
  return geom;
}
