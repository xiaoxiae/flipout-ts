/**
 * Shared mesh factories for the test suite.
 *
 * Each factory returns a plain `{ vertices, faces }` record so the helpers can
 * be consumed by either L0 (math-only) or L1+ tests (which build a
 * `SurfaceMesh` from the data).
 *
 * **Topology must match `tools/gen_fixtures.py`.** The Python script writes
 * golden FlipOut paths with vertex indices baked in; if any of these factories
 * change vertex order or face winding, the JSON fixtures stop lining up.
 */

import type { Vec3 } from '../../src/math/vec3.js';

/** Triangle as an immutable triple of vertex indices. */
export type Triangle = readonly [number, number, number];

/** Trivial mesh container: positions + indexed face list. */
export interface MeshData {
  vertices: Vec3[];
  faces: Triangle[];
}

// ---------------------------------------------------------------------------
// Tetrahedron — matches `tools/gen_fixtures.py::regular_tetrahedron`.
//
// Note: the task brief mentions a tetrahedron with side length sqrt(8/3) (i.e.
// inscribed in the unit sphere). The fixture script instead places the four
// vertices at four corners of the cube `[-1, 1]^3`, giving side length 2*sqrt(2)
// and inscribed-sphere radius sqrt(3). We follow the fixture script because
// the golden JSON files index against that vertex ordering. Down-stream tests
// that need the unit-sphere variant can scale uniformly.
// ---------------------------------------------------------------------------

/**
 * Regular tetrahedron with vertices at four cube corners. Side length = 2√2.
 *
 * Vertices:
 *   0: ( 1,  1,  1)
 *   1: (-1, -1,  1)
 *   2: (-1,  1, -1)
 *   3: ( 1, -1, -1)
 *
 * Faces (CCW outward normals):
 *   [0,1,2], [0,3,1], [0,2,3], [1,3,2]
 */
export function tetrahedron(): MeshData {
  const vertices: Vec3[] = [
    [1, 1, 1],
    [-1, -1, 1],
    [-1, 1, -1],
    [1, -1, -1],
  ];
  const faces: Triangle[] = [
    [0, 1, 2],
    [0, 3, 1],
    [0, 2, 3],
    [1, 3, 2],
  ];
  return { vertices, faces };
}

// ---------------------------------------------------------------------------
// Unit cube — matches `tools/gen_fixtures.py::unit_cube`.
// ---------------------------------------------------------------------------

/**
 * Unit cube on `[0, 1]^3`, 12 triangles (2 per face), all CCW when viewed
 * from outside.
 *
 * Vertex ordering:
 *   0: (0,0,0)  1: (1,0,0)  2: (1,1,0)  3: (0,1,0)
 *   4: (0,0,1)  5: (1,0,1)  6: (1,1,1)  7: (0,1,1)
 */
export function cube(): MeshData {
  const vertices: Vec3[] = [
    [0, 0, 0], // 0
    [1, 0, 0], // 1
    [1, 1, 0], // 2
    [0, 1, 0], // 3
    [0, 0, 1], // 4
    [1, 0, 1], // 5
    [1, 1, 1], // 6
    [0, 1, 1], // 7
  ];
  const faces: Triangle[] = [
    // bottom (z=0), normal -z
    [0, 2, 1],
    [0, 3, 2],
    // top (z=1), normal +z
    [4, 5, 6],
    [4, 6, 7],
    // front (y=0), normal -y
    [0, 1, 5],
    [0, 5, 4],
    // back (y=1), normal +y
    [3, 7, 6],
    [3, 6, 2],
    // left (x=0), normal -x
    [0, 4, 7],
    [0, 7, 3],
    // right (x=1), normal +x
    [1, 2, 6],
    [1, 6, 5],
  ];
  return { vertices, faces };
}

// ---------------------------------------------------------------------------
// Icosahedron — matches `tools/gen_fixtures.py::icosahedron`.
//
// Twelve vertices placed at (±1, ±phi, 0), (0, ±1, ±phi), (±phi, 0, ±1) and
// then projected onto the unit sphere. Twenty equilateral-triangle faces.
// ---------------------------------------------------------------------------

const PHI = (1 + Math.sqrt(5)) / 2;

/** Regular icosahedron inscribed in the unit sphere. */
export function icosahedron(): MeshData {
  const raw: Vec3[] = [
    [-1, PHI, 0], //  0
    [1, PHI, 0], //  1
    [-1, -PHI, 0], //  2
    [1, -PHI, 0], //  3
    [0, -1, PHI], //  4
    [0, 1, PHI], //  5
    [0, -1, -PHI], //  6
    [0, 1, -PHI], //  7
    [PHI, 0, -1], //  8
    [PHI, 0, 1], //  9
    [-PHI, 0, -1], // 10
    [-PHI, 0, 1], // 11
  ];

  const vertices: Vec3[] = raw.map((v) => {
    const n = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return [v[0] / n, v[1] / n, v[2] / n];
  });

  const faces: Triangle[] = [
    [0, 11, 5],
    [0, 5, 1],
    [0, 1, 7],
    [0, 7, 10],
    [0, 10, 11],
    [1, 5, 9],
    [5, 11, 4],
    [11, 10, 2],
    [10, 7, 6],
    [7, 1, 8],
    [3, 9, 4],
    [3, 4, 2],
    [3, 2, 6],
    [3, 6, 8],
    [3, 8, 9],
    [4, 9, 5],
    [2, 4, 11],
    [6, 2, 10],
    [8, 6, 7],
    [9, 8, 1],
  ];

  return { vertices, faces };
}

// ---------------------------------------------------------------------------
// Flat quad — matches `tools/gen_fixtures.py::flat_quad`.
// ---------------------------------------------------------------------------

/** Unit square in z=0, two triangles, CCW from +z. */
export function flatQuad(): MeshData {
  const vertices: Vec3[] = [
    [0, 0, 0], // 0
    [1, 0, 0], // 1
    [1, 1, 0], // 2
    [0, 1, 0], // 3
  ];
  const faces: Triangle[] = [
    [0, 1, 2],
    [0, 2, 3],
  ];
  return { vertices, faces };
}

// ---------------------------------------------------------------------------
// Flat grid — matches `tools/gen_fixtures.py::flat_grid`.
//
// `n` vertices per side, 2*(n-1)^2 triangles, right-triangle topology with
// the diagonal running from (i,j) to (i+1,j+1).
// ---------------------------------------------------------------------------

/**
 * Flat `n × n` vertex grid on `[0, size]^2 × {0}`, right-triangle topology.
 *
 * Vertex `(i, j)` (with `i`, `j` in `[0, n)`) is at `vertices[j * n + i]`,
 * matching `gen_fixtures.py`'s row-major-by-`y` ordering.
 *
 * Faces per cell: `[v00, v10, v11]` and `[v00, v11, v01]`, both CCW from +z.
 *
 * @param n number of vertices per side (must be >= 2)
 * @param size grid extent (default 1.0)
 */
export function flatGrid(n: number, size = 1): MeshData {
  if (!Number.isInteger(n) || n < 2) {
    throw new RangeError(`flatGrid: n must be an integer >= 2, got ${n}`);
  }

  const step = size / (n - 1);
  const vertices: Vec3[] = [];
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      vertices.push([i * step, j * step, 0]);
    }
  }

  const faces: Triangle[] = [];
  const vid = (i: number, j: number): number => j * n + i;
  for (let j = 0; j < n - 1; j++) {
    for (let i = 0; i < n - 1; i++) {
      const v00 = vid(i, j);
      const v10 = vid(i + 1, j);
      const v01 = vid(i, j + 1);
      const v11 = vid(i + 1, j + 1);
      faces.push([v00, v10, v11]);
      faces.push([v00, v11, v01]);
    }
  }

  return { vertices, faces };
}

// ---------------------------------------------------------------------------
// Single-triangle helper. Useful as the smallest possible disk-topology mesh.
// Not in `gen_fixtures.py` — there's no FlipOut path to compute on it.
// ---------------------------------------------------------------------------

/** A single triangle in the z=0 plane. */
export function singleTriangle(): MeshData {
  const vertices: Vec3[] = [
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
  ];
  const faces: Triangle[] = [[0, 1, 2]];
  return { vertices, faces };
}

// ---------------------------------------------------------------------------
// Two-component mesh — two disjoint triangles. Used to verify that the L1
// constructor accepts disconnected manifold input.
// ---------------------------------------------------------------------------

/** Two disjoint triangles sharing no vertices. 6 vertices, 2 faces. */
export function twoDisjointTriangles(): MeshData {
  const vertices: Vec3[] = [
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [10, 10, 0],
    [11, 10, 0],
    [10, 11, 0],
  ];
  const faces: Triangle[] = [
    [0, 1, 2],
    [3, 4, 5],
  ];
  return { vertices, faces };
}
