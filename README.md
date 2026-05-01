# flipout-ts

TypeScript port of **FlipOut** — geodesic path computation by edge flips on triangle meshes
([Sharp & Crane, SIGGRAPH Asia 2020](https://nmwsharp.com/research/flip-geodesics/)).

Based on [geometry-central](https://github.com/nmwsharp/geometry-central) (MIT). See `NOTICE` for attribution.

Built for use with **Three.js** (peer dependency, optional).

## Status

The port is layered. Each layer ships with its own tests; later layers depend on earlier ones.

| Layer | Module | Status | Source (geometry-central) |
| --- | --- | --- | --- |
| L0 | `src/math/` — Vec3 ops, barycentric, predicates | ✅ done (116 tests) | `utilities/vector*.h`, `numerical/linear_algebra_utilities.h` |
| L1 | `src/mesh/` — `SurfaceMesh`, iteration, edge flip | ✅ done (129 tests) | `surface/surface_mesh.cpp`, `halfedge_mesh.cpp` |
| L2 | `src/geometry/` — `VertexPositionGeometry` | ✅ done (73 tests) | `surface/vertex_position_geometry.cpp` |
| L3 | `src/intrinsic/` — `SignpostIntrinsicTriangulation` | ✅ done (113 tests) | `surface/signpost_intrinsic_triangulation.cpp` |
| L4 | `src/flipout/` — `FlipEdgeNetwork` (the algorithm) | ✅ done (110 tests) | `surface/flip_geodesics.cpp` |
| L5 | `src/three/` — `THREE.BufferGeometry` adapter | ✅ done (35 tests) | new code |

## Test strategy

Each layer has tests in three styles:

1. **Mathematical invariants** — flips preserve area, lengths satisfy triangle inequality,
   path length monotonically decreases per iteration, etc. No ground truth needed.
2. **Hand-computed examples** — flat quad, tetrahedron edge-to-edge, cylinder unrolling.
   ~10 cases total, each with paper math behind it.
3. **Golden fixtures from `potpourri3d`** — committed JSON in `fixtures/`, regenerated
   by `tools/gen_fixtures.py` (Python + `potpourri3d`, only needed to regenerate).

```bash
npm install
npm run test           # vitest run
npm run test:watch     # vitest watch
npm run test:coverage  # vitest with v8 coverage
npm run typecheck      # tsc --noEmit
npm run build          # emit dist/
```

## Regenerating fixtures

```bash
cd tools
python3 -m venv .venv && source .venv/bin/activate
pip install potpourri3d numpy
python gen_fixtures.py --out ../fixtures
```

The TS test suite reads from `fixtures/` directly — no Python at runtime.

## Port conventions

- **One source file per geometry-central translation unit.** A header banner names the
  source file and SHA of geometry-central it was ported from.
- **Naming preserved where possible** (`SurfaceMesh`, `VertexPositionGeometry`, `flipEdge`).
  TS conventions used for casing (camelCase methods, PascalCase classes).
- **No Three.js inside `src/{math,mesh,geometry,intrinsic,flipout}`.** Three.js types
  appear only in `src/three/`. The core works on plain typed arrays / `Vec3` tuples.
- **Numerical type:** `number` (Float64) throughout, matching geometry-central's `double`.

See `../flipout-demo/` for an interactive demo on the Utah teapot.
