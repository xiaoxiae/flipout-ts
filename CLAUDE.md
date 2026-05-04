# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
npm run typecheck                       # tsc --noEmit
npm run test                            # vitest run (one shot, used by CI)
npm run test:watch                      # vitest in watch mode
npm run test:coverage                   # vitest run --coverage (v8, HTML in coverage/)
npm run build                           # tsc -p tsconfig.build.json → dist/

# Single test file or single test:
npx vitest run test/unit/flipout/flip-out.test.ts
npx vitest run -t "name of test"

# Demo (separate package with its own node_modules; aliased to ../src so library
# edits hot-reload without rebuilding):
cd demo && npm install && npm run dev   # http://localhost:5173
cd demo && npx tsx scripts/smoke.mts    # node-level import smoke test (CI runs this)

# Regenerate golden geodesic fixtures (only needed if you change the schema or
# add cases; requires Python + potpourri3d, the geometry-central Python wrapper):
cd tools && python3 -m venv .venv && source .venv/bin/activate
pip install potpourri3d numpy trimesh networkx
python gen_fixtures.py --out ../fixtures

# Regenerate bezier fixtures (links geometry-central directly via C++ submodule
# since potpourri3d does not expose bezierSubdivide):
git submodule update --init --recursive
cmake -S tools/gen_bezier_fixtures -B tools/gen_bezier_fixtures/build
cmake --build tools/gen_bezier_fixtures/build -j
./tools/gen_bezier_fixtures/build/gen_bezier_fixtures
```

CI (`.github/workflows/ci.yml`) runs typecheck → test → build → demo build + smoke.
Node ≥20.

## Architecture: layered port of geometry-central

This is a TypeScript port of FlipOut (Sharp & Crane, SIGGRAPH Asia 2020) from
the C++ geometry-central library. The codebase is organized as a strict stack of
layers, each mirroring a geometry-central namespace. Every `.ts` file in `src/`
opens with a header banner naming the geometry-central source files it was
ported from — **preserve and update those banners** when editing.

```
src/math/        L0  Vec2/Vec3 ops, predicates (orient2d), triangle helpers
src/mesh/        L1  SurfaceMesh — manifold half-edge mesh
src/geometry/    L2  VertexPositionGeometry — extrinsic geometry on top of mesh
src/intrinsic/   L3  SignpostIntrinsicTriangulation — parallel intrinsic mesh
src/flipout/     L4  FlipEdgeNetwork — the FlipOut algorithm + Dijkstra helper
src/three/       L5  Three.js adapter (the only module allowed to import three)
```

Higher layers depend only on lower layers. The barrel `src/index.ts`
re-exports the public API; `src/three/index.ts` is exposed as the separate
subpath export `flipout-ts/three` (Three.js is an optional peer dependency).

### Port conventions (do not violate)

- **Naming preserved from geometry-central** where possible (`SurfaceMesh`,
  `flipEdge`, `signpostAngle`, `locallyShortenAt`, `wedgeAngleQueue`). Cases
  adjusted to TS (camelCase methods, PascalCase classes). Comments next to
  ported methods name the gc function they mirror — match control flow exactly,
  don't "improve" the algorithm.
- **No Three.js inside `src/{math,mesh,geometry,intrinsic,flipout}`.** Those
  layers operate on plain `Vec3 = [number, number, number]` tuples and typed
  arrays. Anything Three.js-specific lives in `src/three/`.
- **Numerics:** `number` (Float64) throughout — matches gc's `double`. Magic
  epsilons used inside hot loops are pulled out as named constants at the top of
  the file (e.g. `TRACE_DENOM_EPS`, `SNAP_EPS`). Don't inline them.
- **One source file per gc translation unit.** When porting a new gc file,
  create a parallel TS file rather than merging into an existing one.

### Half-edge convention (load-bearing)

`SurfaceMesh` follows geometry-central's `ManifoldSurfaceMesh` *implicit-twin*
layout — many derivations rely on it:

- `twin(he) = he ^ 1`, `edge(he) = he >> 1`, `firstHalfedge(e) = 2 * e`
- `vertex(he)` is the **tail** (origin) of `he`; tip is `vertex(next(he))`
- Around each face, halfedges are CCW via `next`
- Connectivity stored in `Int32Array` (not `Uint32Array`) so `INVALID_INDEX = -1`
  round-trips cleanly
- Boundary handling diverges from gc: boundary halfedges have
  `face(he) === INVALID_INDEX`, with `next(he)` wired around the boundary loop.
  Most consumers assume closed surfaces.

### Signpost intrinsic triangulation

`SignpostIntrinsicTriangulation` clones the input mesh's connectivity into a
parallel `intrinsicMesh` and stores per-edge `edgeLengths` plus per-halfedge
`signpostAngle`. Critical detail: `signpostAngle[he]` is in radians in the
`[0, vertexAngleSums[v])` wedge — **not** rescaled to `[0, 2π)`. Rescaling
happens at use time inside `halfedgeVector` / `rescaledVertexVector`. The
algorithm mutates the triangulation in-place; rebuild it for each independent
geodesic query.

### FP-determinism notes for the FlipOut/bezier port

When matching gc bit-for-bit, several non-obvious details matter and have
already been ported carefully — preserve them:

- **`vertexAngleSums` is accumulated by iterating halfedges in global index
  order** (`for he in 0..nHe`, sum into `vertexAngleSums[vertex(he)]`), not by
  walking outgoing halfedges per vertex. gc does it that way; per-vertex
  iteration accumulates the same corner angles in a different order and
  produces 1-ulp drift that compounds through every signpost.
- **`standardizeAngle` uses `angle % modulus` directly**, matching gc's
  `std::fmod` semantics for non-negative angles. Don't reintroduce
  `modPositive` (its `+m` then `%m` adds extra rounding) or a
  "snap-near-modulus-to-0" tweak.
- **Wedge queue tie-break** in `WedgeHeap.less` is `(angle, type, segId)`
  ascending — mirroring gc's `std::greater<tuple<double, SegmentAngleType,
  FlipPathSegment>>` lexicographic compare.
- **Stale-entry detection in `flipOut`** uses strict `!==` on type and angle,
  matching gc's `!=`. Re-enqueueing on drift instead of dropping creates an
  infinite cycle (the larger-side queue entry trips the check, gets
  re-enqueued, immediately re-trips, etc.).
- **`wedgeIsClear`** checks both the marked-vertex gate AND the wedge-fan
  edge orbit (no path edge inside the fan). The latter matters once the
  path becomes non-simple — which happens during bezier subdivision when two
  adjacent sub-geodesics share an interior vertex. The class maintains
  `edgeRefCounts` (a `pathsAtEdge`-equivalent index) so `edgeInPath(e)` is
  O(1).

### Bezier subdivision

`FlipEdgeNetwork.bezierSubdivide(nRounds)` ports gc's de-Casteljau-style
geodesic Bezier (Morera et al. 2008 algorithm with FlipOut as the
straightening oracle). Driven by `flipEdgeNetworkFromControlPath` which
builds the initial network from a control-vertex list via piecewise Dijkstra,
marks the controls, and is consumed by `bezierSubdivide`.

**Known gc-divergence on near-degenerate geometries (3 of 17 fixtures):**
14 fixtures match gc to ≤1e-11 (machine epsilon). Three (`teapot-bezier-3pt-r1`,
`teapot-bezier-3pt-r3`, `spot-bezier-5pt-r3`) drift 1e-4 to 3e-3. Root cause is
V8's `Math.acos` vs glibc's `std::acos` returning 1-ulp-different bit patterns
on specific inputs (e.g. q=0.097920746884416418). That ulp drift cascades
through corner angles → signposts → wedge angles, and on near-degenerate
teapot/spot geometries flips a priority-queue tiebreak the other way,
producing a valid-but-different Bezier curve. `Math.PI/2 − Math.asin` and
`Math.atan2(Math.sqrt(1−q²), q)` each match gc on some inputs and not others;
neither reliably matches glibc bit-for-bit.

The bezier fixture test (`test/unit/flipout/bezier-fixtures.test.ts`) uses a
1% relative tolerance to absorb the worst observed drift. Eliminating it
would require a JS port of glibc's `acos`. **Don't tighten the tolerance**
without also addressing this — and don't loosen it further without
investigating: a regression from "ulp drift" to "real algorithmic bug" can
masquerade at this level.

## Tests & fixtures

- `test/unit/<layer>/*.test.ts` — vitest unit tests, one directory per `src/`
  layer.
- `test/_helpers/` — `load-fixture.ts` (snake_case → camelCase JSON loader,
  resolves paths from repo root via `import.meta.url`) and `meshes.ts`
  (programmatic mesh constructors).
- `fixtures/*.json` — golden queries. Two kinds:
  - **`kind: "geodesic"`** (schema_version 1) — generated by
    `tools/gen_fixtures.py` via `potpourri3d` (Python wrapper around
    geometry-central). `potpourri3d.EdgeFlipGeodesicSolver` does not expose
    the intrinsic-edge sequence, so these fixtures only carry the 3D polyline
    + total length.
  - **`kind: "bezier"`** (schema_version 2) — generated by the C++ tool at
    `tools/gen_bezier_fixtures/`, which links geometry-central as a git
    submodule (`extern/geometry-central`, pinned SHA recorded in each fixture
    as `geometry_central_sha`). Needed because `potpourri3d` does not expose
    `bezierSubdivide`.

  Schemas are documented at `tools/README.md` and `tools/gen_bezier_fixtures/README.md`;
  bump `schema_version` and update `load-fixture.ts` if you change either shape.
- `test/_helpers/load-fixture.ts` exports `loadFixture` (geodesic only,
  back-compat), `loadBezierFixture`, and `listFixtures(kind?)`. Iterating
  tests should always pass an explicit kind (`'geodesic'` or `'bezier'`) so
  they don't try to load the wrong shape.
- `tools/data/teapot.obj` — Newell teapot, pre-welded and reduced to its
  largest connected component (the body, ~1601 V / 3160 F) so potpourri3d's
  manifold check passes. The raw mesh is non-manifold and split across four
  pieces; see `utah_teapot()` in `gen_fixtures.py`.
- `tools/data/teapot-welded.obj` — same welded mesh, baked from
  `fixtures/teapot-mid.json` so the C++ bezier-fixture tool can load it via
  `gc::readManifoldSurfaceMesh` without re-implementing the welding pass.

## Demo

`demo/` is a separate Vite + Three.js package with its own `package.json` and
`node_modules`. Its `vite.config.ts` aliases the package import to `../src/`,
so editing library source hot-reloads in the demo without rebuilding `dist/`.
`demo/scripts/smoke.mts` is a node-level import smoke test; `bake-meshes.mjs`
preprocesses meshes into the demo's static assets.
