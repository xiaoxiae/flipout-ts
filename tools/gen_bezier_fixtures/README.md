# gen_bezier_fixtures

C++ tool that generates golden JSON fixtures for the **Bezier subdivision**
extension of FlipOut, using the geometry-central reference implementation.

The Python wrapper (`potpourri3d`) we use for ordinary geodesic fixtures does
not expose `FlipEdgeNetwork::bezierSubdivide`, so we link geometry-central
directly here.

## Build & run

```bash
# Once after first checkout (or after the submodule pin moves):
git submodule update --init --recursive

# Configure & build:
cmake -S tools/gen_bezier_fixtures -B tools/gen_bezier_fixtures/build
cmake --build tools/gen_bezier_fixtures/build -j

# Run from the repo root (default --repo-root . --out-dir fixtures):
./tools/gen_bezier_fixtures/build/gen_bezier_fixtures

# Or run a single case (substring match against case name):
./tools/gen_bezier_fixtures/build/gen_bezier_fixtures --only spot
```

Each invocation overwrites the JSON files for the cases it ran — the tool is
idempotent.

## Adding a case

Append to the `CASES` list in `main.cpp`:

```cpp
{"tools/data/<mesh>.obj",
 "<fixture-name>",          // → fixtures/<fixture-name>.json
 "<one-line description>",
 {<vert idx>, <vert idx>, ...},  // control vertices (>=2)
 <n_rounds>,
 <closed?>}                       // true → closed Bezier loop
```

Mesh paths are relative to `--repo-root`. The mesh must be loadable as a
`ManifoldSurfaceMesh` — pre-weld and reduce to a single connected component if
needed. (Existing OBJs in `tools/data/` and `tools/data/bench/` are usable.)

## Output schema

See the header comment in `main.cpp`. The TS-side loader at
`test/_helpers/load-fixture.ts` dispatches on the `kind` field
(`"geodesic"` for the existing v1 fixtures, `"bezier"` for these v2 fixtures)
and converts the snake_case JSON keys to camelCase.

## geometry-central submodule

The submodule lives at `tools/gen_bezier_fixtures/extern/geometry-central` and
is pinned to a specific upstream commit. The pinned SHA is written into every
fixture as `geometry_central_sha` so drift is visible per-fixture. Bumping the
pin: `cd tools/gen_bezier_fixtures/extern/geometry-central && git fetch && git
checkout <sha>`, then commit the submodule update from the repo root.

## Cross-runtime drift on near-degenerate geometries

The corresponding TS test (`test/unit/flipout/bezier-fixtures.test.ts`) compares
this tool's reference path lengths against the TS port's `bezierSubdivide`
output with a **1% relative tolerance**. 14 of 17 fixtures match to machine
epsilon; three (`teapot-bezier-3pt-r1`, `teapot-bezier-3pt-r3`,
`spot-bezier-5pt-r3`) drift by 1e-4 to 3e-3.

Root cause is **not algorithmic** — both implementations follow the same
algorithm and produce the same flip sequence on the same edges. The drift
comes from V8's `Math.acos` returning a bit pattern that differs from glibc's
`std::acos` by 1 ulp on specific inputs (e.g. q=0.097920746884416418). On
near-degenerate teapot/spot geometries — where many geodesics have
within-ulp-equal lengths — that ulp drift cascades into the FlipOut priority
queue and flips a tiebreak the other way, producing a valid-but-different
Bezier curve. See `CLAUDE.md` ("Bezier subdivision") for the full investigation.

If you regenerate fixtures and the TS test starts failing on **other**
fixtures (not the three above), that's a real algorithmic regression, not
ulp drift. Investigate before relaxing the tolerance.
