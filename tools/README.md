# tools/

Helper scripts that don't ship with the package.

## `gen_fixtures.py` — golden fixture regenerator

Uses [`potpourri3d`](https://github.com/nmwsharp/potpourri3d) (Python wrapper around
geometry-central) to produce JSON fixtures consumed by the TS test suite. The TS suite
reads `fixtures/*.json` directly; Python is only needed to regenerate.

```bash
cd tools
python3 -m venv .venv
source .venv/bin/activate
pip install potpourri3d numpy trimesh networkx
python gen_fixtures.py --out ../fixtures
```

Each fixture is a JSON object (one file per query, written to
`<--out>/<name>.json`):

```jsonc
{
  "schema_version": 1,
  "name": "tetrahedron-edge",
  "note": "human-readable description of the query",
  "mesh": {
    "vertices": [[x, y, z], ...],
    "faces":    [[a, b, c], ...]
  },
  "query": { "src": 0, "dst": 1 },
  "expected": {
    "path_length": 2.8284271,
    "path_points": [[x, y, z], ...]
  },
  "potpourri3d_version": "1.4.0"
}
```

`potpourri3d.EdgeFlipGeodesicSolver` does not expose the intrinsic / hit-edge
sequence to Python (only the polyline of 3D points), so no `intrinsic_edges`
field is emitted. If you change the schema, bump `schema_version` and update
the TS consumer at `test/_helpers/load-fixture.ts`.

## Mesh assets

The Newell **Utah teapot** is checked into `tools/data/teapot.obj` (curated copy
from [`alecjacobson/common-3d-test-models`](https://github.com/alecjacobson/common-3d-test-models)).
The raw mesh is non-manifold and composed of four disconnected pieces (body,
lid, spout, handle) that share vertex *positions* but not *indices*; the
generator welds-by-position via `trimesh` and keeps only the largest connected
component (~1601 V, 3160 F, the teapot body) so potpourri3d's manifold check
passes. See `utah_teapot()` in `gen_fixtures.py` for details.

The TS-side loader is `test/_helpers/load-fixture.ts` — it accepts a fixture
name (without the `.json` extension) and converts snake_case to camelCase.
