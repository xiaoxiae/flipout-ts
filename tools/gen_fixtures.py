"""Generate golden FlipOut geodesic fixtures using potpourri3d.

This script builds a small library of triangle meshes (tetrahedron, cube,
icosahedron, icosphere, flat grid, flat quad), runs
``potpourri3d.EdgeFlipGeodesicSolver.find_geodesic_path`` on a curated set of
(src, dst) vertex pairs, and writes one JSON file per query into the chosen
output directory. The TS test suite reads these JSON files directly; Python
is only needed to regenerate.

Run:

    python3 -m venv .venv
    source .venv/bin/activate
    pip install potpourri3d numpy
    python gen_fixtures.py --out ../fixtures
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import potpourri3d as pp3d
from importlib.metadata import PackageNotFoundError, version as _pkg_version


SCHEMA_VERSION = 1


# ---------------------------------------------------------------------------
# Mesh library
# ---------------------------------------------------------------------------


def regular_tetrahedron() -> tuple[np.ndarray, np.ndarray]:
    """Regular tetrahedron inscribed in the unit cube vertices."""
    V = np.array(
        [
            [1.0, 1.0, 1.0],
            [-1.0, -1.0, 1.0],
            [-1.0, 1.0, -1.0],
            [1.0, -1.0, -1.0],
        ],
        dtype=np.float64,
    )
    # Outward-facing winding (right-hand rule -> normal points away from centroid).
    F = np.array(
        [
            [0, 1, 2],
            [0, 3, 1],
            [0, 2, 3],
            [1, 3, 2],
        ],
        dtype=np.int32,
    )
    return V, F


def unit_cube() -> tuple[np.ndarray, np.ndarray]:
    """Unit cube on [0, 1]^3, 12 triangles (2 per face), outward winding."""
    V = np.array(
        [
            [0.0, 0.0, 0.0],  # 0
            [1.0, 0.0, 0.0],  # 1
            [1.0, 1.0, 0.0],  # 2
            [0.0, 1.0, 0.0],  # 3
            [0.0, 0.0, 1.0],  # 4
            [1.0, 0.0, 1.0],  # 5
            [1.0, 1.0, 1.0],  # 6
            [0.0, 1.0, 1.0],  # 7
        ],
        dtype=np.float64,
    )
    # Each face split into two triangles, all wound CCW when viewed from outside.
    F = np.array(
        [
            # bottom (z=0), normal -z
            [0, 2, 1],
            [0, 3, 2],
            # top (z=1), normal +z
            [4, 5, 6],
            [4, 6, 7],
            # front (y=0), normal -y
            [0, 1, 5],
            [0, 5, 4],
            # back (y=1), normal +y
            [3, 7, 6],
            [3, 6, 2],
            # left (x=0), normal -x
            [0, 4, 7],
            [0, 7, 3],
            # right (x=1), normal +x
            [1, 2, 6],
            [1, 6, 5],
        ],
        dtype=np.int32,
    )
    return V, F


def icosahedron() -> tuple[np.ndarray, np.ndarray]:
    """Regular icosahedron with vertices on the unit sphere."""
    phi = (1.0 + 5.0**0.5) / 2.0
    raw = np.array(
        [
            [-1, phi, 0],
            [1, phi, 0],
            [-1, -phi, 0],
            [1, -phi, 0],
            [0, -1, phi],
            [0, 1, phi],
            [0, -1, -phi],
            [0, 1, -phi],
            [phi, 0, -1],
            [phi, 0, 1],
            [-phi, 0, -1],
            [-phi, 0, 1],
        ],
        dtype=np.float64,
    )
    V = raw / np.linalg.norm(raw, axis=1, keepdims=True)
    F = np.array(
        [
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
        ],
        dtype=np.int32,
    )
    return V, F


def icosphere(subdivisions: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Subdivide an icosahedron and project onto the unit sphere."""
    V, F = icosahedron()
    V = V.tolist()
    F = F.tolist()
    for _ in range(subdivisions):
        midpoint_cache: dict[tuple[int, int], int] = {}

        def midpoint(a: int, b: int) -> int:
            key = (a, b) if a < b else (b, a)
            if key in midpoint_cache:
                return midpoint_cache[key]
            pa, pb = V[a], V[b]
            m = [(pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5, (pa[2] + pb[2]) * 0.5]
            n = (m[0] * m[0] + m[1] * m[1] + m[2] * m[2]) ** 0.5
            V.append([m[0] / n, m[1] / n, m[2] / n])
            idx = len(V) - 1
            midpoint_cache[key] = idx
            return idx

        new_F: list[list[int]] = []
        for a, b, c in F:
            ab = midpoint(a, b)
            bc = midpoint(b, c)
            ca = midpoint(c, a)
            new_F.append([a, ab, ca])
            new_F.append([b, bc, ab])
            new_F.append([c, ca, bc])
            new_F.append([ab, bc, ca])
        F = new_F

    return np.asarray(V, dtype=np.float64), np.asarray(F, dtype=np.int32)


def flat_grid(n: int = 4, size: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """Flat n x n vertex grid in the z=0 plane, right-triangle topology."""
    if n < 2:
        raise ValueError("grid needs at least 2 vertices per side")
    xs = np.linspace(0.0, size, n)
    ys = np.linspace(0.0, size, n)
    V = np.array([[x, y, 0.0] for y in ys for x in xs], dtype=np.float64)

    def vid(i: int, j: int) -> int:
        return j * n + i

    F: list[list[int]] = []
    for j in range(n - 1):
        for i in range(n - 1):
            v00 = vid(i, j)
            v10 = vid(i + 1, j)
            v01 = vid(i, j + 1)
            v11 = vid(i + 1, j + 1)
            # Two right triangles, both CCW when viewed from +z.
            F.append([v00, v10, v11])
            F.append([v00, v11, v01])
    return V, np.asarray(F, dtype=np.int32)


def flat_quad() -> tuple[np.ndarray, np.ndarray]:
    """Single quad split into 2 triangles in z=0."""
    V = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    F = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    return V, F


# ---------------------------------------------------------------------------
# OBJ loader (positions + triangle faces only — ignores vt/vn/groups/etc.)
# ---------------------------------------------------------------------------


def load_obj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Minimal OBJ reader. Triangulates n-gons via fan (face[0], face[i], face[i+1])."""
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    with path.open("r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            tag, *rest = line.split()
            if tag == "v":
                vertices.append((float(rest[0]), float(rest[1]), float(rest[2])))
            elif tag == "f":
                # OBJ indices are 1-based; entries can be `v`, `v/vt`, `v/vt/vn`, or `v//vn`.
                idx = [int(tok.split("/")[0]) - 1 for tok in rest]
                for i in range(1, len(idx) - 1):
                    faces.append((idx[0], idx[i], idx[i + 1]))
    V = np.asarray(vertices, dtype=np.float64)
    F = np.asarray(faces, dtype=np.int32)
    return V, F


def utah_teapot() -> tuple[np.ndarray, np.ndarray]:
    """Newell's Utah teapot (curated copy from alecjacobson/common-3d-test-models).

    The raw mesh is non-manifold (vertex 1833 sits on multiple boundary loops, and
    the teapot is composed of four disconnected pieces — body, lid, spout, handle —
    that share vertex *positions* but not vertex *indices*). potpourri3d's
    EdgeFlipGeodesicSolver requires a connected manifold input, so we:

      1. Weld vertices by position via ``trimesh.load(..., process=True)``.
      2. Split into connected components and keep the largest (the body).

    This yields a connected, manifold-with-boundary mesh of ~1601 vertices
    suitable for FlipOut.
    """
    import trimesh

    raw = trimesh.load(
        Path(__file__).parent / "data" / "teapot.obj",
        process=True,
        force="mesh",
    )
    components = sorted(raw.split(only_watertight=False), key=lambda p: -len(p.faces))
    body = components[0]
    V = np.ascontiguousarray(body.vertices, dtype=np.float64)
    F = np.ascontiguousarray(body.faces, dtype=np.int32)
    return V, F


# ---------------------------------------------------------------------------
# Fixture pipeline
# ---------------------------------------------------------------------------


@dataclass
class Query:
    name: str
    src: int
    dst: int
    note: str = ""


@dataclass
class MeshSpec:
    key: str
    builder: Callable[[], tuple[np.ndarray, np.ndarray]]
    queries: list[Query]


def _path_length(points: np.ndarray) -> float:
    if points.shape[0] < 2:
        return 0.0
    diffs = np.diff(points, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def _potpourri3d_version() -> str:
    try:
        return _pkg_version("potpourri3d")
    except PackageNotFoundError:
        return "unknown"


def build_fixtures(out_dir: Path) -> list[tuple[str, float, int]]:
    """Generate every fixture and write JSON files. Returns a summary list."""
    out_dir.mkdir(parents=True, exist_ok=True)

    pp3d_version = _potpourri3d_version()
    summary: list[tuple[str, float, int]] = []

    specs: list[MeshSpec] = [
        MeshSpec(
            key="tetrahedron",
            builder=regular_tetrahedron,
            queries=[
                Query("tetrahedron-edge", 0, 1, "adjacent vertices, single edge"),
                Query("tetrahedron-opposite", 0, 3, "across one face"),
            ],
        ),
        MeshSpec(
            key="cube",
            builder=unit_cube,
            queries=[
                Query("cube-edge", 0, 1, "single shared edge"),
                Query("cube-face-diagonal", 0, 2, "diagonal across one face"),
                Query("cube-space-diagonal", 0, 6, "antipodal corners"),
            ],
        ),
        MeshSpec(
            key="icosahedron",
            builder=icosahedron,
            queries=[
                Query("icosahedron-edge", 0, 1, "adjacent vertices, single edge"),
                Query("icosahedron-mid", 0, 4, "two-edge hop across one face strip"),
                Query("icosahedron-antipodal", 0, 3, "vertex 0 to its antipode"),
            ],
        ),
        MeshSpec(
            key="icosphere",
            builder=lambda: icosphere(subdivisions=1),
            queries=[
                Query("icosphere-short", 0, 5, "short hop on subdivided sphere"),
                Query("icosphere-antipodal", 0, 3, "near-antipodal hop"),
            ],
        ),
        MeshSpec(
            key="grid",
            builder=lambda: flat_grid(n=4, size=1.0),
            queries=[
                Query("grid-edge", 0, 1, "adjacent grid vertices"),
                Query("grid-diagonal", 0, 15, "opposite corners of 4x4 grid"),
                Query("grid-mid", 5, 10, "interior diagonal hop"),
            ],
        ),
        MeshSpec(
            key="quad",
            builder=flat_quad,
            queries=[
                Query("quad-edge", 0, 1, "single edge"),
                Query("quad-diagonal", 0, 2, "shared diagonal"),
                Query("quad-across", 1, 3, "non-shared diagonal -> goes across both tris"),
            ],
        ),
        MeshSpec(
            key="teapot",
            builder=utah_teapot,
            queries=[
                Query("teapot-near", 0, 50, "nearby vertices on the body (Utah teapot, body component only)"),
                Query("teapot-mid", 0, 800, "across half the body"),
                Query("teapot-far", 0, 1600, "approximately opposite ends of the body"),
            ],
        ),
    ]

    # Avoid duplicated names (e.g. icosahedron-mid / antipodal both pick vert 3 in
    # the regular icosahedron — that's intentional, but rename one).
    queries_seen: dict[str, str] = {}

    for spec in specs:
        V, F = spec.builder()
        # Make sure dtypes match what potpourri3d expects.
        V = np.ascontiguousarray(V, dtype=np.float64)
        F = np.ascontiguousarray(F, dtype=np.int32)

        solver = pp3d.EdgeFlipGeodesicSolver(V, F)

        for q in spec.queries:
            if q.name in queries_seen:
                raise RuntimeError(
                    f"duplicate fixture name {q.name!r} (also in {queries_seen[q.name]})"
                )
            queries_seen[q.name] = spec.key

            if not (0 <= q.src < len(V) and 0 <= q.dst < len(V)):
                raise IndexError(
                    f"{q.name}: src/dst out of range for mesh '{spec.key}' "
                    f"(|V|={len(V)})"
                )

            path = solver.find_geodesic_path(int(q.src), int(q.dst))
            path = np.asarray(path, dtype=np.float64)
            length = _path_length(path)

            fixture = {
                "schema_version": SCHEMA_VERSION,
                "name": q.name,
                "note": q.note,
                "mesh": {
                    "vertices": V.tolist(),
                    "faces": F.tolist(),
                },
                "query": {"src": int(q.src), "dst": int(q.dst)},
                "expected": {
                    "path_length": length,
                    "path_points": path.tolist(),
                },
                "potpourri3d_version": pp3d_version,
            }

            target = out_dir / f"{q.name}.json"
            with target.open("w", encoding="utf-8") as fh:
                json.dump(fixture, fh, indent=2)
                fh.write("\n")

            summary.append((q.name, length, len(path)))

    return summary


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        default="../fixtures",
        type=Path,
        help="Directory to write fixture JSON files into (default: ../fixtures)",
    )
    args = parser.parse_args(argv)

    # Resolve relative to the script location, not the current cwd, so the
    # default works whether run from tools/ or the repo root.
    out_dir = args.out
    if not out_dir.is_absolute():
        out_dir = (Path(__file__).resolve().parent / out_dir).resolve()

    summary = build_fixtures(out_dir)

    print(f"\nWrote {len(summary)} fixtures to {out_dir}:")
    print(f"  {'name':<32}  {'length':>12}  {'points':>7}")
    print(f"  {'-' * 32}  {'-' * 12}  {'-' * 7}")
    for name, length, n_points in summary:
        print(f"  {name:<32}  {length:>12.6f}  {n_points:>7d}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
