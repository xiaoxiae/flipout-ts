// gen_bezier_fixtures — emits golden JSON fixtures for the FlipOut Bezier
// subdivision routine, using the geometry-central reference C++ implementation.
//
// Output schema (one file per case, written under <repo>/fixtures/):
//
//   {
//     "schema_version": 2,
//     "kind": "bezier",
//     "name": "<case>",
//     "note": "<human description>",
//     "mesh": { "vertices": [[x,y,z],...], "faces": [[a,b,c],...] },
//     "query": {
//       "control_vertices": [...],
//       "n_rounds": <int>,
//       "closed": <bool>
//     },
//     "expected": {
//       "path_length": <float>,
//       "path_points": [[x,y,z], ...]
//     },
//     "geometry_central_sha": "<sha-of-submodule-at-build-time>"
//   }
//
// Rebuild + run from the repo root:
//
//   cmake -S tools/gen_bezier_fixtures -B tools/gen_bezier_fixtures/build
//   cmake --build tools/gen_bezier_fixtures/build -j
//   ./tools/gen_bezier_fixtures/build/gen_bezier_fixtures \
//       --out-dir fixtures \
//       --repo-root .

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometrycentral/surface/flip_geodesics.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

using geometrycentral::Vector3;
using geometrycentral::surface::FlipEdgeNetwork;
using geometrycentral::surface::ManifoldSurfaceMesh;
using geometrycentral::surface::Vertex;
using geometrycentral::surface::VertexPositionGeometry;
using geometrycentral::surface::readManifoldSurfaceMesh;

namespace fs = std::filesystem;

// --- Case list -------------------------------------------------------------

struct Case {
    std::string mesh_path;          // OBJ path, relative to --repo-root
    std::string out_name;            // fixture filename stem (no .json)
    std::string note;                // free-form description
    std::vector<size_t> control_vertices;
    size_t n_rounds;
    bool closed;
};

// Each case picks well-separated control vertices so the per-segment Dijkstra
// paths don't collide (`bezierSubdivide` requires a simple curve).
//
// All meshes here load as ManifoldSurfaceMesh. The Newell teapot is pre-welded
// to its largest connected component; see tools/data/teapot-welded.obj.
static const std::vector<Case> CASES = {
    // --- teapot (V=1601, F=3160) — primary "large mesh" coverage --------
    {"tools/data/teapot-welded.obj", "teapot-bezier-3pt-r0",
     "teapot, 3 controls, 0 rounds (initial piecewise-geodesic only)", {100, 700, 1300}, 0, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-2pt-r1",
     "teapot, 2 controls, 1 round (single midpoint insertion)", {100, 1300}, 1, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-2pt-r3",
     "teapot, 2 controls, 3 rounds (de Casteljau on a segment)", {100, 1300}, 3, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-3pt-r1",
     "teapot, 3 control vertices, 1 round", {100, 700, 1300}, 1, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-3pt-r3",
     "teapot, 3 control vertices, 3 rounds (convergence)", {100, 700, 1300}, 3, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-4pt-r1",
     "teapot, 4 control vertices, 1 round", {0, 450, 950, 1450}, 1, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-4pt-r3",
     "teapot, 4 control vertices, 3 rounds", {0, 450, 950, 1450}, 3, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-4pt-r5",
     "teapot, 4 control vertices, 5 rounds (deep subdivision)", {0, 450, 950, 1450}, 5, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-5pt-r2",
     "teapot, 5 control vertices, 2 rounds", {80, 400, 800, 1200, 1550}, 2, false},
    {"tools/data/teapot-welded.obj", "teapot-bezier-6pt-r1",
     "teapot, 6 control vertices, 1 round", {30, 300, 600, 900, 1200, 1500}, 1, false},

    // --- spot (V=2930, F=5856) -----------------------------------------
    {"tools/data/bench/spot.obj", "spot-bezier-3pt-r1",
     "spot, 3 control vertices, 1 round", {100, 1200, 2500}, 1, false},
    {"tools/data/bench/spot.obj", "spot-bezier-4pt-r2",
     "spot, 4 control vertices, 2 rounds", {200, 1000, 1900, 2700}, 2, false},
    {"tools/data/bench/spot.obj", "spot-bezier-5pt-r3",
     "spot, 5 control vertices, 3 rounds", {25, 700, 1400, 2000, 2750}, 3, false},

    // --- icosphere (V=42, F=80) — small mesh sanity --------------------
    {"tools/data/icosphere.obj", "icosphere-bezier-3pt-r1",
     "icosphere, 3 control vertices, 1 round", {0, 15, 30}, 1, false},
    {"tools/data/icosphere.obj", "icosphere-bezier-4pt-r2",
     "icosphere, 4 control vertices, 2 rounds", {0, 12, 24, 36}, 2, false},

    // --- larger meshes — single stress fixture each --------------------
    {"tools/data/bench/armadillo.obj", "armadillo-bezier-3pt-r2",
     "armadillo (50k V), 3 control vertices, 2 rounds", {1000, 25000, 49000}, 2, false},
    {"tools/data/bench/nefertiti.obj", "nefertiti-bezier-3pt-r2",
     "nefertiti (50k V), 3 control vertices, 2 rounds", {1000, 25000, 49000}, 2, false},
};

// --- JSON writer (hand-rolled, schema-specific) ----------------------------

struct Json {
    std::ostringstream s;

    Json() {
        s << std::setprecision(17);
    }

    void number(double v) {
        if (std::isfinite(v)) {
            s << v;
        } else {
            std::cerr << "non-finite number in output\n";
            std::exit(1);
        }
    }

    void string(const std::string& v) {
        s << '"';
        for (char c : v) {
            switch (c) {
                case '"': s << "\\\""; break;
                case '\\': s << "\\\\"; break;
                case '\n': s << "\\n"; break;
                case '\r': s << "\\r"; break;
                case '\t': s << "\\t"; break;
                default: s << c;
            }
        }
        s << '"';
    }
};

static void write_vec3(Json& j, const Vector3& v) {
    j.s << "[";
    j.number(v.x); j.s << ",";
    j.number(v.y); j.s << ",";
    j.number(v.z);
    j.s << "]";
}

static void write_face(Json& j, size_t a, size_t b, size_t c) {
    j.s << "[" << a << "," << b << "," << c << "]";
}

// --- Driver ----------------------------------------------------------------

struct CliArgs {
    fs::path repo_root = ".";
    fs::path out_dir = "fixtures";
    std::string only;  // run only the case with this name (substring match)
};

static CliArgs parse_cli(int argc, char** argv) {
    CliArgs a;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << arg << "\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--repo-root") a.repo_root = next();
        else if (arg == "--out-dir") a.out_dir = next();
        else if (arg == "--only") a.only = next();
        else {
            std::cerr << "unknown arg: " << arg << "\n";
            std::exit(2);
        }
    }
    return a;
}

// Read a small text file (e.g. .git submodule SHA snapshot) into a string.
// Returns empty string if missing — geometry_central_sha is best-effort.
static std::string read_gc_sha(const fs::path& repo_root) {
    fs::path head_file =
        repo_root / "tools/gen_bezier_fixtures/extern/geometry-central/.git";
    // The submodule's `.git` may be a file pointing to the parent repo's
    // `.git/modules/...` dir, or a directory. We don't try to be clever —
    // just shell out to `git -C <submodule> rev-parse HEAD` if present.
    fs::path submodule =
        repo_root / "tools/gen_bezier_fixtures/extern/geometry-central";
    if (!fs::exists(submodule)) return "";
    std::string cmd = "git -C \"" + submodule.string() + "\" rev-parse HEAD 2>/dev/null";
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return "";
    char buf[64] = {};
    size_t n = fread(buf, 1, sizeof(buf) - 1, p);
    pclose(p);
    std::string sha(buf, n);
    while (!sha.empty() && (sha.back() == '\n' || sha.back() == '\r' || sha.back() == ' '))
        sha.pop_back();
    return sha;
}

static int run_case(const Case& c, const CliArgs& args, const std::string& gc_sha) {
    fs::path mesh_full = args.repo_root / c.mesh_path;
    std::cout << "[" << c.out_name << "] loading " << mesh_full << std::endl;

    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geom;
    try {
        std::tie(mesh, geom) = readManifoldSurfaceMesh(mesh_full.string());
    } catch (const std::exception& e) {
        std::cerr << "  failed to load mesh: " << e.what() << "\n";
        return 1;
    }

    std::cout << "  V=" << mesh->nVertices()
              << " F=" << mesh->nFaces() << "\n";

    // --- Build control-vertex list ----------------------------------------
    std::vector<Vertex> ctrl;
    ctrl.reserve(c.control_vertices.size());
    for (size_t idx : c.control_vertices) {
        if (idx >= mesh->nVertices()) {
            std::cerr << "  control vertex index " << idx
                      << " out of range (V=" << mesh->nVertices() << ")\n";
            return 1;
        }
        ctrl.push_back(mesh->vertex(idx));
    }

    // --- Run the algorithm ------------------------------------------------
    auto net = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(
        *mesh, *geom, ctrl, c.closed, /*markInterior=*/true);
    if (!net) {
        std::cerr << "  constructFromPiecewiseDijkstraPath returned null\n";
        return 1;
    }
    net->posGeom = geom.get();
    net->bezierSubdivide(c.n_rounds);

    auto polylines3D = net->getPathPolyline3D();
    if (polylines3D.size() != 1) {
        std::cerr << "  expected 1 path, got " << polylines3D.size() << "\n";
        return 1;
    }
    const auto& poly = polylines3D[0];

    double path_length = 0.0;
    for (size_t i = 1; i < poly.size(); ++i) {
        path_length += (poly[i] - poly[i - 1]).norm();
    }

    // --- Emit JSON --------------------------------------------------------
    Json j;
    j.s << "{\n";
    j.s << "  \"schema_version\": 2,\n";
    j.s << "  \"kind\": \"bezier\",\n";
    j.s << "  \"name\": "; j.string(c.out_name); j.s << ",\n";
    j.s << "  \"note\": "; j.string(c.note); j.s << ",\n";

    // mesh.vertices
    j.s << "  \"mesh\": {\n";
    j.s << "    \"vertices\": [\n";
    for (size_t i = 0; i < mesh->nVertices(); ++i) {
        j.s << "      ";
        write_vec3(j, geom->vertexPositions[mesh->vertex(i)]);
        if (i + 1 < mesh->nVertices()) j.s << ",";
        j.s << "\n";
    }
    j.s << "    ],\n";
    j.s << "    \"faces\": [\n";
    {
        std::vector<std::array<size_t, 3>> faces;
        faces.reserve(mesh->nFaces());
        for (auto f : mesh->faces()) {
            auto h = f.halfedge();
            faces.push_back({h.vertex().getIndex(),
                             h.next().vertex().getIndex(),
                             h.next().next().vertex().getIndex()});
        }
        for (size_t i = 0; i < faces.size(); ++i) {
            j.s << "      ";
            write_face(j, faces[i][0], faces[i][1], faces[i][2]);
            if (i + 1 < faces.size()) j.s << ",";
            j.s << "\n";
        }
    }
    j.s << "    ]\n";
    j.s << "  },\n";

    // query
    j.s << "  \"query\": {\n";
    j.s << "    \"control_vertices\": [";
    for (size_t i = 0; i < c.control_vertices.size(); ++i) {
        if (i) j.s << ",";
        j.s << c.control_vertices[i];
    }
    j.s << "],\n";
    j.s << "    \"n_rounds\": " << c.n_rounds << ",\n";
    j.s << "    \"closed\": " << (c.closed ? "true" : "false") << "\n";
    j.s << "  },\n";

    // expected
    j.s << "  \"expected\": {\n";
    j.s << "    \"path_length\": "; j.number(path_length); j.s << ",\n";
    j.s << "    \"path_points\": [\n";
    for (size_t i = 0; i < poly.size(); ++i) {
        j.s << "      ";
        write_vec3(j, poly[i]);
        if (i + 1 < poly.size()) j.s << ",";
        j.s << "\n";
    }
    j.s << "    ]\n";
    j.s << "  },\n";

    j.s << "  \"geometry_central_sha\": "; j.string(gc_sha); j.s << "\n";
    j.s << "}\n";

    fs::path out_path = args.repo_root / args.out_dir / (c.out_name + ".json");
    fs::create_directories(out_path.parent_path());
    {
        std::ofstream f(out_path);
        if (!f) {
            std::cerr << "  failed to open output: " << out_path << "\n";
            return 1;
        }
        f << j.s.str();
    }
    std::cout << "  wrote " << out_path
              << " (length=" << path_length
              << ", pts=" << poly.size() << ")\n";
    return 0;
}

int main(int argc, char** argv) {
    CliArgs args = parse_cli(argc, argv);
    std::string gc_sha = read_gc_sha(args.repo_root);
    std::cout << "geometry-central sha: " << (gc_sha.empty() ? "(unknown)" : gc_sha) << "\n";

    int failures = 0;
    int ran = 0;
    std::vector<std::string> failed_names;
    for (const auto& c : CASES) {
        if (!args.only.empty() && c.out_name.find(args.only) == std::string::npos)
            continue;
        ++ran;
        int rc = 0;
        try {
            rc = run_case(c, args, gc_sha);
        } catch (const std::exception& e) {
            std::cerr << "  exception: " << e.what() << std::endl;
            rc = 1;
        }
        if (rc != 0) {
            ++failures;
            failed_names.push_back(c.out_name);
        }
    }
    std::cout << "ran " << ran << " case(s), " << failures << " failure(s)\n";
    for (const auto& n : failed_names) std::cout << "  FAIL: " << n << "\n";
    return failures == 0 ? 0 : 1;
}
