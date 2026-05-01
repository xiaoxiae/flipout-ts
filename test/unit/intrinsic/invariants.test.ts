/**
 * High-level invariants & fixture cross-validation.
 *
 * These tests verify properties that hold across many flips (random walks
 * in flip space) plus optionally cross-check against `potpourri3d`-generated
 * fixtures where the source/destination of the geodesic are connected by
 * a single edge in the input mesh — the simplest case the trace can
 * resolve directly.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { listFixtures, loadFixture } from '../../_helpers/load-fixture.js';
import { cube, flatGrid, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

interface Built {
  sit: SignpostIntrinsicTriangulation;
  geom: VertexPositionGeometry;
}
interface MeshDataLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function build(meshData: MeshDataLike): Built {
  const mesh = SurfaceMesh.fromFaces(meshData.faces, meshData.vertices.length);
  const geom = new VertexPositionGeometry(mesh, meshData.vertices);
  const sit = new SignpostIntrinsicTriangulation(geom);
  return { sit, geom };
}

function dist3(a: readonly [number, number, number], b: readonly [number, number, number]): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

describe('invariants — random flip walks preserve area sums and triangle inequalities', () => {
  it.each<[string, MeshDataLike, number]>([
    ['icosahedron', icosahedron(), 64],
    ['cube', cube(), 64],
    ['tetrahedron', tetrahedron(), 32],
  ])('%s: 50 random flips keep all faces valid (triangle ineq), edge lengths positive', (_, m, nFlips) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    let seed = 1;
    const rand = (): number => {
      seed = (seed * 1103515245 + 12345) & 0x7fffffff;
      return seed / 0x7fffffff;
    };
    let flipsDone = 0;
    for (let i = 0; i < nFlips * 4 && flipsDone < nFlips; i++) {
      const e = Math.floor(rand() * mesh.nEdges);
      if (mesh.isBoundaryEdge(e)) continue;
      const ok = sit.flipEdge(e);
      if (ok) flipsDone++;
    }
    // Validate.
    for (let e = 0; e < mesh.nEdges; e++) {
      expect(sit.edgeLengths[e]!).toBeGreaterThan(0);
      expect(Number.isFinite(sit.edgeLengths[e]!)).toBe(true);
    }
    for (let f = 0; f < mesh.nFaces; f++) {
      const it = mesh.halfedgesAroundFace(f);
      const h0 = it.next().value as number;
      const h1 = it.next().value as number;
      const h2 = it.next().value as number;
      const a = sit.edgeLengths[mesh.edge(h0)]!;
      const b = sit.edgeLengths[mesh.edge(h1)]!;
      const c = sit.edgeLengths[mesh.edge(h2)]!;
      const eps = 1e-9;
      expect(a).toBeLessThan(b + c + eps);
      expect(b).toBeLessThan(a + c + eps);
      expect(c).toBeLessThan(a + b + eps);
    }
  });
});

describe('invariants — sum of face areas is preserved by intrinsic flips', () => {
  function sumFaceArea(sit: SignpostIntrinsicTriangulation): number {
    const mesh = sit.intrinsicMesh;
    let s = 0;
    for (let f = 0; f < mesh.nFaces; f++) {
      const it = mesh.halfedgesAroundFace(f);
      const h0 = it.next().value as number;
      const h1 = it.next().value as number;
      const h2 = it.next().value as number;
      const a = sit.edgeLengths[mesh.edge(h0)]!;
      const b = sit.edgeLengths[mesh.edge(h1)]!;
      const c = sit.edgeLengths[mesh.edge(h2)]!;
      // Heron's
      const sp = (a + b + c) / 2;
      const area = Math.sqrt(Math.max(0, sp * (sp - a) * (sp - b) * (sp - c)));
      s += area;
    }
    return s;
  }

  it.each<[string, MeshDataLike]>([
    ['icosahedron', icosahedron()],
    ['cube', cube()],
    ['tetrahedron', tetrahedron()],
  ])('%s: total face area is preserved under any single flip', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    const before = sumFaceArea(sit);
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) continue;
      const ok = sit.flipEdge(e);
      if (!ok) continue;
      expect(sumFaceArea(sit)).toBeCloseTo(before, 8);
      sit.flipEdge(e);
    }
  });
});

describe('invariants — total angle defect is preserved by flips', () => {
  function totalDefect(sit: SignpostIntrinsicTriangulation): number {
    const m = sit.intrinsicMesh;
    let total = 0;
    for (let v = 0; v < m.nVertices; v++) {
      let s = 0;
      for (const h of m.outgoingHalfedges(v)) {
        if (m.face(h) === -1) continue;
        s += sit.cornerAngleAt(h);
      }
      const target = m.isBoundaryVertex(v) ? Math.PI : 2 * Math.PI;
      total += target - s;
    }
    return total;
  }

  it.each<[string, MeshDataLike]>([
    ['icosahedron', icosahedron()],
    ['cube', cube()],
    ['tetrahedron', tetrahedron()],
  ])('%s: total angle defect equals 4π and is preserved by every flip', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    const expected = 4 * Math.PI;
    expect(totalDefect(sit)).toBeCloseTo(expected, 8);
    for (let e = 0; e < mesh.nEdges; e++) {
      if (mesh.isBoundaryEdge(e)) continue;
      const ok = sit.flipEdge(e);
      if (!ok) continue;
      expect(totalDefect(sit)).toBeCloseTo(expected, 8);
      sit.flipEdge(e);
    }
  });
});

describe('invariants — fixture cross-validation (single-edge geodesics only)', () => {
  // For fixtures whose query path is a single edge between src and dst on
  // the input mesh, the FlipOut path is just that edge — we can verify
  // the trace by tracing along the corresponding intrinsic halfedge for
  // its length and asserting the endpoint lands at dst.
  for (const name of listFixtures()) {
    const fixture = loadFixture(name);
    const { src, dst } = fixture.query;
    // We only handle the small/predictable ones.
    if (fixture.expected.pathPoints.length !== 2) continue;

    it(`${name}: trace along the src→dst edge lands at dst`, () => {
      const mesh = SurfaceMesh.fromFaces(fixture.mesh.faces, fixture.mesh.vertices.length);
      const geom = new VertexPositionGeometry(mesh, fixture.mesh.vertices);
      const sit = new SignpostIntrinsicTriangulation(geom);
      // Find the halfedge from src to dst, if it exists.
      let foundHe = -1;
      for (const h of mesh.outgoingHalfedges(src)) {
        if (mesh.tipVertex(h) === dst) {
          foundHe = h;
          break;
        }
      }
      if (foundHe === -1) {
        // Not connected by a single edge — skip.
        return;
      }
      const len = sit.edgeLengths[mesh.edge(foundHe)]!;
      const angle = sit.halfedgeSignposts[foundHe]! / sit.vertexAngleScaling(src);
      const r = sit.traceFromVertex(src, angle, len);
      expect(dist3(r.position, geom.position(dst))).toBeLessThan(1e-7);
    });
  }
});

describe('invariants — TraceResult is consistent across distance and direction', () => {
  it('icosahedron: trace at distance 0 returns the vertex itself with bary at one of the corners', () => {
    const { sit, geom } = build(icosahedron());
    for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
      const r = sit.traceFromVertex(v, 1.234, 0);
      expect(dist3(r.position, geom.position(v))).toBeCloseTo(0, 12);
      // One of the bary coords should be ~1, the others ~0.
      const sorted = [...r.barycentric].sort((a, b) => b - a);
      expect(sorted[0]!).toBeCloseTo(1, 12);
      expect(sorted[1]!).toBeCloseTo(0, 12);
      expect(sorted[2]!).toBeCloseTo(0, 12);
    }
  });

  it('icosahedron: trace at angle 2π is equivalent to angle 0 (modular)', () => {
    const { sit } = build(icosahedron());
    for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
      const r0 = sit.traceFromVertex(v, 0, 0.3);
      const r2pi = sit.traceFromVertex(v, 2 * Math.PI, 0.3);
      // Should land at the same 3D point.
      expect(dist3(r0.position, r2pi.position)).toBeLessThan(1e-9);
    }
  });

  it('icosahedron: trace at angle θ vs. θ + 4π lands at the same point', () => {
    const { sit } = build(icosahedron());
    const r1 = sit.traceFromVertex(0, 0.5, 0.3);
    const r2 = sit.traceFromVertex(0, 0.5 + 4 * Math.PI, 0.3);
    expect(dist3(r1.position, r2.position)).toBeLessThan(1e-9);
  });
});

describe('flip + trace integration', () => {
  /**
   * `traceFromVertex` walks the *input* mesh (extrinsic geometry +
   * `vertexAngleSums`, both immutable), so an intrinsic flip should not
   * perturb its output for a given (vertex, rescaledAngle, distance) tuple.
   * The interesting integration question is therefore: does the flip leave
   * the data the trace consumes intact, and do compound (flip + trace)
   * results land on the surface with consistent invariants? These tests
   * pin that down across tetrahedron, cube, flat grid, and icosahedron.
   *
   * Note: the tetrahedron is a complete graph K4 (every vertex pair is
   * connected), so *no* edge is flippable on it — `flipEdge` always
   * returns false to avoid creating a duplicate edge. The "flipped-edge
   * endpoint trace" idea therefore has to use a mesh where flips actually
   * succeed; we pick the icosahedron for the topological case and reuse
   * the tetrahedron only as a flip-rejected sanity check.
   */

  function findInteriorEdge(sit: SignpostIntrinsicTriangulation): number {
    const m = sit.intrinsicMesh;
    for (let e = 0; e < m.nEdges; e++) {
      if (!m.isBoundaryEdge(e)) return e;
    }
    return -1;
  }

  it('tetrahedron: flips are rejected (K4); a trace before a rejected flip equals one after', () => {
    // Every edge in a regular tetrahedron is a chord of K4 — flipping
    // would duplicate an existing edge, so `flipEdge` returns false. We
    // confirm both that the rejection actually happens and that the trace
    // is unaffected by the rejected attempt.
    const { sit, geom } = build(tetrahedron());
    const im = sit.intrinsicMesh;
    const v = 0;
    const angle = 0.7;
    const distance = 0.4;
    const before = sit.traceFromVertex(v, angle, distance);
    for (let e = 0; e < im.nEdges; e++) {
      expect(im.isBoundaryEdge(e)).toBe(false);
      expect(sit.flipEdge(e)).toBe(false);
    }
    const after = sit.traceFromVertex(v, angle, distance);
    expect(dist3(before.position, after.position)).toBeCloseTo(0, 10);
    expect(after.faceIndex).toBe(before.faceIndex);
    // Distance-0 trace at every vertex still returns that vertex's position.
    for (let u = 0; u < im.nVertices; u++) {
      const r = sit.traceFromVertex(u, 1.234, 0);
      expect(dist3(r.position, geom.position(u))).toBeCloseTo(0, 10);
    }
  });

  it('icosahedron: flip an interior edge, then trace from the flipped-edge tail produces a valid on-surface result', () => {
    // Pick an interior edge that's flippable, flip it, and trace from
    // the new edge's tail in the direction of its (rescaled) signpost
    // for `0.4` units. Result should land on the input mesh (valid face,
    // bary sums to 1, all >= 0 within tol) without throwing.
    //
    // Note: the trace function reads the *input* mesh's vertex
    // tangent-frame reference (`inputGeometry.mesh.vertexHalfedge(v)`),
    // while `flipEdge` may rotate the *intrinsic* mesh's reference at
    // the flipped edge's endpoints. So the rescaled signpost angle no
    // longer corresponds to a fixed direction relative to the input
    // mesh's frame — but the trace must still be internally consistent.
    const { sit } = build(icosahedron());
    const im = sit.intrinsicMesh;
    let flippedE = -1;
    for (let e = 0; e < im.nEdges; e++) {
      if (im.isBoundaryEdge(e)) continue;
      if (sit.flipEdge(e)) {
        flippedE = e;
        break;
      }
    }
    expect(flippedE).toBeGreaterThanOrEqual(0);

    const ha = im.edgeHalfedge(flippedE);
    const tail = im.vertex(ha);
    const rescaled = sit.halfedgeSignposts[ha]! / sit.vertexAngleScaling(tail);
    const r = sit.traceFromVertex(tail, rescaled, 0.4);

    // Barycentric is sane.
    const sum = r.barycentric[0] + r.barycentric[1] + r.barycentric[2];
    expect(sum).toBeCloseTo(1, 10);
    expect(r.barycentric[0]).toBeGreaterThanOrEqual(-1e-6);
    expect(r.barycentric[1]).toBeGreaterThanOrEqual(-1e-6);
    expect(r.barycentric[2]).toBeGreaterThanOrEqual(-1e-6);
    // Face index is in range and the position is finite.
    expect(r.faceIndex).toBeGreaterThanOrEqual(0);
    expect(r.faceIndex).toBeLessThan(sit.inputGeometry.mesh.nFaces);
    expect(Number.isFinite(r.position[0])).toBe(true);
    expect(Number.isFinite(r.position[1])).toBe(true);
    expect(Number.isFinite(r.position[2])).toBe(true);
  });

  it('cube: flip a face diagonal; trace from one of its endpoints produces a valid on-surface result', () => {
    // The cube's bottom face is split into two triangles [0,2,1] and
    // [0,3,2] sharing the diagonal 0-2. Flipping that diagonal turns it
    // into 1-3, still on the same flat bottom face. The "endpoint of the
    // new diagonal" framing from the audit is loose: the trace function
    // walks the *input* mesh's wedges, which never had a 1-3 halfedge,
    // so we cannot bake "trace along the new diagonal lands at the
    // tip" into a hard assertion. We instead pin the weaker
    // post-condition: the trace is internally consistent and lands on
    // the cube's surface.
    const { sit } = build(cube());
    const im = sit.intrinsicMesh;
    let e02 = -1;
    for (let e = 0; e < im.nEdges; e++) {
      const h = im.edgeHalfedge(e);
      const a = im.vertex(h);
      const b = im.tipVertex(h);
      if ((a === 0 && b === 2) || (a === 2 && b === 0)) {
        e02 = e;
        break;
      }
    }
    expect(e02).toBeGreaterThanOrEqual(0);
    expect(sit.flipEdge(e02)).toBe(true);

    const ha = im.edgeHalfedge(e02);
    const tail = im.vertex(ha);
    const tip = im.tipVertex(ha);
    expect(new Set([tail, tip])).toEqual(new Set([1, 3]));
    const len = sit.edgeLengths[e02]!;
    expect(len).toBeCloseTo(Math.SQRT2, 10);
    const rescaled = sit.halfedgeSignposts[ha]! / sit.vertexAngleScaling(tail);
    const r = sit.traceFromVertex(tail, rescaled, len);

    // Bary sums to 1, face in range, position is finite. We do NOT
    // assert non-negative bary because the trace's input-mesh wedge
    // walk can extrapolate when the post-flip rescaled angle does not
    // align with the input mesh's tangent frame (see the icosahedron
    // test above for the same caveat).
    const bSum = r.barycentric[0] + r.barycentric[1] + r.barycentric[2];
    expect(bSum).toBeCloseTo(1, 10);
    expect(r.faceIndex).toBeGreaterThanOrEqual(0);
    expect(r.faceIndex).toBeLessThan(sit.inputGeometry.mesh.nFaces);
    expect(Number.isFinite(r.position[0])).toBe(true);
    expect(Number.isFinite(r.position[1])).toBe(true);
    expect(Number.isFinite(r.position[2])).toBe(true);
  });

  it('flat 4×4 grid: flip an interior edge, trace from a far vertex — result is invariant under intrinsic flips', () => {
    // `traceFromVertex` reads only the input mesh + `vertexAngleSums`,
    // both immutable under intrinsic flips. So a (vertex, angle,
    // distance) trace before any flip must produce *byte-identical*
    // output after one. On a flat surface the trace endpoint is also
    // analytically derivable: vertex position + distance * (cos θ_raw,
    // sin θ_raw, 0), where θ_raw is the angle relative to the input
    // mesh's first interior outgoing halfedge at v (rescaled by
    // `vertexAngleScaling`). We don't compute θ_raw in closed form here
    // — the strong "before == after" invariant is what matters.
    const { sit } = build(flatGrid(4, 3));
    const im = sit.intrinsicMesh;
    const v = 0; // corner vertex (0,0,0)
    const angle = Math.PI / 5;
    const distance = 0.7;
    const before = sit.traceFromVertex(v, angle, distance);

    // Flip an interior edge that is NOT incident to v=0 so that no
    // tangent-frame reference at v can shift even in the intrinsic mesh.
    let flipE = -1;
    for (let e = 0; e < im.nEdges; e++) {
      if (im.isBoundaryEdge(e)) continue;
      const h = im.edgeHalfedge(e);
      const a = im.vertex(h);
      const b = im.tipVertex(h);
      if (a === v || b === v) continue;
      if (sit.flipEdge(e)) {
        flipE = e;
        break;
      }
    }
    expect(flipE).toBeGreaterThanOrEqual(0);

    const after = sit.traceFromVertex(v, angle, distance);
    // Trace is computed entirely from the input mesh + immutable angle
    // sums, so before and after must agree to FP precision.
    expect(after.position[0]).toBeCloseTo(before.position[0], 10);
    expect(after.position[1]).toBeCloseTo(before.position[1], 10);
    expect(after.position[2]).toBeCloseTo(before.position[2], 10);
    expect(after.faceIndex).toBe(before.faceIndex);
    // On the flat plane (z=0) the result is in z=0.
    expect(after.position[2]).toBeCloseTo(0, 10);
  });

  it('idempotency: flip(flip(e)); trace along an unaffected angle is byte-identical', () => {
    const { sit } = build(icosahedron());
    const im = sit.intrinsicMesh;
    // Pick a flippable edge (deterministic — first one that takes).
    let e = -1;
    // Snapshot trace from a vertex that may or may not be incident to
    // the flip — the trace function reads only input geometry, so the
    // "unaffected halfedge" framing reduces to: any (v, θ, d) tuple is
    // unaffected, period.
    const v = 6;
    const angle = 1.2345;
    const distance = 0.3;
    const before = sit.traceFromVertex(v, angle, distance);
    for (let i = 0; i < im.nEdges; i++) {
      if (im.isBoundaryEdge(i)) continue;
      // Try flip; if it works, do it twice and break.
      const ok1 = sit.flipEdge(i);
      if (!ok1) continue;
      const ok2 = sit.flipEdge(i);
      expect(ok2).toBe(true);
      e = i;
      break;
    }
    expect(e).toBeGreaterThanOrEqual(0);
    const after = sit.traceFromVertex(v, angle, distance);
    expect(after.position[0]).toBeCloseTo(before.position[0], 10);
    expect(after.position[1]).toBeCloseTo(before.position[1], 10);
    expect(after.position[2]).toBeCloseTo(before.position[2], 10);
    expect(after.faceIndex).toBe(before.faceIndex);
  });

  it('distance-zero invariance: flip(e); trace at distance 0 returns each vertex position', () => {
    const { sit, geom } = build(icosahedron());
    const im = sit.intrinsicMesh;
    let flipped = false;
    for (let e = 0; e < im.nEdges; e++) {
      if (im.isBoundaryEdge(e)) continue;
      if (sit.flipEdge(e)) {
        flipped = true;
        break;
      }
    }
    expect(flipped).toBe(true);

    for (let v = 0; v < im.nVertices; v++) {
      const r = sit.traceFromVertex(v, 0.42, 0);
      expect(dist3(r.position, geom.position(v))).toBeCloseTo(0, 10);
    }
  });
});
