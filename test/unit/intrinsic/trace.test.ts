/**
 * Trace-from-vertex tests for SignpostIntrinsicTriangulation.
 *
 * `traceFromVertex(v, angle, distance)` walks a tangent vector across the
 * input mesh from vertex v. The most direct test is to trace along an
 * existing intrinsic edge: starting at v in the direction of an outgoing
 * halfedge, distance = its length — should land at its tip vertex.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { cube, flatGrid, flatQuad, icosahedron, tetrahedron } from '../../_helpers/meshes.js';

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

const TWO_PI = 2 * Math.PI;

function dist3(a: readonly [number, number, number], b: readonly [number, number, number]): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

describe('traceFromVertex — distance-0 returns the vertex itself', () => {
  it.each<[string, MeshDataLike]>([
    ['flatQuad', flatQuad()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
  ])('%s: trace(v, angle, 0) = position(v)', (_, m) => {
    const { sit, geom } = build(m);
    for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
      const r = sit.traceFromVertex(v, 0.7, 0);
      expect(dist3(r.position, geom.position(v))).toBeCloseTo(0, 12);
    }
  });
});

describe('traceFromVertex — tracing along an existing intrinsic edge lands at its tip', () => {
  it.each<[string, MeshDataLike]>([
    ['flatQuad', flatQuad()],
    ['cube', cube()],
    ['icosahedron', icosahedron()],
    ['tetrahedron', tetrahedron()],
  ])('%s: trace(tail, signpost(he)/scaling*2π, edgeLength) lands within FP eps of tip', (_, m) => {
    const { sit, geom } = build(m);
    const mesh = sit.intrinsicMesh;
    for (let h = 0; h < mesh.nHalfedges; h++) {
      if (mesh.face(h) === -1) continue;
      const tail = mesh.vertex(h);
      const tip = mesh.tipVertex(h);
      const len = sit.edgeLengths[mesh.edge(h)]!;
      // The signpost angle is in [0, vertexAngleSum). Convert to the
      // "rescaled" tangent angle by dividing by vertexAngleScaling.
      const rescaledAngle =
        sit.halfedgeSignposts[h]! / sit.vertexAngleScaling(tail);
      const result = sit.traceFromVertex(tail, rescaledAngle, len);
      const expected = geom.position(tip);
      // Allow some tolerance: the trace projects through 2D layouts and
      // accumulates FP error. 1e-9 is generous for these short walks.
      expect(dist3(result.position, expected)).toBeLessThan(1e-9);
    }
  });
});

describe('traceFromVertex — flat quad, hand-computed trace', () => {
  it('flatQuad: trace from v0 along the halfedge with signpost 0 by 0.5 lands halfway along it', () => {
    // The "rescaled-angle 0" direction is whichever halfedge of v=0 has
    // signpost 0 (the FIRST in the iteration's interior wedge). With L1
    // aligned to gc's CCW convention, `vertexHalfedge(0)` is the first
    // INTERIOR outgoing halfedge in the CCW arc from the boundary — at
    // v=0 of the flatQuad that's the halfedge to v=1 (along +x). Tracing
    // distance 0.5 along it lands at (0.5, 0, 0).
    const { sit } = build(flatQuad());
    const result = sit.traceFromVertex(0, 0, 0.5);
    expect(result.position[0]).toBeCloseTo(0.5, 9);
    expect(result.position[1]).toBeCloseTo(0, 9);
    expect(result.position[2]).toBeCloseTo(0, 9);
  });

  it('flatQuad: trace from v0 along its diagonal lands at v=2 = (1,1,0)', () => {
    // The diagonal halfedge from v=0 to v=2 has length √2. We don't
    // hard-code its signpost angle (it depends on which CCW order L1
    // produces) — we just look it up directly and feed the rescaled
    // angle to `traceFromVertex`.
    const { sit } = build(flatQuad());
    const im = sit.intrinsicMesh;
    let heDiag = -1;
    for (const h of im.outgoingHalfedges(0)) {
      if (im.tipVertex(h) === 2) {
        heDiag = h;
        break;
      }
    }
    expect(heDiag).toBeGreaterThanOrEqual(0);
    const len = sit.edgeLengths[im.edge(heDiag)]!;
    expect(len).toBeCloseTo(Math.SQRT2, 12);
    const rescaled = sit.halfedgeSignposts[heDiag]! / sit.vertexAngleScaling(0);
    const result = sit.traceFromVertex(0, rescaled, len);
    expect(result.position[0]).toBeCloseTo(1, 9);
    expect(result.position[1]).toBeCloseTo(1, 9);
    expect(result.position[2]).toBeCloseTo(0, 9);
  });
});

describe('traceFromVertex — distance preserves geodesic length on flat surfaces', () => {
  it('flatGrid 5x5: 5 evenly-spaced traces from interior vertex are at equal distance', () => {
    const { sit, geom } = build(flatGrid(5, 4));
    // Pick the centre vertex (index 12 in a 5x5 grid, position (2, 2, 0)).
    const v = 12;
    const positions: [number, number, number][] = [];
    for (let i = 0; i < 5; i++) {
      const a = (i * TWO_PI) / 5;
      const r = sit.traceFromVertex(v, a, 1);
      positions.push([r.position[0], r.position[1], r.position[2]]);
    }
    const center = geom.position(v);
    // All distances from centre should be exactly 1.0 (this is a flat plane,
    // so the trace is just a straight line in 2D).
    for (const p of positions) {
      expect(dist3(p, center)).toBeCloseTo(1, 9);
    }
  });
});

describe('traceFromVertex — icosahedron (curved surface)', () => {
  it('icosahedron: 5 evenly-spaced traces from vertex 0 land at equal geodesic distance', () => {
    const { sit, geom } = build(icosahedron());
    // Vertex 0 has angle sum 5π/3. Trace 5 directions equally spaced.
    const positions: [number, number, number][] = [];
    for (let i = 0; i < 5; i++) {
      const a = (i * TWO_PI) / 5;
      const r = sit.traceFromVertex(0, a, 0.5);
      positions.push([r.position[0], r.position[1], r.position[2]]);
    }
    const center = geom.position(0);
    // All distances should be ≈ 0.5 (the trace walks geodesic length 0.5).
    // We use a coarser tolerance because traces cross multiple faces and
    // accumulate small FP errors at each edge transition.
    for (const p of positions) {
      expect(dist3(p, center)).toBeCloseTo(0.5, 7);
    }
  });

  it('icosahedron: trace at signpost angle of an outgoing halfedge lands precisely at tip', () => {
    const { sit, geom } = build(icosahedron());
    const m = sit.intrinsicMesh;
    for (let v = 0; v < m.nVertices; v++) {
      for (const h of m.outgoingHalfedges(v)) {
        if (m.face(h) === -1) continue;
        const len = sit.edgeLengths[m.edge(h)]!;
        const rescaled = sit.halfedgeSignposts[h]! / sit.vertexAngleScaling(v);
        const r = sit.traceFromVertex(v, rescaled, len);
        const tip = geom.position(m.tipVertex(h));
        expect(dist3(r.position, tip)).toBeLessThan(1e-9);
      }
    }
  });
});

describe('traceFromVertex — barycentric coordinates sum to 1', () => {
  it.each<[string, MeshDataLike]>([
    ['icosahedron', icosahedron()],
    ['flatGrid 4x4', flatGrid(4)],
  ])('%s: barycentric coords of every trace sum to 1', (_, m) => {
    const { sit } = build(m);
    const mesh = sit.intrinsicMesh;
    // Pick first interior vertex.
    let v = 0;
    while (v < mesh.nVertices && mesh.isBoundaryVertex(v)) v++;
    if (v === mesh.nVertices) return;
    for (let i = 0; i < 8; i++) {
      const a = (i * TWO_PI) / 8;
      const r = sit.traceFromVertex(v, a, 0.3);
      const sum = r.barycentric[0] + r.barycentric[1] + r.barycentric[2];
      // Sum is always 1 by construction.
      expect(sum).toBeCloseTo(1, 8);
    }
  });

  it('icosahedron: barycentric coords of every trace are non-negative', () => {
    // Curved-surface meshes don't have the "phantom angle wraparound"
    // pathology that affects flat-grid meshes when the rescaled frame's
    // CCW direction differs from the geometric CCW direction at a
    // vertex. On the icosahedron (interior vertices, scale != 1), traces
    // should always land inside their reported face.
    const { sit } = build(icosahedron());
    for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
      for (let i = 0; i < 6; i++) {
        const a = (i * TWO_PI) / 6;
        const r = sit.traceFromVertex(v, a, 0.4);
        // Allow tiny FP slack for points on edges.
        expect(r.barycentric[0]).toBeGreaterThanOrEqual(-1e-6);
        expect(r.barycentric[1]).toBeGreaterThanOrEqual(-1e-6);
        expect(r.barycentric[2]).toBeGreaterThanOrEqual(-1e-6);
      }
    }
  });
});

describe('traceFromVertex — face index is in valid range', () => {
  it.each<[string, MeshDataLike]>([
    ['flatQuad', flatQuad()],
    ['icosahedron', icosahedron()],
    ['cube', cube()],
  ])('%s: face index of trace result is a valid mesh face', (_, m) => {
    const { sit } = build(m);
    for (let v = 0; v < sit.intrinsicMesh.nVertices; v++) {
      for (let i = 0; i < 6; i++) {
        const a = (i * TWO_PI) / 6;
        const r = sit.traceFromVertex(v, a, 0.4);
        expect(r.faceIndex).toBeGreaterThanOrEqual(0);
        expect(r.faceIndex).toBeLessThan(sit.inputGeometry.mesh.nFaces);
      }
    }
  });
});

describe('post-flip trace correctness', () => {
  it('cube: tracing along a freshly-flipped intrinsic edge lands at its tip', () => {
    // The trace's wedge walk navigates around `v` on the *input* mesh and
    // reads wedge widths from the input geometry's corner angles. Pre-flip
    // the intrinsic and input meshes have identical connectivity / corner
    // angles, so the existing tests pass. After a flip, the intrinsic
    // mesh's halfedge fan around the new edge's endpoints diverges from
    // the input mesh's (a halfedge that didn't exist before now does, etc),
    // and the signpost stored on the new intrinsic halfedge is computed
    // from intrinsic corner angles. Tracing it must still land at the
    // tip — the signposts are designed so that following a halfedge's
    // signpost+length on the input mesh resolves the same surface point.
    const { sit, geom } = build(cube());
    const im = sit.intrinsicMesh;

    // Find a flippable interior edge incident to vertex 0. Pre-flip the
    // intrinsic and input meshes coincide, so any incident interior edge
    // of vertex 0 in `im` is a valid candidate.
    let edgeToFlip = -1;
    let heToFlip = -1;
    for (const he of im.outgoingHalfedges(0)) {
      const e = im.edge(he);
      if (im.isBoundaryEdge(e)) continue;
      // Tentatively pick it; we need the *current* (pre-flip) endpoints
      // and opposite vertices.
      heToFlip = he;
      edgeToFlip = e;
      break;
    }
    expect(edgeToFlip).toBeGreaterThanOrEqual(0);

    // Endpoints (a, b) and opposite face-vertices (c, d) BEFORE flipping.
    const a = im.vertex(heToFlip);
    const b = im.tipVertex(heToFlip);
    const c = im.vertex(im.next(im.next(heToFlip)));
    const twinHe = im.twin(heToFlip);
    const d = im.vertex(im.next(im.next(twinHe)));
    expect(a).toBe(0);
    expect(b).not.toBe(c);
    expect(b).not.toBe(d);

    // Perform the intrinsic flip. After this, edge `edgeToFlip` has
    // endpoints (c, d).
    const ok = sit.flipEdge(edgeToFlip);
    expect(ok).toBe(true);

    // Find the intrinsic halfedge of the new edge that goes from c -> d.
    let heNew = -1;
    for (const he of im.outgoingHalfedges(c)) {
      if (im.tipVertex(he) === d) {
        heNew = he;
        break;
      }
    }
    expect(heNew).toBeGreaterThanOrEqual(0);

    // Read the signpost and length stored on the new intrinsic halfedge.
    const signpost = sit.halfedgeSignposts[heNew]!;
    const lenCD = sit.edgeLengths[im.edge(heNew)]!;

    // Convert from "raw" signpost angle (in [0, vertexAngleSum[c])) to
    // the "rescaled" tangent-plane angle that traceFromVertex expects.
    const rescaled = signpost / sit.vertexAngleScaling(c);

    // The trace from c in this direction by `lenCD` should land at d
    // (the new edge's other endpoint, on the input mesh).
    const result = sit.traceFromVertex(c, rescaled, lenCD);
    const expected = geom.position(d);
    expect(dist3(result.position, expected)).toBeLessThan(1e-9);
  });
});
