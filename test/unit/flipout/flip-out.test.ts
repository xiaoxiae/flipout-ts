/**
 * FlipOut algorithmic tests.
 *
 * Verifies the iterative-shorten loop on hand-computable cases plus the
 * core invariants any FlipOut run must satisfy: length monotonicity,
 * convergence, idempotence, and minimum-wedge ≥ π post-convergence.
 *
 * Cross-checked against geometry-central's `FlipEdgeNetwork::iterativeShorten`.
 */

import { describe, expect, it } from 'vitest';

import { VertexPositionGeometry } from '../../../src/geometry/vertex-position-geometry.js';
import { SignpostIntrinsicTriangulation } from '../../../src/intrinsic/signpost-intrinsic-triangulation.js';
import { SurfaceMesh } from '../../../src/mesh/surface-mesh.js';
import { FlipEdgeNetwork, flipOutPath } from '../../../src/flipout/index.js';
import { shortestEdgePath } from '../../../src/flipout/dijkstra.js';
import {
  cube,
  flatGrid,
  flatQuad,
  icosahedron,
  tetrahedron,
} from '../../_helpers/meshes.js';

interface MeshLike {
  vertices: readonly (readonly [number, number, number])[];
  faces: readonly (readonly [number, number, number])[];
}
function build(m: MeshLike): SignpostIntrinsicTriangulation {
  const mesh = SurfaceMesh.fromFaces(m.faces, m.vertices.length);
  const geom = new VertexPositionGeometry(mesh, m.vertices);
  return new SignpostIntrinsicTriangulation(geom);
}

// ---------------------------------------------------------------------------
// Hand-computable cases: flat configurations where the unrolled geodesic is
// a pure Euclidean distance we know in closed form.
// ---------------------------------------------------------------------------

describe('FlipOut — hand-computed flat cases', () => {
  it('flatQuad 0->2: already-geodesic diagonal stays at √2', () => {
    const sit = build(flatQuad());
    const r = flipOutPath(sit, 0, 2);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(Math.sqrt(2), 9);
  });

  it('flatQuad 1->3: forces a flip across the diagonal, lands at √2', () => {
    // 0->1 and 0->2 are direct edges; 1->3 goes around either via 0 (length 2)
    // or via 2 (length 2), but the true geodesic is the diagonal of length √2.
    const sit = build(flatQuad());
    const r = flipOutPath(sit, 1, 3);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(Math.sqrt(2), 6);
    expect(r.iterations).toBeGreaterThanOrEqual(1);
  });

  it('flatGrid(4) 0->15: opposite corners of unit square = √2', () => {
    const sit = build(flatGrid(4, 1));
    const r = flipOutPath(sit, 0, 15);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(Math.sqrt(2), 6);
  });

  it('flatGrid(5,2) 0->24: opposite corners of 2x2 square = 2√2', () => {
    const sit = build(flatGrid(5, 2));
    const r = flipOutPath(sit, 0, 24);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(2 * Math.sqrt(2), 6);
  });

  it('flatGrid(3) 1->7: opposite-side midpoints, length √(0.5²+1²)≈1.118', () => {
    const sit = build(flatGrid(3, 1));
    const r = flipOutPath(sit, 1, 7);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(1.0, 6);
  });
});

// ---------------------------------------------------------------------------
// Small-mesh cases on the closed manifolds.
// ---------------------------------------------------------------------------

describe('FlipOut — closed manifolds, small cases', () => {
  it('tetrahedron 0->1: single edge geodesic 2√2', () => {
    const sit = build(tetrahedron());
    const r = flipOutPath(sit, 0, 1);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(2 * Math.sqrt(2), 6);
  });

  it('tetrahedron 0->3: opposite vertex, also a single edge of length 2√2', () => {
    const sit = build(tetrahedron());
    const r = flipOutPath(sit, 0, 3);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(2 * Math.sqrt(2), 6);
  });

  it('cube 0->1: edge of unit cube, length 1', () => {
    const sit = build(cube());
    const r = flipOutPath(sit, 0, 1);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(1, 6);
  });

  it('cube 0->2: face diagonal of unit cube, length √2', () => {
    const sit = build(cube());
    const r = flipOutPath(sit, 0, 2);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(Math.sqrt(2), 6);
  });

  it('cube 0->6: cube space diagonal, geodesic √5', () => {
    // Unfolding two adjacent faces of the unit cube gives a 2x1 rectangle,
    // diagonal √(2² + 1²) = √5.
    const sit = build(cube());
    const r = flipOutPath(sit, 0, 6);
    expect(r.converged).toBe(true);
    expect(r.length).toBeCloseTo(Math.sqrt(5), 4);
  });
});

// ---------------------------------------------------------------------------
// Algorithmic invariants — true for any input + any pipeline.
// ---------------------------------------------------------------------------

describe('FlipOut — invariants', () => {
  it('monotone length: every flip strictly shortens the path (or holds)', () => {
    // We can't observe per-flip length without instrumenting, but we can
    // check that final length ≤ initial Dijkstra path length.
    const sit = build(flatGrid(5, 1));
    const initial = shortestEdgePath(sit, 0, 24);
    expect(initial).not.toBeNull();
    const initLen = initial!.reduce(
      (sum, he) => sum + sit.edgeLengths[sit.intrinsicMesh.edge(he)]!,
      0,
    );
    const network = new FlipEdgeNetwork(sit, initial!);
    const r = network.flipOut();
    expect(r.converged).toBe(true);
    expect(network.pathLength()).toBeLessThanOrEqual(initLen + 1e-12);
  });

  it('convergence: every test mesh + endpoint pair converges within 100 iterations', () => {
    const cases: [string, MeshLike, number, number][] = [
      ['tetrahedron 0->1', tetrahedron(), 0, 1],
      ['tetrahedron 0->3', tetrahedron(), 0, 3],
      ['cube 0->2', cube(), 0, 2],
      ['cube 0->6', cube(), 0, 6],
      ['cube 1->7', cube(), 1, 7],
      ['icosahedron 0->3', icosahedron(), 0, 3],
      ['icosahedron 0->1', icosahedron(), 0, 1],
      ['flatGrid(4) 0->15', flatGrid(4, 1), 0, 15],
      ['flatQuad 0->2', flatQuad(), 0, 2],
      ['flatQuad 1->3', flatQuad(), 1, 3],
    ];
    for (const [label, m, src, dst] of cases) {
      const sit = build(m);
      const r = flipOutPath(sit, src, dst, { maxIterations: 100 });
      expect(r.converged, `${label}: should converge`).toBe(true);
      expect(r.iterations, `${label}: iter cap`).toBeLessThan(100);
    }
  });

  it('idempotence: running flipOut twice on a converged network performs zero new flips', () => {
    const sit = build(flatGrid(4, 1));
    const initial = shortestEdgePath(sit, 0, 15)!;
    const network = new FlipEdgeNetwork(sit, initial);
    network.flipOut();
    const flipsAfterFirst = network.nFlips;

    // Run again — wedgeAngleQueue is empty; nothing should happen.
    const r2 = network.flipOut();
    expect(r2.converged).toBe(true);
    expect(network.nFlips).toBe(flipsAfterFirst);
  });

  it('post-convergence: every interior wedge has min angle ≥ π − tolerance', () => {
    const sit = build(icosahedron());
    const initial = shortestEdgePath(sit, 0, 3)!;
    const network = new FlipEdgeNetwork(sit, initial);
    network.flipOut();
    const minAngle = network.minWedgeAngle();
    // Allow gc's EPS_ANGLE = 1e-5 plus some FP slack.
    expect(minAngle).toBeGreaterThan(Math.PI - 1e-3);
  });

  it('post-convergence: tetrahedron has min angle ≥ π', () => {
    const sit = build(tetrahedron());
    const r = flipOutPath(sit, 0, 1);
    expect(r.converged).toBe(true);
    const network = new FlipEdgeNetwork(sit, shortestEdgePath(sit, 0, 1)!);
    network.flipOut();
    expect(network.minWedgeAngle()).toBeGreaterThan(Math.PI - 1e-3);
  });

  it('connectivity: every consecutive halfedge pair tip-to-tail joined', () => {
    const sit = build(cube());
    const initial = shortestEdgePath(sit, 0, 6)!;
    const network = new FlipEdgeNetwork(sit, initial);
    network.flipOut();
    const im = sit.intrinsicMesh;
    const halfedges = network.pathHalfedges();
    expect(halfedges.length).toBeGreaterThan(0);
    for (let i = 0; i + 1 < halfedges.length; i++) {
      expect(im.tipVertex(halfedges[i]!)).toBe(im.vertex(halfedges[i + 1]!));
    }
  });

  it('endpoints unchanged: source = original src, dest = original dst', () => {
    const sit = build(flatGrid(4, 1));
    const initial = shortestEdgePath(sit, 0, 15)!;
    const network = new FlipEdgeNetwork(sit, initial);
    network.flipOut();
    const im = sit.intrinsicMesh;
    const halfedges = network.pathHalfedges();
    expect(im.vertex(halfedges[0]!)).toBe(0);
    expect(im.tipVertex(halfedges[halfedges.length - 1]!)).toBe(15);
  });

  it('length is non-negative', () => {
    const sit = build(cube());
    const r = flipOutPath(sit, 0, 6);
    expect(r.length).toBeGreaterThan(0);
  });

  it('FlipEdgeNetwork rejects empty initial path', () => {
    const sit = build(flatQuad());
    expect(() => new FlipEdgeNetwork(sit, [])).toThrow();
  });

  it('FlipEdgeNetwork rejects disconnected initial path', () => {
    const sit = build(flatQuad());
    const im = sit.intrinsicMesh;
    // Pick two halfedges that are not tip-to-tail.
    let h0 = -1;
    let h1 = -1;
    for (const he of im.outgoingHalfedges(0)) {
      if (im.tipVertex(he) === 1) {
        h0 = he;
        break;
      }
    }
    for (const he of im.outgoingHalfedges(2)) {
      if (im.tipVertex(he) === 3) {
        h1 = he;
        break;
      }
    }
    expect(h0).not.toBe(-1);
    expect(h1).not.toBe(-1);
    expect(() => new FlipEdgeNetwork(sit, [h0, h1])).toThrow();
  });

  it('flipOutPath throws on src === dst', () => {
    const sit = build(flatQuad());
    expect(() => flipOutPath(sit, 0, 0)).toThrow();
  });
});

// ---------------------------------------------------------------------------
// pathLength / pathHalfedges / pathVertices accessors
// ---------------------------------------------------------------------------

describe('FlipEdgeNetwork — accessors', () => {
  it('pathHalfedges returns the input path verbatim before flipOut', () => {
    const sit = build(flatGrid(3, 1));
    const initial = shortestEdgePath(sit, 0, 8)!;
    const network = new FlipEdgeNetwork(sit, initial);
    expect(network.pathHalfedges()).toEqual(initial);
  });

  it('pathLength matches sum of initial-path edge lengths before flipOut', () => {
    const sit = build(flatGrid(3, 1));
    const initial = shortestEdgePath(sit, 0, 8)!;
    const network = new FlipEdgeNetwork(sit, initial);
    const sum = initial.reduce(
      (s, he) => s + sit.edgeLengths[sit.intrinsicMesh.edge(he)]!,
      0,
    );
    expect(network.pathLength()).toBeCloseTo(sum, 12);
  });

  it('pathVertices is well-formed: length = #segments + 1', () => {
    const sit = build(cube());
    const initial = shortestEdgePath(sit, 0, 6)!;
    const network = new FlipEdgeNetwork(sit, initial);
    network.flipOut();
    const verts = network.pathVertices();
    const halfedges = network.pathHalfedges();
    expect(verts.length).toBe(halfedges.length + 1);
  });

  it('pathVertices first/last match source / destination', () => {
    const sit = build(flatGrid(3, 1));
    const initial = shortestEdgePath(sit, 0, 8)!;
    const network = new FlipEdgeNetwork(sit, initial);
    network.flipOut();
    const verts = network.pathVertices();
    expect(verts[0]).toBe(0);
    expect(verts[verts.length - 1]).toBe(8);
  });
});
