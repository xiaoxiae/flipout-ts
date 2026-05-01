// Ported from geometry-central:
//   src/surface/mesh_graph_algorithms.cpp::shortestEdgePath
//   (used by `FlipEdgeNetwork::constructFromDijkstraPath` to seed an
//    initial edge-aligned path on the input mesh)
//
// L4 helper — single-source single-target Dijkstra over the *intrinsic*
// mesh's edge graph, using the intrinsic edge lengths as edge weights.
// Returns the sequence of intrinsic halfedges from `vSrc` to `vDst` along
// shortest edges. The FlipOut algorithm then iteratively shortens this
// path. A simple binary-heap priority queue is sufficient for our scale.

import type { SignpostIntrinsicTriangulation } from '../intrinsic/signpost-intrinsic-triangulation.js';

/**
 * Tiny min-heap of `(priority, value)` pairs. Values are arbitrary numbers
 * (we use halfedge indices). Not exported — internal to this module.
 *
 * Implementation note: the heap is keyed solely by priority; we accept
 * stale entries (vertices whose priority later improves) by re-inserting
 * and ignoring pops with priority above the recorded best on extraction.
 */
class MinHeap {
  private heap: { p: number; v: number }[] = [];

  size(): number {
    return this.heap.length;
  }

  push(p: number, v: number): void {
    this.heap.push({ p, v });
    this.siftUp(this.heap.length - 1);
  }

  pop(): { p: number; v: number } | undefined {
    if (this.heap.length === 0) return undefined;
    const top = this.heap[0]!;
    const last = this.heap.pop()!;
    if (this.heap.length > 0) {
      this.heap[0] = last;
      this.siftDown(0);
    }
    return top;
  }

  private siftUp(i: number): void {
    const h = this.heap;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (h[parent]!.p <= h[i]!.p) break;
      [h[parent], h[i]] = [h[i]!, h[parent]!];
      i = parent;
    }
  }

  private siftDown(i: number): void {
    const h = this.heap;
    const n = h.length;
    while (true) {
      const l = 2 * i + 1;
      const r = 2 * i + 2;
      let smallest = i;
      if (l < n && h[l]!.p < h[smallest]!.p) smallest = l;
      if (r < n && h[r]!.p < h[smallest]!.p) smallest = r;
      if (smallest === i) break;
      [h[smallest], h[i]] = [h[i]!, h[smallest]!];
      i = smallest;
    }
  }
}

/**
 * Mirrors gc's `shortestEdgePath(geom, startVert, endVert)`: Dijkstra over
 * the mesh's edge graph using `tri.edgeLengths` as weights. Returns the
 * sequence of *intrinsic* halfedges from `vSrc` to `vDst`, or `null` if
 * disconnected / `vSrc === vDst`.
 *
 * The path is reconstructed by walking back-pointers (`prevHe[v]` = the
 * halfedge whose tip is `v` along the shortest path).
 */
export function shortestEdgePath(
  tri: SignpostIntrinsicTriangulation,
  vSrc: number,
  vDst: number,
): number[] | null {
  if (vSrc === vDst) return null;

  const m = tri.intrinsicMesh;
  const nVerts = m.nVertices;
  if (vSrc < 0 || vSrc >= nVerts || vDst < 0 || vDst >= nVerts) {
    throw new RangeError(
      `shortestEdgePath: vSrc/vDst out of [0, ${nVerts}): ${vSrc} -> ${vDst}`,
    );
  }

  // Best-known distance to each vertex; +Inf until relaxed.
  const dist = new Float64Array(nVerts).fill(Number.POSITIVE_INFINITY);
  // Halfedge whose TIP is the corresponding vertex on the discovered SP
  // (i.e. the last edge of the shortest path FROM vSrc TO this vertex).
  const prevHe = new Int32Array(nVerts).fill(-1);

  dist[vSrc] = 0;
  const heap = new MinHeap();
  heap.push(0, vSrc);

  while (heap.size() > 0) {
    const top = heap.pop()!;
    const u = top.v;
    if (top.p > dist[u]!) continue; // stale
    if (u === vDst) break;
    for (const he of m.outgoingHalfedges(u)) {
      const w = m.tipVertex(he);
      const e = m.edge(he);
      const len = tri.edgeLengths[e]!;
      const newDist = dist[u]! + len;
      if (newDist < dist[w]!) {
        dist[w] = newDist;
        prevHe[w] = he;
        heap.push(newDist, w);
      }
    }
  }

  if (!Number.isFinite(dist[vDst]!)) return null;

  // Reconstruct halfedge sequence by walking back-pointers from vDst.
  const path: number[] = [];
  let curr = vDst;
  while (curr !== vSrc) {
    const he = prevHe[curr]!;
    if (he < 0) return null; // disconnected (shouldn't happen if dist finite)
    path.push(he);
    curr = m.vertex(he); // tail of `he`, i.e. the predecessor on the SP
  }
  path.reverse();
  return path;
}
