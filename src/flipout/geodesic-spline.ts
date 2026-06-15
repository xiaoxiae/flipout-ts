// ============================================================================
// Geodesic splines — application-level curve schemes built on top of the
// FlipOut geodesic-Bezier primitive (`FlipEdgeNetwork.bezierSubdivide`).
//
// This is NOT a direct geometry-central port. It composes the ported
// primitives into a small family of piecewise curve schemes through a list of
// surface control points:
//
//   - 'bezier'       a single global geodesic Bezier (Morera et al. 2008) when
//                    the control path stays simple, falling back to a piecewise
//                    Bezier spline over short windows when it would self-touch
//                    (gc's bezierSubdivide requires a simple curve).
//   - 'catmull-rom'  a G1, *interpolating* geodesic Catmull-Rom spline: a cubic
//                    geodesic Bezier per consecutive control pair, with handle
//                    control points placed along the geodesic tangent derived
//                    from each knot's neighbours. Passes through every control
//                    point, corner-free, and inherently robust (each piece is a
//                    short, locally-simple segment).
//   - 'bspline'      a C1, *approximating* geodesic quadratic B-spline: a
//                    quadratic geodesic Bezier `[M(i-1), P(i), M(i)]` per
//                    interior control, where `M(i)` is the geodesic midpoint of
//                    consecutive controls. Stays near (not through) the controls
//                    and is smooth at the midpoint joints.
//
// Every piece is realised by building a fresh `SignpostIntrinsicTriangulation`
// (bezierSubdivide mutates it), running the geodesic Bezier, and extracting the
// on-surface polyline; pieces are concatenated with shared-endpoint dedup.
// ============================================================================

import type { Vec3 } from '../math/vec3.js';
import { cartesianToBarycentric } from '../math/triangle.js';
import { VertexPositionGeometry } from '../geometry/vertex-position-geometry.js';
import {
  SignpostIntrinsicTriangulation,
  type SurfacePoint,
} from '../intrinsic/signpost-intrinsic-triangulation.js';
import {
  BezierNonSimpleError,
  flipEdgeNetworkFromSurfacePointControlPath,
} from './flip-edge-network.js';

/** Control points per piece when falling back to a piecewise Bezier. */
const BEZIER_WINDOW = 4;
/** Endpoint dedup tolerance, relative to the geometry's scale. */
const JOIN_EPS_REL = 1e-7;

export type GeodesicSplineType = 'bezier' | 'catmull-rom' | 'bspline';

export interface GeodesicSplineOptions {
  /** Curve scheme. Default `'bezier'`. */
  type?: GeodesicSplineType;
  /** Subdivision rounds per geodesic-Bezier piece. Default 3. */
  rounds?: number;
  /** Catmull-Rom handle length as a fraction of segment length. Default 1/3. */
  tension?: number;
}

export interface GeodesicSplineResult {
  /** On-surface 3D polyline. */
  polyline: Vec3[];
  /** Sum of the pieces' intrinsic geodesic lengths. */
  length: number;
  /** Number of curve pieces concatenated. */
  pieces: number;
  /** True if a single global Bezier self-intersected and a fallback was used. */
  fellBack: boolean;
}

function isSelfIntersect(e: unknown): boolean {
  return e instanceof BezierNonSimpleError;
}

function dist3(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function bboxDiag(positions: readonly Vec3[]): number {
  let mnx = Infinity, mny = Infinity, mnz = Infinity;
  let mxx = -Infinity, mxy = -Infinity, mxz = -Infinity;
  for (const p of positions) {
    mnx = Math.min(mnx, p[0]); mny = Math.min(mny, p[1]); mnz = Math.min(mnz, p[2]);
    mxx = Math.max(mxx, p[0]); mxy = Math.max(mxy, p[1]); mxz = Math.max(mxz, p[2]);
  }
  return Math.hypot(mxx - mnx, mxy - mny, mxz - mnz);
}

/**
 * Build one geodesic-Bezier piece from a control-point list on a fresh
 * triangulation. Throws {@link BezierNonSimpleError} if the path
 * self-intersects. Returns null if the controls aren't Dijkstra-connected.
 */
function buildPiece(
  geom: VertexPositionGeometry,
  points: readonly SurfacePoint[],
  rounds: number,
): { polyline: Vec3[]; length: number } | null {
  const intrinsic = new SignpostIntrinsicTriangulation(geom);
  const net = flipEdgeNetworkFromSurfacePointControlPath(intrinsic, points, {
    markInterior: true,
  });
  if (net === null) return null;
  net.bezierSubdivide(rounds);
  return { polyline: net.extractPolyline(), length: net.pathLength() };
}

/**
 * Like {@link buildPiece} but retries with fewer subdivision rounds if the
 * piece self-intersects (`rounds = 0` is the straightened control polygon and
 * never trips the precondition), so it always returns a renderable piece.
 */
function buildPieceRobust(
  geom: VertexPositionGeometry,
  points: readonly SurfacePoint[],
  rounds: number,
): { polyline: Vec3[]; length: number } | null {
  for (let r = rounds; r >= 0; r--) {
    try {
      return buildPiece(geom, points, r);
    } catch (e) {
      if (!isSelfIntersect(e)) throw e;
    }
  }
  return null;
}

/** Concatenate piece polylines, dropping a duplicated shared endpoint. */
function concatPieces(parts: (Vec3[] | null)[], eps: number): Vec3[] {
  const out: Vec3[] = [];
  for (const part of parts) {
    if (part === null || part.length === 0) continue;
    let start = 0;
    if (out.length > 0 && dist3(out[out.length - 1]!, part[0]!) < eps) start = 1;
    for (let i = start; i < part.length; i++) out.push(part[i]!);
  }
  return out;
}

/** Partition controls into overlapping windows that share one endpoint. */
function bezierWindows(points: readonly SurfacePoint[]): SurfacePoint[][] {
  if (points.length <= BEZIER_WINDOW) return [points.slice()];
  const out: SurfacePoint[][] = [];
  let i = 0;
  while (i < points.length - 1) {
    const end = Math.min(i + BEZIER_WINDOW, points.length);
    out.push(points.slice(i, end));
    i = end - 1;
  }
  if (out.length > 1 && out[out.length - 1]!.length < 3) {
    const tail = out.pop()!;
    out[out.length - 1]!.push(...tail.slice(1));
  }
  return out;
}

// --- Catmull-Rom geodesic tangent + exp-map helpers --------------------------

/** Trace angle at the tail of `he` (input-frame for original tails, δ-frame
 *  for inserted tails) — i.e. the angle to feed `tracePolyline*`. */
function traceAngleAtTail(sit: SignpostIntrinsicTriangulation, he: number): number {
  const im = sit.intrinsicMesh;
  const tail = im.vertex(he);
  if (tail < sit.inputGeometry.mesh.nVertices) {
    return sit.halfedgeSignposts[he]! / sit.vertexAngleScaling(tail);
  }
  return sit.insertedTraceAngle(he);
}

/** Barycentric coordinates of 3D point `p` (assumed on input face `face`) in
 *  the canonical `halfedgesAroundFace` corner order. Robust to the tracer's
 *  own (sometimes frame-rotated) barycentric output. */
function baryInFace3D(sit: SignpostIntrinsicTriangulation, face: number, p: Vec3): Vec3 {
  const m = sit.inputGeometry.mesh;
  const it = m.halfedgesAroundFace(face);
  const A = sit.inputGeometry.position(m.vertex(it.next().value as number));
  const B = sit.inputGeometry.position(m.vertex(it.next().value as number));
  const C = sit.inputGeometry.position(m.vertex(it.next().value as number));
  return cartesianToBarycentric(p, A, B, C);
}

const SIMPLEX_EPS = 1e-6;
function inSimplex(b: Vec3): boolean {
  return b[0] >= -SIMPLEX_EPS && b[1] >= -SIMPLEX_EPS && b[2] >= -SIMPLEX_EPS &&
    b[0] <= 1 + SIMPLEX_EPS && b[1] <= 1 + SIMPLEX_EPS && b[2] <= 1 + SIMPLEX_EPS;
}

/**
 * Geodesic exp map: from intrinsic vertex `v` (at its input `SurfacePoint`),
 * walk `dist` along trace-angle `angle`; return the endpoint as an input-mesh
 * `SurfacePoint`. The barycentric is recomputed by 3D-projecting the reliable
 * trace endpoint into its face (the tracer's own barycentric ordering is not
 * trustworthy). If the exp ray would leave the mesh (e.g. across a boundary
 * rim), the endpoint lands off its face — we shorten the handle and retry, so
 * the handle always stays on the surface while preserving the tangent
 * direction (and thus G1 continuity at the knot).
 */
function expMap(
  sit: SignpostIntrinsicTriangulation,
  v: number,
  angle: number,
  dist: number,
): SurfacePoint {
  const inputNV = sit.inputGeometry.mesh.nVertices;
  const atStart = (): SurfacePoint =>
    v < inputNV ? { kind: 'vertex', vertex: v } : sit.insertedVertexLocations.get(v)!;
  if (dist <= 1e-12) return atStart();

  // Trace `dd` along the ray; return the endpoint SurfacePoint if it landed on
  // its face (stayed on the surface), else null.
  const tryDist = (dd: number): SurfacePoint | null => {
    const fs = { face: -1, bary: [0, 0, 0] as Vec3 };
    const poly = v < inputNV
      ? sit.tracePolylineFromVertex(v, angle, dd, fs)
      : sit.tracePolylineFromSurfacePoint(sit.insertedVertexLocations.get(v)!, angle, dd, fs);
    if (fs.face < 0) return null;
    const bary = baryInFace3D(sit, fs.face, poly[poly.length - 1]!);
    return inSimplex(bary) ? { kind: 'face', face: fs.face, bary: [bary[0], bary[1], bary[2]] } : null;
  };

  const full = tryDist(dist);
  if (full !== null) return full;

  // The exp ray left the mesh (e.g. across a boundary rim). Binary-search the
  // *longest* on-surface distance so the handle is clamped to the boundary
  // rather than collapsed to zero (which would cusp the cubic).
  let lo = 0, hi = dist, best: SurfacePoint | null = null;
  for (let i = 0; i < 14; i++) {
    const mid = 0.5 * (lo + hi);
    const r = tryDist(mid);
    if (r !== null) { best = r; lo = mid; } else hi = mid;
  }
  return best ?? atStart();
}

/** Circular mean of two angles. */
function circularMean(a: number, b: number): number {
  return Math.atan2(Math.sin(a) + Math.sin(b), Math.cos(a) + Math.cos(b));
}

/**
 * Compute the Catmull-Rom handle control point(s) at knot `p` given its
 * neighbours. Builds a single triangulation containing the (up to) three
 * controls so both incident geodesic directions share one tangent frame.
 * `aOut` is the outgoing handle (for the segment p→next); `bIn` is the
 * incoming handle (for the segment prev→p).
 */
function knotHandles(
  geom: VertexPositionGeometry,
  prev: SurfacePoint | null,
  p: SurfacePoint,
  next: SurfacePoint | null,
  tension: number,
): { aOut: SurfacePoint | null; bIn: SurfacePoint | null } {
  const controls = [prev, p, next].filter((x): x is SurfacePoint => x !== null);
  const sit = new SignpostIntrinsicTriangulation(geom);
  const net = flipEdgeNetworkFromSurfacePointControlPath(sit, controls, {
    markInterior: true,
  });
  if (net === null) return { aOut: null, bIn: null };
  net.bezierSubdivide(0); // straighten (pins controls); never recurses → safe

  const im = sit.intrinsicMesh;
  const hes = net.pathHalfedges();
  const vs = net.pathVertices();

  // Locate p's intrinsic vertex in the straightened path.
  let pIdx: number;
  if (prev && next) {
    pIdx = -1;
    for (let i = 1; i < vs.length - 1; i++) {
      if (net.isMarkedVertex(vs[i]!)) { pIdx = i; break; }
    }
    if (pIdx < 0) return { aOut: null, bIn: null };
  } else if (!prev) {
    pIdx = 0;
  } else {
    pIdx = vs.length - 1;
  }
  const pv = vs[pIdx]!;

  let thetaNext = NaN, thetaPrev = NaN, lenNext = 0, lenPrev = 0;
  for (let j = 0; j < hes.length; j++) {
    if (im.vertex(hes[j]!) === pv) {
      thetaNext = traceAngleAtTail(sit, hes[j]!);
      for (let k = j; k < hes.length; k++) lenNext += sit.edgeLengths[im.edge(hes[k]!)]!;
      break;
    }
  }
  for (let j = 0; j < hes.length; j++) {
    if (im.tipVertex(hes[j]!) === pv) {
      thetaPrev = traceAngleAtTail(sit, im.twin(hes[j]!));
      lenPrev = 0;
      for (let k = 0; k <= j; k++) lenPrev += sit.edgeLengths[im.edge(hes[k]!)]!;
    }
  }

  let tangent: number;
  if (prev && next) tangent = circularMean(thetaNext, thetaPrev + Math.PI);
  else if (!prev) tangent = thetaNext;
  else tangent = thetaPrev + Math.PI;

  const aOut = next ? expMap(sit, pv, tangent, lenNext * tension) : null;
  const bIn = prev ? expMap(sit, pv, tangent + Math.PI, lenPrev * tension) : null;
  return { aOut, bIn };
}

function catmullRom(
  geom: VertexPositionGeometry,
  points: readonly SurfacePoint[],
  rounds: number,
  tension: number,
  eps: number,
): GeodesicSplineResult {
  const n = points.length;
  const aOut: (SurfacePoint | null)[] = new Array(n).fill(null);
  const bIn: (SurfacePoint | null)[] = new Array(n).fill(null);
  for (let i = 0; i < n; i++) {
    const h = knotHandles(geom, i > 0 ? points[i - 1]! : null, points[i]!, i < n - 1 ? points[i + 1]! : null, tension);
    aOut[i] = h.aOut;
    bIn[i] = h.bIn;
  }

  const parts: (Vec3[] | null)[] = [];
  let length = 0;
  let count = 0;
  for (let i = 0; i < n - 1; i++) {
    // Cubic control polygon; drop handles that failed to resolve.
    const poly = [points[i]!, aOut[i], bIn[i + 1], points[i + 1]!].filter(
      (x): x is SurfacePoint => x !== null,
    );
    const piece = buildPieceRobust(geom, poly, rounds);
    if (piece) {
      parts.push(piece.polyline);
      length += piece.length;
      count++;
    }
  }
  return { polyline: concatPieces(parts, eps), length, pieces: count, fellBack: false };
}

/**
 * Geodesic midpoint of `a` and `b` as an input-mesh `SurfacePoint`: the
 * arc-length midpoint of the geodesic between them, obtained by one bezier
 * subdivision (which inserts and marks exactly that midpoint). Returns null if
 * the two points aren't Dijkstra-connected.
 */
function geodesicMidpoint(
  geom: VertexPositionGeometry,
  a: SurfacePoint,
  b: SurfacePoint,
): SurfacePoint | null {
  const sit = new SignpostIntrinsicTriangulation(geom);
  const net = flipEdgeNetworkFromSurfacePointControlPath(sit, [a, b], { markInterior: true });
  if (net === null) return null;
  net.bezierSubdivide(1);
  const vs = net.pathVertices();
  for (let i = 1; i < vs.length - 1; i++) {
    if (net.isMarkedVertex(vs[i]!)) {
      return sit.insertedVertexLocations.get(vs[i]!) ?? { kind: 'vertex', vertex: vs[i]! };
    }
  }
  return null;
}

function bspline(
  geom: VertexPositionGeometry,
  points: readonly SurfacePoint[],
  rounds: number,
  eps: number,
): GeodesicSplineResult {
  const n = points.length;
  if (n === 2) return bezierAdaptive(geom, points, rounds, eps);

  // Geodesic midpoints M[i] between consecutive controls P[i], P[i+1].
  const mid: (SurfacePoint | null)[] = [];
  for (let i = 0; i + 1 < n; i++) mid.push(geodesicMidpoint(geom, points[i]!, points[i + 1]!));

  // Quadratic B-spline pieces (C1 at the midpoint joints):
  //   [P0, M0], [M0, P1, M1], …, [M(n-3), P(n-2), M(n-2)], [M(n-2), P(n-1)].
  const pieces: (SurfacePoint | null)[][] = [];
  pieces.push([points[0]!, mid[0]!]);
  for (let i = 1; i < n - 1; i++) pieces.push([mid[i - 1]!, points[i]!, mid[i]!]);
  pieces.push([mid[n - 2]!, points[n - 1]!]);

  const parts: (Vec3[] | null)[] = [];
  let length = 0;
  let count = 0;
  for (const piece of pieces) {
    const clean = piece.filter((x): x is SurfacePoint => x !== null);
    if (clean.length < 2) continue;
    const built = buildPieceRobust(geom, clean, rounds);
    if (built) {
      parts.push(built.polyline);
      length += built.length;
      count++;
    }
  }
  return { polyline: concatPieces(parts, eps), length, pieces: count, fellBack: false };
}

function bezierAdaptive(
  geom: VertexPositionGeometry,
  points: readonly SurfacePoint[],
  rounds: number,
  eps: number,
): GeodesicSplineResult {
  // Try a single global Bezier first (smoothest — no joint corners).
  try {
    const piece = buildPiece(geom, points, rounds);
    if (piece === null) return { polyline: [], length: 0, pieces: 0, fellBack: false };
    return { polyline: piece.polyline, length: piece.length, pieces: 1, fellBack: false };
  } catch (e) {
    if (!isSelfIntersect(e)) throw e;
  }
  // Self-intersecting → piecewise Bezier over short windows.
  const parts: (Vec3[] | null)[] = [];
  let length = 0;
  let count = 0;
  for (const win of bezierWindows(points)) {
    const piece = buildPieceRobust(geom, win, rounds);
    if (piece) { parts.push(piece.polyline); length += piece.length; count++; }
  }
  return { polyline: concatPieces(parts, eps), length, pieces: count, fellBack: true };
}

/**
 * Build a geodesic spline through `controlPoints` on the surface described by
 * `geom`. See {@link GeodesicSplineType} for the available schemes. Builds
 * fresh triangulations internally, so callers pass the geometry (not an
 * intrinsic triangulation).
 */
export function geodesicSpline(
  geom: VertexPositionGeometry,
  controlPoints: readonly SurfacePoint[],
  options: GeodesicSplineOptions = {},
): GeodesicSplineResult {
  const { type = 'bezier', rounds = 3, tension = 1 / 3 } = options;
  if (controlPoints.length < 2) {
    throw new Error(`geodesicSpline: need ≥2 control points, got ${controlPoints.length}`);
  }
  const eps = JOIN_EPS_REL * Math.max(1, bboxDiag(geom.positions));
  if (type === 'catmull-rom') {
    return catmullRom(geom, controlPoints, rounds, tension, eps);
  }
  if (type === 'bspline') {
    return bspline(geom, controlPoints, rounds, eps);
  }
  return bezierAdaptive(geom, controlPoints, rounds, eps);
}
