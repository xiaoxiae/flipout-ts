/**
 * Polyline comparison helpers for geometry tests.
 *
 * The 3D polylines we compare against geometry-central's `getPathPolyline3D`
 * reference (`expected.pathPoints`) are samplings of the *same* geodesic
 * curve, but the two implementations emit different numbers of points (gc
 * inserts a vertex at every input-mesh face crossing; our sampling can differ
 * by a point or two at shared endpoints). A point-to-point comparison would
 * therefore overstate the error wherever one polyline has a vertex in the
 * middle of the other's segment. We instead use the symmetric Hausdorff
 * distance with point-to-segment distances, which measures how far the two
 * curves are as point sets — the right notion of "same curve, different
 * sampling".
 */

export type P3 = readonly [number, number, number];

/** Euclidean distance between two 3D points. */
export function dist3(a: P3, b: P3): number {
  const dx = a[0] - b[0];
  const dy = a[1] - b[1];
  const dz = a[2] - b[2];
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/** Bounding-box diagonal length of a point set (used to normalise errors). */
export function bboxDiagonal(points: readonly P3[]): number {
  if (points.length === 0) return 0;
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (const p of points) {
    if (p[0] < minX) minX = p[0];
    if (p[1] < minY) minY = p[1];
    if (p[2] < minZ) minZ = p[2];
    if (p[0] > maxX) maxX = p[0];
    if (p[1] > maxY) maxY = p[1];
    if (p[2] > maxZ) maxZ = p[2];
  }
  return Math.sqrt(
    (maxX - minX) ** 2 + (maxY - minY) ** 2 + (maxZ - minZ) ** 2,
  );
}

/** Total 3D arc length of a polyline (sum of consecutive segment lengths). */
export function polylineLength(poly: readonly P3[]): number {
  let total = 0;
  for (let i = 1; i < poly.length; i++) total += dist3(poly[i - 1]!, poly[i]!);
  return total;
}

/** Distance from point `p` to segment `[a, b]`. */
export function pointToSegmentDistance(p: P3, a: P3, b: P3): number {
  const abx = b[0] - a[0];
  const aby = b[1] - a[1];
  const abz = b[2] - a[2];
  const apx = p[0] - a[0];
  const apy = p[1] - a[1];
  const apz = p[2] - a[2];
  const abLen2 = abx * abx + aby * aby + abz * abz;
  let t = abLen2 > 0 ? (apx * abx + apy * aby + apz * abz) / abLen2 : 0;
  if (t < 0) t = 0;
  else if (t > 1) t = 1;
  const cx = a[0] + t * abx;
  const cy = a[1] + t * aby;
  const cz = a[2] + t * abz;
  const dx = p[0] - cx;
  const dy = p[1] - cy;
  const dz = p[2] - cz;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/** Minimum distance from point `p` to polyline `poly` (as connected segments). */
function pointToPolylineDistance(p: P3, poly: readonly P3[]): number {
  if (poly.length === 0) return Infinity;
  if (poly.length === 1) return dist3(p, poly[0]!);
  let best = Infinity;
  for (let i = 0; i + 1 < poly.length; i++) {
    const d = pointToSegmentDistance(p, poly[i]!, poly[i + 1]!);
    if (d < best) best = d;
  }
  return best;
}

/** Directed Hausdorff distance: max over `a in A` of dist(a, polyline B). */
export function directedHausdorff(A: readonly P3[], B: readonly P3[]): number {
  let worst = 0;
  for (const a of A) {
    const d = pointToPolylineDistance(a, B);
    if (d > worst) worst = d;
  }
  return worst;
}

/** Symmetric Hausdorff distance between two polylines (as curves). */
export function symmetricHausdorff(A: readonly P3[], B: readonly P3[]): number {
  return Math.max(directedHausdorff(A, B), directedHausdorff(B, A));
}
