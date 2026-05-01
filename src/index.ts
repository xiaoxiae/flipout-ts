// flipout-ts — TypeScript port of FlipOut (Sharp & Crane 2020) from
// geometry-central (https://github.com/nmwsharp/geometry-central, MIT).
//
// This barrel re-exports the public API. Each layer (math, mesh, geometry,
// intrinsic, flipout) corresponds to a geometry-central namespace and is
// itself a barrel. The `three/` subpath export lives in its own entry
// (`flipout-ts/three`) since it's the only module that depends on Three.js.

export type { Vec2, Vec3 } from './math/index.js';
export { Vec2Ops, Vec3Ops, orient2d, triangleArea, triangleAreaFromLengths } from './math/index.js';
export { SurfaceMesh, INVALID_INDEX } from './mesh/index.js';
export type { Triangle } from './mesh/index.js';
export { VertexPositionGeometry } from './geometry/index.js';
export { SignpostIntrinsicTriangulation, layoutTriangleVertex } from './intrinsic/index.js';
export type { TraceResult, SurfacePoint } from './intrinsic/index.js';
export {
  FlipEdgeNetwork,
  flipOutPath,
  flipOutPathFromSurfacePoints,
  SegmentAngleType,
  SNAP_EPS,
  shortestEdgePath,
} from './flipout/index.js';
