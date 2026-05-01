// flipout-ts — TypeScript port of FlipOut (Sharp & Crane 2020) from geometry-central.
// Public API is assembled here as layers come online.

export type { Vec2, Vec3 } from './math/index.js';
export { Vec2Ops, Vec3Ops, orient2d, triangleArea, triangleAreaFromLengths } from './math/index.js';
export { SurfaceMesh, INVALID_INDEX } from './mesh/index.js';
export type { Triangle } from './mesh/index.js';
export { VertexPositionGeometry } from './geometry/index.js';
export { SignpostIntrinsicTriangulation, layoutTriangleVertex } from './intrinsic/index.js';
export type { TraceResult } from './intrinsic/index.js';
