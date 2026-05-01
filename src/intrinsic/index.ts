// Barrel for the L3 intrinsic-triangulation layer. Public surface:
// `SignpostIntrinsicTriangulation` (the main class) plus the small
// 2D layout helper `layoutTriangleVertex` and the `TraceResult` type.
//
// Ported from geometry-central:
//   include/geometrycentral/surface/signpost_intrinsic_triangulation.h
//   src/surface/signpost_intrinsic_triangulation.cpp
//   src/surface/intrinsic_triangulation.cpp
//
// Higher layers (L4 FlipOut) consume this through:
//
//   import { SignpostIntrinsicTriangulation } from '@chalkbag/flipout-ts/intrinsic';
//
// The class itself depends on `src/math`, `src/mesh`, and `src/geometry`;
// no Three.js.

export {
  SignpostIntrinsicTriangulation,
  layoutTriangleVertex,
} from './signpost-intrinsic-triangulation.js';
export type { TraceResult, SurfacePoint } from './signpost-intrinsic-triangulation.js';
