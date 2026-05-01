// Barrel for the L2 extrinsic geometry layer. Public surface is a single
// class: `VertexPositionGeometry`, wrapping a `SurfaceMesh` (L1) and a
// per-vertex `Vec3[]` of positions.
//
// Ported from geometry-central's vertex-position-geometry namespace:
//   include/geometrycentral/surface/vertex_position_geometry.h
//   src/surface/vertex_position_geometry.cpp
//
// Higher layers (L3 signpost intrinsic triangulation, L4 FlipOut) consume
// this through:
//
//   import { VertexPositionGeometry } from 'flipout-ts/geometry';
//
// The class itself only depends on `src/math` and `src/mesh`; no Three.js.

export { VertexPositionGeometry } from './vertex-position-geometry.js';
