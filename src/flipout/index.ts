// Barrel for the L4 FlipOut layer. Public surface:
// `FlipEdgeNetwork` (the algorithm) plus the high-level `flipOutPath`
// convenience function and the `shortestEdgePath` Dijkstra helper.
//
// Ported from geometry-central:
//   include/geometrycentral/surface/flip_geodesics.h
//   src/surface/flip_geodesics.cpp
//   src/surface/mesh_graph_algorithms.cpp::shortestEdgePath
//
// Higher consumers (tests, CLI) import via:
//
//   import { FlipEdgeNetwork, flipOutPath } from '@chalkbag/flipout-ts/flipout';

export { FlipEdgeNetwork, flipOutPath, SegmentAngleType } from './flip-edge-network.js';
export { shortestEdgePath } from './dijkstra.js';
