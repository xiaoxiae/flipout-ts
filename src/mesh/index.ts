// Barrel for the L1 half-edge mesh layer. The public surface is a single class
// (`SurfaceMesh`) plus the indexed-face triangle type and the `INVALID_INDEX`
// sentinel used in halfedge / face arrays.
//
// Higher layers (L2 geometry, L3 intrinsic triangulation) consume this through:
//
//   import { SurfaceMesh } from '@chalkbag/flipout-ts/mesh';
//
// or pull individual symbols from `./surface-mesh.js`.

export { SurfaceMesh, INVALID_INDEX, meshFromPositionsAndFaces } from './surface-mesh.js';
export type { Triangle } from './surface-mesh.js';
