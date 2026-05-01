// Barrel for the L0 math layer. Re-exports the Vec2 / Vec3 types and all
// pure-function operators. Operators are exposed under namespace imports
// (`Vec2Ops`, `Vec3Ops`) since `add`/`sub`/etc. collide between vec2 and vec3.
// Higher layers can import the type directly:
//
//   import type { Vec3 } from '@chalkbag/flipout-ts/math';
//   import { Vec3Ops } from '@chalkbag/flipout-ts/math';
//
// or pull individual functions from the per-module entry points.

export type { Vec2 } from './vec2.js';
export type { Vec3 } from './vec3.js';

export * as Vec2Ops from './vec2.js';
export * as Vec3Ops from './vec3.js';

export {
  barycentricToCartesian,
  cartesianToBarycentric,
  cornerAngleFromLengths,
  triangleArea,
  triangleAreaFromLengths,
} from './triangle.js';

export { orient2d } from './predicates.js';
