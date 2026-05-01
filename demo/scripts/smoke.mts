// Programmatic smoke test for flipout-demo. Run with `npx tsx scripts/smoke.mts`.
//
// Loads the *baked* `public/teapot.json`, runs `flipOutPath` on a hardcoded
// (src, dst) pair, and asserts the resulting length matches the expected
// length from `flipout-ts/fixtures/teapot-far.json`. This validates that
// the alias resolution and end-to-end usage works at the Node level.
//
// The imports go through the same path aliases used by the Vite + tsc setup
// for the browser demo.

import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { SurfaceMesh, VertexPositionGeometry, SignpostIntrinsicTriangulation } from '../../flipout-ts/src/index.js';
import { flipOutPath, flipOutPathFromSurfacePoints } from '../../flipout-ts/src/flipout/index.js';
import type { SurfacePoint } from '../../flipout-ts/src/index.js';

const __dirname = dirname(fileURLToPath(import.meta.url));

const TEAPOT_JSON = resolve(__dirname, '../public/meshes/teapot.json');
const FIXTURE_JSON = resolve(__dirname, '../../flipout-ts/fixtures/teapot-far.json');

interface TeapotJson {
  vertices: [number, number, number][];
  faces: [number, number, number][];
}
interface FixtureJson {
  query: { src: number; dst: number };
  expected: { path_length: number };
}

const teapot = JSON.parse(readFileSync(TEAPOT_JSON, 'utf8')) as TeapotJson;
const fixture = JSON.parse(readFileSync(FIXTURE_JSON, 'utf8')) as FixtureJson;

const SRC = fixture.query.src;
const DST = fixture.query.dst;
const EXPECTED_LENGTH = fixture.expected.path_length;

console.log(`smoke: vertices=${teapot.vertices.length}, faces=${teapot.faces.length}`);
console.log(`smoke: query src=${SRC}, dst=${DST}`);
console.log(`smoke: expected path length (teapot-far.json): ${EXPECTED_LENGTH}`);

const mesh = SurfaceMesh.fromFaces(teapot.faces, teapot.vertices.length);
const geom = new VertexPositionGeometry(mesh, teapot.vertices);
const intrinsic = new SignpostIntrinsicTriangulation(geom);
const result = flipOutPath(intrinsic, SRC, DST);

console.log(
  `smoke: result length=${result.length}, iterations=${result.iterations}, converged=${result.converged}`,
);

const tolerance = 1e-4;
const diff = Math.abs(result.length - EXPECTED_LENGTH);
if (diff > tolerance) {
  console.error(`smoke: FAIL — length differs from fixture by ${diff} (> ${tolerance})`);
  process.exit(1);
}
if (!result.converged) {
  console.error('smoke: FAIL — flipOut did not converge');
  process.exit(1);
}
console.log('smoke: vertex-to-vertex OK');

// ----- Surface-point smoke: face center → face center on the teapot ---------
//
// Asserts (1) length is finite & positive, (2) length is below the teapot
// bbox diagonal × 5 (very generous upper bound), (3) flipOut converges.

const fSrc = 0;
const fDst = Math.floor(teapot.faces.length / 2);

function bboxDiagonal(verts: [number, number, number][]): number {
  let lo = [Infinity, Infinity, Infinity];
  let hi = [-Infinity, -Infinity, -Infinity];
  for (const v of verts) {
    for (let k = 0; k < 3; k++) {
      if (v[k] < lo[k]!) lo[k] = v[k];
      if (v[k] > hi[k]!) hi[k] = v[k];
    }
  }
  return Math.hypot(hi[0]! - lo[0]!, hi[1]! - lo[1]!, hi[2]! - lo[2]!);
}
const diag = bboxDiagonal(teapot.vertices);
const sanityUpperBound = diag * 5;

const meshSP = SurfaceMesh.fromFaces(teapot.faces, teapot.vertices.length);
const geomSP = new VertexPositionGeometry(meshSP, teapot.vertices);
const intrinsicSP = new SignpostIntrinsicTriangulation(geomSP);

const srcSP: SurfacePoint = { kind: 'face', face: fSrc, bary: [1 / 3, 1 / 3, 1 / 3] };
const dstSP: SurfacePoint = { kind: 'face', face: fDst, bary: [1 / 3, 1 / 3, 1 / 3] };
const resultSP = flipOutPathFromSurfacePoints(intrinsicSP, srcSP, dstSP);
console.log(
  `smoke[SP]: face ${fSrc} centroid → face ${fDst} centroid: length=${resultSP.length}, ` +
    `iterations=${resultSP.iterations}, converged=${resultSP.converged}`,
);
console.log(`smoke[SP]: bbox diagonal=${diag.toFixed(4)}, upper bound=${sanityUpperBound.toFixed(4)}`);

if (!Number.isFinite(resultSP.length) || resultSP.length <= 0) {
  console.error(`smoke[SP]: FAIL — non-finite or non-positive length: ${resultSP.length}`);
  process.exit(1);
}
if (resultSP.length > sanityUpperBound) {
  console.error(
    `smoke[SP]: FAIL — length ${resultSP.length} exceeds sanity upper bound ${sanityUpperBound}`,
  );
  process.exit(1);
}
if (!resultSP.converged) {
  console.error('smoke[SP]: FAIL — flipOut did not converge');
  process.exit(1);
}
console.log('smoke[SP]: OK');
console.log('smoke: ALL OK');
