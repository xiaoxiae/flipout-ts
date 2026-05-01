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
import { flipOutPath } from '../../flipout-ts/src/flipout/index.js';

const __dirname = dirname(fileURLToPath(import.meta.url));

const TEAPOT_JSON = resolve(__dirname, '../public/teapot.json');
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
console.log('smoke: OK');
