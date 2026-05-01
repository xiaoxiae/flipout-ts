// Extract the welded teapot body mesh (1601 V / 3160 F) from
// `../flipout-ts/fixtures/teapot-far.json` and write it as
// `public/teapot.json`. Run once during setup; the generated asset is
// committed into the demo so runtime needs no `trimesh`-equivalent.

import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const SRC = resolve(__dirname, '../../flipout-ts/fixtures/teapot-far.json');
const DST = resolve(__dirname, '../public/teapot.json');

const raw = JSON.parse(readFileSync(SRC, 'utf8'));
const { vertices, faces } = raw.mesh;

if (!Array.isArray(vertices) || !Array.isArray(faces)) {
  throw new Error('bake-teapot: teapot-far.json mesh missing vertices/faces');
}

mkdirSync(dirname(DST), { recursive: true });
writeFileSync(DST, JSON.stringify({ vertices, faces }));

console.log(
  `bake-teapot: wrote ${DST} (${vertices.length} vertices, ${faces.length} faces)`,
);
