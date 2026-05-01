// Bake demo mesh assets from `../../fixtures/*.json` into
// `public/meshes/<name>.json`. Each fixture stores a welded, manifold
// mesh in its `mesh` field; we extract just that and drop the FlipOut
// query/expected payloads. Run once during setup; the generated assets
// are committed into the demo so runtime needs no `trimesh` equivalent.

import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const FIXTURES = resolve(__dirname, '../../fixtures');
const OUT = resolve(__dirname, '../public/meshes');

// [demo name, source fixture]. Several fixtures share the same mesh
// (e.g. teapot-near/mid/far all carry the same body); pick any one.
const SOURCES = [
  ['teapot', 'teapot-far'],
  ['icosphere', 'icosphere-antipodal'],
  ['icosahedron', 'icosahedron-antipodal'],
  ['cube', 'cube-space-diagonal'],
  ['grid', 'grid-diagonal'],
];

mkdirSync(OUT, { recursive: true });

for (const [name, fixture] of SOURCES) {
  const src = resolve(FIXTURES, `${fixture}.json`);
  const raw = JSON.parse(readFileSync(src, 'utf8'));
  const { vertices, faces } = raw.mesh;
  if (!Array.isArray(vertices) || !Array.isArray(faces)) {
    throw new Error(`bake-meshes: ${fixture}.json mesh missing vertices/faces`);
  }
  const dst = resolve(OUT, `${name}.json`);
  writeFileSync(dst, JSON.stringify({ vertices, faces }));
  console.log(
    `bake-meshes: ${name.padEnd(12)} ← ${fixture.padEnd(24)} (${vertices.length} V, ${faces.length} F)`,
  );
}
