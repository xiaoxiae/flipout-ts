# flipout-ts demo

Interactive Three.js demo of the FlipOut geodesic algorithm. Live at
[flipout.slama.dev](https://flipout.slama.dev).

## Run locally

```bash
cd demo
npm install
npm run dev    # serves at http://localhost:5173
```

The dev server uses Vite with a source-level alias into `../src/`, so edits to
the library source hot-reload without rebuilding.

## Production build

```bash
npm run build
npm run preview
```

## What you can do

- Pick a mesh from the dropdown (teapot, icosphere, icosahedron, cube, flat
  grid).
- Click two points on the mesh to compute the geodesic between them.
- Drag to orbit, scroll to zoom.

## Smoke test

```bash
npx tsx scripts/smoke.mts
```

Verifies the library imports work end-to-end at the Node level — useful for CI.
