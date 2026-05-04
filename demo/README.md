# flipout-ts demo

Interactive Three.js demo of the FlipOut geodesic algorithm. Live at
[flipout-ts.slama.dev](https://flipout-ts.slama.dev).

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
- **Geodesic mode** (default): click two points on the mesh to compute the
  geodesic between them.
- **Bezier mode**: pick 3+ control vertices, drag the *Subdivisions* slider
  (0–3 rounds), and click *Compute* to render a geodesic Bezier curve through
  the controls. Picks anywhere on the surface snap to the nearest mesh vertex
  (`flipEdgeNetworkFromControlPath` requires vertex indices).
- Drag to orbit, scroll to zoom.

> **Bezier rendering caveat:** at higher subdivision rounds, segments between
> two intermediate midpoints can't be reconstructed in 3D — see the long
> comment on `extractPolyline` in `src/flipout/flip-edge-network.ts`. Such
> segments appear as straight chords through the surface. Visible mostly on
> meshes with non-degenerate input edges (teapot more than icosphere).

## Smoke test

```bash
npx tsx scripts/smoke.mts
```

Verifies the library imports work end-to-end at the Node level — useful for CI.
