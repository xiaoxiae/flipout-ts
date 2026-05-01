// FlipOut geodesic demo. Click two points on the mesh — anywhere, including
// face interiors and edge midpoints — to compute the geodesic between them.
// Surface clicks that land within ~5% of a mesh vertex are snapped to that
// vertex (cheaper to query than insert).
//
// Per click-pair the SignpostIntrinsicTriangulation is rebuilt from
// scratch — `flipOutPathFromSurfacePoints` inserts and flips, mutating it
// in place; reusing across runs would give wrong results.

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

import {
  SurfaceMesh,
  VertexPositionGeometry,
  SignpostIntrinsicTriangulation,
} from 'flipout-ts';
import type { SurfacePoint } from 'flipout-ts';
import { flipOutPathFromSurfacePoints } from 'flipout-ts/flipout';
import { meshFromBufferGeometry, pathToBufferGeometry } from 'flipout-ts/three';

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(0x1a1a22, 1);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 100);
camera.position.set(4, 4, 4);

const ambient = new THREE.AmbientLight(0xffffff, 0.4);
scene.add(ambient);
const dir = new THREE.DirectionalLight(0xffffff, 0.9);
dir.position.set(3, 5, 4);
scene.add(dir);
const dir2 = new THREE.DirectionalLight(0xffffff, 0.4);
dir2.position.set(-4, -2, -3);
scene.add(dir2);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// ---------------------------------------------------------------------------
// Mesh load
// ---------------------------------------------------------------------------

interface TeapotJson {
  vertices: number[][];
  faces: number[][];
}

let teapotMesh: THREE.Mesh | null = null;
let wireframeOverlay: THREE.LineSegments | null = null;
let surfaceMesh: SurfaceMesh | null = null;
let surfacePositions: [number, number, number][] | null = null;
/**
 * Map from "(a*nV + b*nV + c)" sorted-triple key → face index in the
 * SurfaceMesh. Built once per loaded mesh so picks (which give
 * (face.a, face.b, face.c) from Three.js) can be resolved to face indices
 * in O(1) via a string key.
 */
let triangleKeyToFaceIndex: Map<string, number> | null = null;
let bboxDiag = 1;

function triKey(a: number, b: number, c: number): string {
  const sorted = [a, b, c].sort((x, y) => x - y);
  return `${sorted[0]},${sorted[1]},${sorted[2]}`;
}

const teapotMaterial = new THREE.MeshStandardMaterial({
  color: 0x9aa3b8,
  roughness: 0.55,
  metalness: 0.05,
  flatShading: false,
});

const wireframeMaterial = new THREE.LineBasicMaterial({
  color: 0x1a1a22,
  transparent: true,
  opacity: 0.55,
});

async function loadMesh(name: string): Promise<void> {
  const res = await fetch(`meshes/${name}.json`);
  if (!res.ok) throw new Error(`failed to load meshes/${name}.json: ${res.status}`);
  const data = (await res.json()) as TeapotJson;

  const nV = data.vertices.length;
  const positions = new Float32Array(nV * 3);
  for (let i = 0; i < nV; i++) {
    const v = data.vertices[i]!;
    positions[i * 3 + 0] = v[0]!;
    positions[i * 3 + 1] = v[1]!;
    positions[i * 3 + 2] = v[2]!;
  }
  const nF = data.faces.length;
  const indices = new Uint32Array(nF * 3);
  for (let f = 0; f < nF; f++) {
    const tri = data.faces[f]!;
    indices[f * 3 + 0] = tri[0]!;
    indices[f * 3 + 1] = tri[1]!;
    indices[f * 3 + 2] = tri[2]!;
  }

  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setIndex(new THREE.BufferAttribute(indices, 1));
  geom.computeVertexNormals();
  geom.computeBoundingBox();

  installGeometry(geom);
}

/**
 * Take a populated `THREE.BufferGeometry` and install it as the active
 * mesh: replace any existing mesh, re-frame the camera, build the
 * flipout-ts `SurfaceMesh` + face-lookup, refresh wireframe overlay if
 * shown. Used by both the bundled-asset path (`loadMesh`) and the
 * user-OBJ-import path (`loadObjFile`).
 */
function installGeometry(geom: THREE.BufferGeometry): void {
  // Tear down previous mesh + markers/path before swapping.
  clearAll();
  if (teapotMesh !== null) {
    scene.remove(teapotMesh);
    teapotMesh.geometry.dispose();
    teapotMesh = null;
  }
  if (wireframeOverlay !== null) {
    scene.remove(wireframeOverlay);
    wireframeOverlay.geometry.dispose();
    wireframeOverlay = null;
  }

  if (geom.boundingBox === null) geom.computeBoundingBox();

  teapotMesh = new THREE.Mesh(geom, teapotMaterial);
  scene.add(teapotMesh);

  // Frame the camera on the bbox.
  const bb = geom.boundingBox!;
  const center = new THREE.Vector3();
  bb.getCenter(center);
  const size = new THREE.Vector3();
  bb.getSize(size);
  bboxDiag = size.length();
  controls.target.copy(center);
  const camOffset = new THREE.Vector3(1, 0.6, 1).normalize().multiplyScalar(bboxDiag * 1.6);
  camera.position.copy(center).add(camOffset);
  camera.near = bboxDiag * 0.005;
  camera.far = bboxDiag * 100;
  camera.updateProjectionMatrix();
  controls.update();

  // Build the flipout-ts SurfaceMesh once per loaded mesh. flipOutPath
  // mutates the intrinsic triangulation, so we keep only the immutable
  // connectivity + positions here — the SignpostIntrinsicTriangulation is
  // rebuilt per click-pair.
  const r = meshFromBufferGeometry(geom);
  surfaceMesh = r.mesh;
  surfacePositions = r.positions as [number, number, number][];

  // Build a (sorted vertex triple) -> face index lookup so we can map
  // Three.js raycaster hits (which expose vertex indices, not face indices)
  // back to flipout-ts face indices.
  triangleKeyToFaceIndex = new Map();
  for (let f = 0; f < surfaceMesh.nFaces; f++) {
    const it = surfaceMesh.verticesOfFace(f);
    const a = it.next().value as number;
    const b = it.next().value as number;
    const c = it.next().value as number;
    triangleKeyToFaceIndex.set(triKey(a, b, c), f);
  }

  applyWireframeToggle();
}

/**
 * Add or remove a `LineSegments` wireframe overlay over the active mesh
 * to match `wireframeChk.checked`. The overlay sits on top of the solid
 * surface (no depth conflict for clicks because the raycaster targets
 * the underlying mesh, not the lines).
 */
function applyWireframeToggle(): void {
  if (teapotMesh === null) return;
  const want = wireframeChk?.checked ?? false;
  if (want && wireframeOverlay === null) {
    const wf = new THREE.WireframeGeometry(teapotMesh.geometry);
    wireframeOverlay = new THREE.LineSegments(wf, wireframeMaterial);
    wireframeOverlay.renderOrder = 0.5;
    scene.add(wireframeOverlay);
  } else if (!want && wireframeOverlay !== null) {
    scene.remove(wireframeOverlay);
    wireframeOverlay.geometry.dispose();
    wireframeOverlay = null;
  }
}

/**
 * Parse a user-supplied OBJ file's text and install it. Uses Three.js's
 * `OBJLoader` to handle the parsing, then runs through the same
 * `meshFromBufferGeometry` welding pipeline as the bundled assets — so
 * non-indexed OBJ output is welded to a manifold mesh before flipout-ts
 * sees it. Throws on non-manifold input (FlipOut requires a manifold
 * triangle mesh); the caller surfaces the error to the user.
 */
function loadObjText(text: string, fileName: string): void {
  const loader = new OBJLoader();
  const group = loader.parse(text);
  let firstMesh: THREE.Mesh | null = null;
  group.traverse((obj) => {
    if (firstMesh === null && (obj as THREE.Mesh).isMesh) {
      firstMesh = obj as THREE.Mesh;
    }
  });
  if (firstMesh === null) {
    throw new Error(`OBJ "${fileName}" contains no mesh data.`);
  }
  const geom = (firstMesh as THREE.Mesh).geometry as THREE.BufferGeometry;
  // OBJLoader produces a non-indexed BufferGeometry; meshFromBufferGeometry
  // welds it. Make sure we have a position attribute.
  if (geom.getAttribute('position') === undefined) {
    throw new Error(`OBJ "${fileName}" has no position attribute.`);
  }
  geom.computeVertexNormals();
  geom.computeBoundingBox();
  installGeometry(geom);
}

// ---------------------------------------------------------------------------
// State machine + UI
// ---------------------------------------------------------------------------

type State = 'idle' | 'awaiting-dst' | 'done';
let state: State = 'idle';
let srcPoint: SurfacePoint | null = null;
let dstPoint: SurfacePoint | null = null;
let srcWorld: [number, number, number] | null = null;
let dstWorld: [number, number, number] | null = null;

const statusEl = document.getElementById('status') as HTMLParagraphElement;
const resetBtn = document.getElementById('reset') as HTMLButtonElement;
const meshSel = document.getElementById('mesh') as HTMLSelectElement;
const objButton = document.getElementById('obj-button') as HTMLButtonElement;
const objInput = document.getElementById('obj-input') as HTMLInputElement;
const wireframeChk = document.getElementById('wireframe') as HTMLInputElement;
const pathPointsChk = document.getElementById('path-points') as HTMLInputElement;
const animateBtn = document.getElementById('animate') as HTMLButtonElement;

// Animation state. `lastResultIters` is set to `result.iterations` after
// every successful runFlipOut; the Animate button uses it as the upper
// bound. `animationToken` is bumped on cancel (reset / new path) so a
// running animation can detect interruption and bail.
let lastResultIters: number | null = null;
let lastResultLength: number | null = null;
let animationToken = 0;
const ANIMATE_STEP_MS = 120;
// Each animated step's line stays in the scene as a ghost, fading out
// linearly over the next GHOST_FADE_STEPS frames so the eye can see the
// transformation as a continuous trail rather than a discrete jump.
// Ghosts are drawn in a muted color at lower starting opacity so the
// active (most recent) line stands out clearly above the trail.
const ACTIVE_LINE_COLOR = 0xff5577;
const GHOST_LINE_COLOR = 0xa46676;
const GHOST_INITIAL_OPACITY = 0.75;
const GHOST_FADE_STEPS = 8;
const ghostLines: { line: THREE.Line; mat: THREE.LineBasicMaterial }[] = [];

function disposeAllGhosts(): void {
  for (const g of ghostLines) {
    scene.remove(g.line);
    g.line.geometry.dispose();
    g.mat.dispose();
  }
  ghostLines.length = 0;
}

/**
 * Switch a `THREE.Line` from the active style (bright color, opacity 1)
 * to the ghost style (muted color, GHOST_INITIAL_OPACITY) so it can fade
 * out gracefully behind the new active line.
 */
function demoteToGhost(line: THREE.Line): void {
  const mat = line.material as THREE.LineBasicMaterial;
  mat.color.setHex(GHOST_LINE_COLOR);
  mat.transparent = true;
  mat.opacity = GHOST_INITIAL_OPACITY;
  mat.needsUpdate = true;
}

meshSel.addEventListener('change', () => {
  void loadMesh(meshSel.value).catch((err: unknown) => {
    setStatus(`Failed to load mesh: ${err instanceof Error ? err.message : String(err)}`);
    console.error(err);
  });
});

objButton.addEventListener('click', () => {
  objInput.click();
});

objInput.addEventListener('change', () => {
  const file = objInput.files?.[0];
  if (file === undefined) return;
  setStatus(`Loading ${file.name}...`);
  file
    .text()
    .then((text) => {
      loadObjText(text, file.name);
      setStatus(`Loaded ${file.name}. Click to set source.`);
    })
    .catch((err: unknown) => {
      setStatus(`Failed to load ${file.name}: ${err instanceof Error ? err.message : String(err)}`);
      console.error(err);
    })
    .finally(() => {
      // Reset so the same file can be re-selected.
      objInput.value = '';
    });
});

wireframeChk.addEventListener('change', () => {
  applyWireframeToggle();
});

pathPointsChk.addEventListener('change', () => {
  if (pathPoints !== null) pathPoints.visible = pathPointsChk.checked;
});

animateBtn.addEventListener('click', () => {
  void animateIterations();
});

/**
 * Re-run flipOutPathFromSurfacePoints with maxIterations=0,1,...,N
 * (where N is the iteration count of the current finalized path) and
 * step through the resulting polylines. The algorithm is deterministic
 * given a fresh intrinsic, so each k produces the same polyline as the
 * first k flips of the actual run.
 *
 * Cancelable via `animationToken`: any reset or new path bumps the
 * token and the running loop bails on its next iteration check.
 */
async function animateIterations(): Promise<void> {
  if (
    surfaceMesh === null ||
    surfacePositions === null ||
    srcPoint === null ||
    dstPoint === null ||
    lastResultIters === null ||
    pathLine === null ||
    pathPoints === null
  ) {
    return;
  }
  const finalIters = lastResultIters;
  const token = ++animationToken;
  setControlsEnabled(false);

  // Demote the existing finalized line to a ghost (muted color, lower
  // opacity) so it fades out as step 0 (the Dijkstra path) takes over.
  demoteToGhost(pathLine);
  ghostLines.push({
    line: pathLine,
    mat: pathLine.material as THREE.LineBasicMaterial,
  });
  pathLine = null;

  const sleep = (ms: number) => new Promise<void>((r) => setTimeout(r, ms));

  for (let k = 0; k <= finalIters; k++) {
    if (token !== animationToken) return;
    let result;
    try {
      const g = new VertexPositionGeometry(surfaceMesh, surfacePositions);
      const intr = new SignpostIntrinsicTriangulation(g);
      result = flipOutPathFromSurfacePoints(intr, srcPoint, dstPoint, {
        maxIterations: k,
      });
    } catch (err) {
      setStatus(
        `Animation aborted at step ${k}: ${err instanceof Error ? err.message : String(err)}`,
      );
      setControlsEnabled(true);
      return;
    }
    if (token !== animationToken) return;

    // Demote the previously-active line into the ghost list (muted
    // color, lower starting opacity), then fade existing ghosts.
    if (pathLine !== null) {
      demoteToGhost(pathLine);
      ghostLines.push({
        line: pathLine,
        mat: pathLine.material as THREE.LineBasicMaterial,
      });
      pathLine = null;
    }
    for (let i = ghostLines.length - 1; i >= 0; i--) {
      const g = ghostLines[i]!;
      // Skip the just-demoted one (already at GHOST_INITIAL_OPACITY); only
      // decay older ghosts. We detect by opacity being already higher than
      // its post-decay value of GHOST_INITIAL_OPACITY - 1/N.
      // Simpler: always decay; entries pushed this frame just got reset
      // to GHOST_INITIAL_OPACITY *after* this loop runs at the next step.
      g.mat.opacity = Math.max(0, g.mat.opacity - GHOST_INITIAL_OPACITY / GHOST_FADE_STEPS);
      if (g.mat.opacity <= 0.001) {
        scene.remove(g.line);
        g.line.geometry.dispose();
        g.mat.dispose();
        ghostLines.splice(i, 1);
      }
    }

    // Build the new step's line at full opacity, bright color — it's the
    // active path. It only becomes a ghost when the next step demotes it.
    const newLineGeom = pathToBufferGeometry(result.polyline);
    const newLineMat = new THREE.LineBasicMaterial({
      color: ACTIVE_LINE_COLOR,
      depthTest: false,
      transparent: true,
      opacity: 1.0,
      linewidth: 2,
    });
    const newLine = new THREE.Line(newLineGeom, newLineMat);
    newLine.renderOrder = 1;
    scene.add(newLine);
    pathLine = newLine;

    // Path points still swap instantly each step (a fading-points trail
    // would crowd the visual; the line ghost trail is the main effect).
    if (pathPoints !== null) {
      const flat = new Float32Array(result.polyline.length * 3);
      for (let i = 0; i < result.polyline.length; i++) {
        flat[i * 3 + 0] = result.polyline[i]![0];
        flat[i * 3 + 1] = result.polyline[i]![1];
        flat[i * 3 + 2] = result.polyline[i]![2];
      }
      const newPtsGeom = new THREE.BufferGeometry();
      newPtsGeom.setAttribute('position', new THREE.BufferAttribute(flat, 3));
      pathPoints.geometry.dispose();
      pathPoints.geometry = newPtsGeom;
    }

    setStatus(
      `Animating step ${k} / ${finalIters} — length=${result.length.toFixed(4)}`,
    );

    await sleep(ANIMATE_STEP_MS);
  }
  if (token !== animationToken) return;

  // Animation done: pathLine already holds the final-iteration line at
  // full opacity / active color. Just clear the trailing ghosts.
  disposeAllGhosts();

  setControlsEnabled(true);
  if (lastResultLength !== null) {
    setStatus(
      `Geodesic length: ${lastResultLength.toFixed(4)} (${finalIters} iters, ✓ converged). Click to reset.`,
    );
  }
}

function setControlsEnabled(enabled: boolean): void {
  meshSel.disabled = !enabled;
  objButton.disabled = !enabled;
  // Reset stays enabled so the user can cancel mid-animation.
  // animateBtn is special: only enabled when there's a path AND not animating.
  animateBtn.disabled = !enabled || lastResultIters === null;
}

function setStatus(msg: string): void {
  statusEl.textContent = msg;
}

// ---------------------------------------------------------------------------
// Markers + path line
// ---------------------------------------------------------------------------

let srcMarker: THREE.Mesh | null = null;
let dstMarker: THREE.Mesh | null = null;
let pathLine: THREE.Line | null = null;
let pathPoints: THREE.Points | null = null;

function makeMarker(color: number): THREE.Mesh {
  const geom = new THREE.SphereGeometry(bboxDiag * 0.005, 16, 16);
  const mat = new THREE.MeshBasicMaterial({ color });
  return new THREE.Mesh(geom, mat);
}

function disposeObject3D(obj: THREE.Object3D | null): void {
  if (obj === null) return;
  scene.remove(obj);
  obj.traverse((child) => {
    if ((child as THREE.Mesh).geometry !== undefined) {
      (child as THREE.Mesh).geometry?.dispose();
    }
    const mat = (child as THREE.Mesh).material;
    if (mat !== undefined) {
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
      else mat?.dispose();
    }
  });
}

function clearAll(): void {
  // Cancel any running animation and re-enable controls.
  animationToken++;
  disposeAllGhosts();
  disposeObject3D(srcMarker);
  disposeObject3D(dstMarker);
  disposeObject3D(pathLine);
  disposeObject3D(pathPoints);
  srcMarker = null;
  dstMarker = null;
  pathLine = null;
  pathPoints = null;
  srcPoint = null;
  dstPoint = null;
  srcWorld = null;
  dstWorld = null;
  lastResultIters = null;
  lastResultLength = null;
  state = 'idle';
  meshSel.disabled = false;
  objButton.disabled = false;
  animateBtn.disabled = true;
  setStatus('Click to set source.');
}

resetBtn.addEventListener('click', () => {
  clearAll();
});

// ---------------------------------------------------------------------------
// Pick handler
// ---------------------------------------------------------------------------

const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();

/** Threshold above which a barycentric coord triggers a snap-to-vertex. */
const VERTEX_SNAP_BARY = 0.95;

interface PickResult {
  point: SurfacePoint;
  worldPos: [number, number, number];
}

/**
 * Raycast the click into the mesh and return both a `SurfacePoint` and the
 * exact 3D hit position. Snaps to the nearest vertex if the max bary coord
 * exceeds {@link VERTEX_SNAP_BARY} (cheaper queries than a full insertion).
 */
function pickSurfacePoint(clientX: number, clientY: number): PickResult | null {
  if (teapotMesh === null || surfaceMesh === null || triangleKeyToFaceIndex === null) {
    return null;
  }
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  ndc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(ndc, camera);
  const hits = raycaster.intersectObject(teapotMesh, false);
  if (hits.length === 0) return null;
  const hit = hits[0]!;
  if (
    hit.face === null ||
    hit.face === undefined ||
    hit.barycoord === null ||
    hit.barycoord === undefined
  ) {
    return null;
  }
  const worldPos: [number, number, number] = [hit.point.x, hit.point.y, hit.point.z];
  const bary = hit.barycoord;
  const bx = bary.x;
  const by = bary.y;
  const bz = bary.z;

  // Snap to vertex if any bary coord is large enough.
  if (bx >= VERTEX_SNAP_BARY) {
    return { point: { kind: 'vertex', vertex: hit.face.a }, worldPos };
  }
  if (by >= VERTEX_SNAP_BARY) {
    return { point: { kind: 'vertex', vertex: hit.face.b }, worldPos };
  }
  if (bz >= VERTEX_SNAP_BARY) {
    return { point: { kind: 'vertex', vertex: hit.face.c }, worldPos };
  }

  // Otherwise: face-interior pick. Look up our SurfaceMesh face index.
  const fIdx = triangleKeyToFaceIndex.get(triKey(hit.face.a, hit.face.b, hit.face.c));
  if (fIdx === undefined) {
    // Mesh built on a different triangle indexing — fall back to corner snap.
    if (bx >= by && bx >= bz) return { point: { kind: 'vertex', vertex: hit.face.a }, worldPos };
    if (by >= bx && by >= bz) return { point: { kind: 'vertex', vertex: hit.face.b }, worldPos };
    return { point: { kind: 'vertex', vertex: hit.face.c }, worldPos };
  }

  // The barycentric order from Three.js is (a, b, c) which corresponds to
  // the *vertex order in the BufferGeometry's index*. flipout-ts's
  // `verticesOfFace(f)` returns vertices in CCW order — but the face
  // construction in `meshFromBufferGeometry` uses the same triangle list,
  // so corner ordering matches up.
  //
  // We need to map (a, b, c) bary coords to the order expected by
  // flipout-ts's `insertVertex_face` (which is the order from
  // `halfedgesAroundFace(f)`'s tail vertices).
  const surfaceM = surfaceMesh;
  const it = surfaceM.verticesOfFace(fIdx);
  const v0 = it.next().value as number;
  const v1 = it.next().value as number;
  const v2 = it.next().value as number;
  const baryByVertex = new Map<number, number>([
    [hit.face.a, bx],
    [hit.face.b, by],
    [hit.face.c, bz],
  ]);
  const bIn0 = baryByVertex.get(v0) ?? 0;
  const bIn1 = baryByVertex.get(v1) ?? 0;
  const bIn2 = baryByVertex.get(v2) ?? 0;
  return {
    point: { kind: 'face', face: fIdx, bary: [bIn0, bIn1, bIn2] },
    worldPos,
  };
}

// We want pick-on-click but not on drag. Track pointerdown position; only
// treat the release as a "click" if movement was small.
let downX = 0;
let downY = 0;
let downValid = false;
const CLICK_SLOP_PX = 5;

renderer.domElement.addEventListener('pointerdown', (e) => {
  downX = e.clientX;
  downY = e.clientY;
  downValid = true;
});

renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downValid) return;
  downValid = false;
  if (Math.hypot(e.clientX - downX, e.clientY - downY) > CLICK_SLOP_PX) return;
  handleClick(e.clientX, e.clientY);
});

function describePoint(p: SurfacePoint): string {
  if (p.kind === 'vertex') return `vertex ${p.vertex}`;
  if (p.kind === 'edge') return `edge ${p.edge} t=${p.t.toFixed(3)}`;
  const [b0, b1, b2] = p.bary;
  return `face ${p.face} bary [${b0.toFixed(2)}, ${b1.toFixed(2)}, ${b2.toFixed(2)}]`;
}

function pointsEqual(a: SurfacePoint, b: SurfacePoint): boolean {
  if (a.kind !== b.kind) return false;
  if (a.kind === 'vertex' && b.kind === 'vertex') return a.vertex === b.vertex;
  if (a.kind === 'edge' && b.kind === 'edge') return a.edge === b.edge && Math.abs(a.t - b.t) < 1e-9;
  if (a.kind === 'face' && b.kind === 'face') {
    return (
      a.face === b.face &&
      Math.abs(a.bary[0] - b.bary[0]) < 1e-9 &&
      Math.abs(a.bary[1] - b.bary[1]) < 1e-9 &&
      Math.abs(a.bary[2] - b.bary[2]) < 1e-9
    );
  }
  return false;
}

function handleClick(clientX: number, clientY: number): void {
  if (surfaceMesh === null || surfacePositions === null) return;

  if (state === 'done') {
    // Reset and start over with this click as the new source.
    clearAll();
  }

  const pick = pickSurfacePoint(clientX, clientY);
  if (pick === null) return;

  if (state === 'idle') {
    srcPoint = pick.point;
    srcWorld = pick.worldPos;
    srcMarker = makeMarker(0x55ff77);
    srcMarker.position.set(pick.worldPos[0], pick.worldPos[1], pick.worldPos[2]);
    scene.add(srcMarker);
    state = 'awaiting-dst';
    setStatus(`Source: ${describePoint(pick.point)}. Click to set destination.`);
    return;
  }

  if (state === 'awaiting-dst') {
    if (srcPoint !== null && pointsEqual(srcPoint, pick.point)) {
      setStatus(
        `Source: ${describePoint(srcPoint)}. Pick a *different* point for the destination.`,
      );
      return;
    }
    dstPoint = pick.point;
    dstWorld = pick.worldPos;
    dstMarker = makeMarker(0xff77aa);
    dstMarker.position.set(pick.worldPos[0], pick.worldPos[1], pick.worldPos[2]);
    scene.add(dstMarker);
    runFlipOut();
    state = 'done';
  }
}

// ---------------------------------------------------------------------------
// Run handler
// ---------------------------------------------------------------------------

function runFlipOut(): void {
  if (surfaceMesh === null || surfacePositions === null) return;
  if (srcPoint === null || dstPoint === null) return;

  setStatus(`Computing geodesic ${describePoint(srcPoint)} -> ${describePoint(dstPoint)}...`);

  let result;
  let computeMs = 0;
  try {
    // Rebuild intrinsic triangulation per click-pair (insertions + flips
    // mutate it in place, so reuse would corrupt subsequent queries).
    const t0 = performance.now();
    const geom = new VertexPositionGeometry(surfaceMesh, surfacePositions);
    const intrinsic = new SignpostIntrinsicTriangulation(geom);
    result = flipOutPathFromSurfacePoints(intrinsic, srcPoint, dstPoint);
    computeMs = performance.now() - t0;
  } catch (err) {
    setStatus(`Error: ${err instanceof Error ? err.message : String(err)}`);
    return;
  }

  const lineGeom = pathToBufferGeometry(result.polyline);
  const lineMat = new THREE.LineBasicMaterial({
    color: 0xff5577,
    depthTest: false,
    linewidth: 2,
  });
  pathLine = new THREE.Line(lineGeom, lineMat);
  pathLine.renderOrder = 1;
  scene.add(pathLine);

  // Points along the polyline (one per face crossing). Visibility toggled
  // by the "Path points" checkbox; built every run so toggling on later
  // shows the right set without needing to recompute.
  const pointsGeom = new THREE.BufferGeometry();
  const flatPts = new Float32Array(result.polyline.length * 3);
  for (let i = 0; i < result.polyline.length; i++) {
    flatPts[i * 3 + 0] = result.polyline[i]![0];
    flatPts[i * 3 + 1] = result.polyline[i]![1];
    flatPts[i * 3 + 2] = result.polyline[i]![2];
  }
  pointsGeom.setAttribute('position', new THREE.BufferAttribute(flatPts, 3));
  const pointsMat = new THREE.PointsMaterial({
    color: 0xffd060,
    size: bboxDiag * 0.008,
    sizeAttenuation: true,
    depthTest: false,
  });
  pathPoints = new THREE.Points(pointsGeom, pointsMat);
  pathPoints.renderOrder = 2;
  pathPoints.visible = pathPointsChk.checked;
  scene.add(pathPoints);

  const lengthStr = result.length.toFixed(4);
  const conv = result.converged ? '✓ converged' : '✗ did not converge';
  void srcWorld;
  void dstWorld;
  setStatus(
    `Geodesic length: ${lengthStr} (${result.iterations} iters, ${conv}, ${computeMs.toFixed(1)} ms). Click to reset.`,
  );

  // Enable Animate now that we have a finalized path.
  lastResultIters = result.iterations;
  lastResultLength = result.length;
  animateBtn.disabled = false;
}

// ---------------------------------------------------------------------------
// Render loop + resize
// ---------------------------------------------------------------------------

function animate(): void {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ---------------------------------------------------------------------------
// Go
// ---------------------------------------------------------------------------

loadMesh(meshSel.value)
  .then(() => {
    animate();
  })
  .catch((err: unknown) => {
    setStatus(`Failed to load mesh: ${err instanceof Error ? err.message : String(err)}`);
    console.error(err);
  });
