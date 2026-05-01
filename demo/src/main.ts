// FlipOut geodesic demo. Click two points on the mesh to compute the
// geodesic between the nearest mesh vertices.
//
// Per click-pair the SignpostIntrinsicTriangulation is rebuilt from
// scratch — flipOutPath mutates it in place, so reusing across runs would
// give wrong results.

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

import {
  SurfaceMesh,
  VertexPositionGeometry,
  SignpostIntrinsicTriangulation,
} from '@chalkbag/flipout-ts';
import { flipOutPath } from '@chalkbag/flipout-ts/flipout';
import { meshFromBufferGeometry, pathToBufferGeometry } from '@chalkbag/flipout-ts/three';

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
let surfaceMesh: SurfaceMesh | null = null;
let surfacePositions: [number, number, number][] | null = null;
let bboxDiag = 1;

const teapotMaterial = new THREE.MeshStandardMaterial({
  color: 0x9aa3b8,
  roughness: 0.55,
  metalness: 0.05,
  flatShading: false,
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

  // Tear down the previous mesh + any markers/path before swapping.
  clearAll();
  if (teapotMesh !== null) {
    scene.remove(teapotMesh);
    teapotMesh.geometry.dispose();
  }

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
}

// ---------------------------------------------------------------------------
// State machine + UI
// ---------------------------------------------------------------------------

type State = 'idle' | 'awaiting-dst' | 'done';
let state: State = 'idle';
let srcVertex = -1;
let dstVertex = -1;

const statusEl = document.getElementById('status') as HTMLParagraphElement;
const resetBtn = document.getElementById('reset') as HTMLButtonElement;
const meshSel = document.getElementById('mesh') as HTMLSelectElement;

meshSel.addEventListener('change', () => {
  void loadMesh(meshSel.value).catch((err: unknown) => {
    setStatus(`Failed to load mesh: ${err instanceof Error ? err.message : String(err)}`);
    console.error(err);
  });
});

function setStatus(msg: string): void {
  statusEl.textContent = msg;
}

// ---------------------------------------------------------------------------
// Markers + path line
// ---------------------------------------------------------------------------

let srcMarker: THREE.Mesh | null = null;
let dstMarker: THREE.Mesh | null = null;
let pathLine: THREE.Line | null = null;

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
  disposeObject3D(srcMarker);
  disposeObject3D(dstMarker);
  disposeObject3D(pathLine);
  srcMarker = null;
  dstMarker = null;
  pathLine = null;
  srcVertex = -1;
  dstVertex = -1;
  state = 'idle';
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

function pickVertex(clientX: number, clientY: number): number {
  if (teapotMesh === null) return -1;
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  ndc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(ndc, camera);
  const hits = raycaster.intersectObject(teapotMesh, false);
  if (hits.length === 0) return -1;
  const hit = hits[0]!;
  if (hit.face === null || hit.face === undefined || hit.barycoord === null || hit.barycoord === undefined) {
    return -1;
  }
  // Pick the corner with the highest barycentric coordinate.
  const bx = hit.barycoord.x;
  const by = hit.barycoord.y;
  const bz = hit.barycoord.z;
  if (bx >= by && bx >= bz) return hit.face.a;
  if (by >= bx && by >= bz) return hit.face.b;
  return hit.face.c;
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

function handleClick(clientX: number, clientY: number): void {
  if (surfaceMesh === null || surfacePositions === null) return;

  if (state === 'done') {
    // Reset and start over with this click as the new source.
    clearAll();
  }

  const v = pickVertex(clientX, clientY);
  if (v < 0) return;
  const pos = surfacePositions[v];
  if (pos === undefined) return;

  if (state === 'idle') {
    srcVertex = v;
    srcMarker = makeMarker(0x55ff77);
    srcMarker.position.set(pos[0], pos[1], pos[2]);
    scene.add(srcMarker);
    state = 'awaiting-dst';
    setStatus(`Source: v${v}. Click to set destination.`);
    return;
  }

  if (state === 'awaiting-dst') {
    if (v === srcVertex) {
      // Same vertex — no path. Keep waiting for a different destination.
      setStatus(`Source: v${v}. Pick a *different* vertex for the destination.`);
      return;
    }
    dstVertex = v;
    dstMarker = makeMarker(0xff77aa);
    dstMarker.position.set(pos[0], pos[1], pos[2]);
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
  if (srcVertex < 0 || dstVertex < 0) return;

  setStatus(`Computing geodesic v${srcVertex} -> v${dstVertex}...`);

  let result;
  try {
    // Rebuild intrinsic triangulation per click-pair (flipOutPath mutates).
    const geom = new VertexPositionGeometry(surfaceMesh, surfacePositions);
    const intrinsic = new SignpostIntrinsicTriangulation(geom);
    result = flipOutPath(intrinsic, srcVertex, dstVertex);
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

  const lengthStr = result.length.toFixed(4);
  const conv = result.converged ? '✓ converged' : '✗ did not converge';
  setStatus(
    `Geodesic length: ${lengthStr} (${result.iterations} iters, ${conv}). Click to reset.`,
  );
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
