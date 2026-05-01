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

import {
  SurfaceMesh,
  VertexPositionGeometry,
  SignpostIntrinsicTriangulation,
} from '@chalkbag/flipout-ts';
import type { SurfacePoint } from '@chalkbag/flipout-ts';
import { flipOutPathFromSurfacePoints } from '@chalkbag/flipout-ts/flipout';
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
  srcPoint = null;
  dstPoint = null;
  srcWorld = null;
  dstWorld = null;
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
  try {
    // Rebuild intrinsic triangulation per click-pair (insertions + flips
    // mutate it in place, so reuse would corrupt subsequent queries).
    const geom = new VertexPositionGeometry(surfaceMesh, surfacePositions);
    const intrinsic = new SignpostIntrinsicTriangulation(geom);
    result = flipOutPathFromSurfacePoints(intrinsic, srcPoint, dstPoint);
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
  void srcWorld;
  void dstWorld;
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
