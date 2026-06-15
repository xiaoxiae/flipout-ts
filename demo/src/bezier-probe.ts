// Standalone bezier visualization probe. Drives the SAME library code path as
// the demo's bezier mode (flipEdgeNetworkFromControlPath -> bezierSubdivide ->
// extractPolyline) with deterministic, farthest-point-sampled control vertices
// so the resulting curve is well spread across the mesh and any rendering bug
// is obvious. Renders the round-0 control geodesic (dim cyan) alongside the
// subdivided bezier (bright red + yellow points). Screenshot-friendly: sets
// `window.__probeReady = true` once a frame has been drawn.
//
// URL params:  ?mesh=teapot&k=4&rounds=2&yaw=0.0
//   mesh   bundled mesh name (teapot | icosphere | icosahedron | cube | grid)
//   k      number of control vertices (farthest-point sampled), default 4
//   rounds bezier subdivision rounds, default 2
//   seed   index of the first FPS seed vertex, default 0
//   yaw    extra camera yaw in radians, default 0

import * as THREE from 'three';
import {
  VertexPositionGeometry,
  SignpostIntrinsicTriangulation,
} from 'flipout-ts';
import { flipEdgeNetworkFromControlPath } from 'flipout-ts/flipout';
import { meshFromBufferGeometry, pathToBufferGeometry } from 'flipout-ts/three';

type MeshJson = { vertices: [number, number, number][]; faces: [number, number, number][] };

const params = new URLSearchParams(location.search);
const meshName = params.get('mesh') ?? 'teapot';
const k = Number.parseInt(params.get('k') ?? '4', 10);
const rounds = Number.parseInt(params.get('rounds') ?? '2', 10);
const seed = Number.parseInt(params.get('seed') ?? '0', 10);
const yaw = Number.parseFloat(params.get('yaw') ?? '0');

const info = document.getElementById('info') as HTMLDivElement;
function setInfo(s: string) { info.textContent = s; }

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(1);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x15151c);
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 1000);
scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dir = new THREE.DirectionalLight(0xffffff, 0.8);
dir.position.set(1, 1, 1);
scene.add(dir);
const dir2 = new THREE.DirectionalLight(0xffffff, 0.4);
dir2.position.set(-1, -0.5, -1);
scene.add(dir2);

function dist(a: number[], b: number[]) {
  const dx = a[0]! - b[0]!, dy = a[1]! - b[1]!, dz = a[2]! - b[2]!;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

// Farthest-point sampling of k control vertices, deterministic given `seed`.
function farthestPointSample(verts: number[][], count: number, first: number): number[] {
  const chosen = [first % verts.length];
  const minD = verts.map((v) => dist(v, verts[chosen[0]!]!));
  while (chosen.length < count) {
    let best = -1, bestD = -1;
    for (let i = 0; i < verts.length; i++) {
      if (minD[i]! > bestD) { bestD = minD[i]!; best = i; }
    }
    chosen.push(best);
    for (let i = 0; i < verts.length; i++) {
      const d = dist(verts[i]!, verts[best]!);
      if (d < minD[i]!) minD[i] = d;
    }
  }
  return chosen;
}

async function main() {
  const res = await fetch(`/meshes/${meshName}.json`);
  if (!res.ok) throw new Error(`failed to load ${meshName}: ${res.status}`);
  const data = (await res.json()) as MeshJson;

  const nV = data.vertices.length;
  const positions = new Float32Array(nV * 3);
  for (let i = 0; i < nV; i++) {
    positions[i * 3 + 0] = data.vertices[i]![0];
    positions[i * 3 + 1] = data.vertices[i]![1];
    positions[i * 3 + 2] = data.vertices[i]![2];
  }
  const nF = data.faces.length;
  const indices = new Uint32Array(nF * 3);
  for (let f = 0; f < nF; f++) {
    indices[f * 3 + 0] = data.faces[f]![0];
    indices[f * 3 + 1] = data.faces[f]![1];
    indices[f * 3 + 2] = data.faces[f]![2];
  }

  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setIndex(new THREE.BufferAttribute(indices, 1));
  geom.computeVertexNormals();
  geom.computeBoundingBox();

  // Build the library mesh exactly like the demo (welds duplicate vertices).
  // FPS / markers / control indices must all be in WELDED-vertex space.
  const { mesh: surfaceMesh, positions: weldedPos } = meshFromBufferGeometry(geom);
  const wpos = weldedPos as [number, number, number][];
  const nWV = wpos.length;
  const meshMat = new THREE.MeshStandardMaterial({
    color: 0x8a8aa0, roughness: 0.85, metalness: 0.0,
    transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false,
    polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
  });
  scene.add(new THREE.Mesh(geom, meshMat));
  // Wireframe overlay for surface reference.
  scene.add(new THREE.LineSegments(
    new THREE.WireframeGeometry(geom),
    new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.12 }),
  ));

  // Frame camera.
  const bb = geom.boundingBox!;
  const center = new THREE.Vector3(); bb.getCenter(center);
  const size = new THREE.Vector3(); bb.getSize(size);
  const diag = size.length();
  const off = new THREE.Vector3(Math.cos(yaw), 0.55, Math.sin(yaw)).normalize().multiplyScalar(diag * 1.7);
  camera.position.copy(center).add(off);
  camera.near = diag * 0.005; camera.far = diag * 100;
  camera.lookAt(center);
  camera.updateProjectionMatrix();

  // Pick control vertices (in welded-vertex space).
  const controls = farthestPointSample(wpos as number[][], Math.max(2, k), seed);

  // Control-vertex markers.
  const markerR = diag * 0.012;
  controls.forEach((v, i) => {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(markerR, 16, 12),
      new THREE.MeshBasicMaterial({ color: i === 0 || i === controls.length - 1 ? 0x33ff88 : 0x33aaff }),
    );
    m.position.set(wpos[v]![0], wpos[v]![1], wpos[v]![2]);
    scene.add(m);
  });

  function buildNet() {
    const g = new VertexPositionGeometry(surfaceMesh, wpos);
    const intr = new SignpostIntrinsicTriangulation(g);
    const net = flipEdgeNetworkFromControlPath(intr, controls, { markInterior: true });
    if (net === null) throw new Error('control vertices not connected by Dijkstra');
    return net;
  }

  let infoLines: string[] = [`mesh=${meshName}  V=${nWV} F=${nF}`, `controls=[${controls.join(', ')}]  rounds=${rounds}`];

  // Round-0 control geodesic (no subdivision) for reference.
  try {
    const net0 = buildNet();
    const poly0 = net0.extractPolyline();
    if (poly0.length >= 2) {
      const lg = pathToBufferGeometry(poly0);
      const l = new THREE.Line(lg, new THREE.LineBasicMaterial({ color: 0x00d0d0, depthTest: false }));
      l.renderOrder = 1; scene.add(l);
    }
    infoLines.push(`round-0 geodesic: ${poly0.length} pts  len=${net0.pathLength().toFixed(4)}`);
  } catch (e) {
    infoLines.push(`round-0 ERROR: ${(e as Error).message}`);
  }

  // Subdivided bezier.
  try {
    const net = buildNet();
    const t0 = performance.now();
    net.bezierSubdivide(rounds);
    const poly = net.extractPolyline();
    const ms = performance.now() - t0;
    if (poly.length >= 2) {
      const lg = pathToBufferGeometry(poly);
      const l = new THREE.Line(lg, new THREE.LineBasicMaterial({ color: 0xff3355, depthTest: false }));
      l.renderOrder = 2; scene.add(l);

      const flat = new Float32Array(poly.length * 3);
      for (let i = 0; i < poly.length; i++) {
        flat[i * 3] = poly[i]![0]; flat[i * 3 + 1] = poly[i]![1]; flat[i * 3 + 2] = poly[i]![2];
      }
      const pg = new THREE.BufferGeometry();
      pg.setAttribute('position', new THREE.BufferAttribute(flat, 3));
      const pts = new THREE.Points(pg, new THREE.PointsMaterial({
        color: 0xffd040, size: diag * 0.02, sizeAttenuation: true, depthTest: false,
      }));
      pts.renderOrder = 3; scene.add(pts);
    }
    infoLines.push(`bezier r${rounds}: ${poly.length} pts  len=${net.pathLength().toFixed(4)}  ${ms.toFixed(1)}ms`);
  } catch (e) {
    infoLines.push(`bezier ERROR: ${(e as Error).message}`);
  }

  setInfo(infoLines.join('\n'));

  let frames = 0;
  function loop() {
    renderer.render(scene, camera);
    frames++;
    if (frames < 3) requestAnimationFrame(loop);
    else (window as unknown as { __probeReady: boolean }).__probeReady = true;
  }
  loop();
}

main().catch((e) => {
  setInfo(`FATAL: ${(e as Error).message}\n${(e as Error).stack ?? ''}`);
  (window as unknown as { __probeReady: boolean }).__probeReady = true;
});
