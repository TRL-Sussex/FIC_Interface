// view3d.js — the robot in 3D, drawn in the browser.
//
// The Studio sends its robot's MuJoCo model as a small scene description
// (/api/scene) plus one binary buffer per mesh (/api/mesh?i=N), and the page
// poses the bodies from the joint values already in the state stream. The
// robot's PC renders nothing, and the same view works for the simulation and
// the real arm, because both fill the same joint values.
//
// MuJoCo is Z-up, so the camera's up is Z and the orbit maths follows.
import * as THREE from './three.module.js';

const DEG = Math.PI / 180;
let renderer = null;
let scene = null;
let camera = null;
let bodies = [];          // Object3D per body, in scene order
let data = null;          // the scene description
let orbit = { theta: -1.0, phi: 1.0, radius: 1.8, target: new THREE.Vector3(0, 0, 0.35) };
let pose = {};            // joint index -> angle: the arm and the gripper together
let frame = 0;
let host = null;
let gridHelper = null;
let maxFps = 60;          // the Dashboard asks for 30; the 3D page keeps the display rate
let lastDraw = 0;
const overlay = {};       // name -> three.js object, made in mount()
const OVERLAY_POINTS = 1500;

// ── camera ────────────────────────────────────────────────────────────────
function placeCamera() {
  const { theta, phi, radius, target } = orbit;
  camera.position.set(
    target.x + radius * Math.sin(phi) * Math.cos(theta),
    target.y + radius * Math.sin(phi) * Math.sin(theta),
    target.z + radius * Math.cos(phi));
  camera.up.set(0, 0, 1);
  camera.lookAt(target);
}

function addPointerControls(canvas) {
  let dragging = null;
  const pointers = new Map();
  canvas.style.touchAction = 'none';
  canvas.addEventListener('pointerdown', (e) => {
    canvas.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, e);
    dragging = { x: e.clientX, y: e.clientY };
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!pointers.has(e.pointerId)) return;
    pointers.set(e.pointerId, e);
    if (pointers.size === 2) {                     // pinch to zoom
      const [a, b] = [...pointers.values()];
      const dist = Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
      if (dragging.pinch) orbit.radius *= dragging.pinch / (dist || 1);
      dragging.pinch = dist;
      orbit.radius = Math.min(6, Math.max(0.35, orbit.radius));
      return;
    }
    if (!dragging) return;
    orbit.theta -= (e.clientX - dragging.x) * 0.008;
    orbit.phi = Math.min(Math.PI - 0.05, Math.max(0.05, orbit.phi - (e.clientY - dragging.y) * 0.008));
    dragging = { x: e.clientX, y: e.clientY };
  });
  const release = (e) => {
    pointers.delete(e.pointerId);
    if (pointers.size < 2 && dragging) delete dragging.pinch;
    if (pointers.size === 0) dragging = null;
  };
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    orbit.radius = Math.min(6, Math.max(0.35, orbit.radius * (1 + Math.sign(e.deltaY) * 0.12)));
  }, { passive: false });
}

// ── geometry ──────────────────────────────────────────────────────────────
function meshGeometry(buffer) {
  const head = new DataView(buffer, 0, 16);
  const magic = String.fromCharCode(head.getUint8(0), head.getUint8(1), head.getUint8(2), head.getUint8(3));
  if (magic !== 'FICM') throw new Error('not a mesh buffer');
  const nvert = head.getUint32(8, true);
  const nface = head.getUint32(12, true);
  const verts = new Float32Array(buffer, 16, nvert * 3);
  const faces = new Uint32Array(buffer, 16 + nvert * 12, nface * 3);
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
  geom.setIndex(new THREE.BufferAttribute(faces, 1));
  geom.computeVertexNormals();
  return geom;
}

// MuJoCo capsules and cylinders run along local Z; three.js builds them along Y.
function alongZ(geom) {
  geom.rotateX(Math.PI / 2);
  return geom;
}

function shapeFor(geom) {
  const s = geom.size;
  switch (geom.type) {
    case 'box': return new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    case 'sphere': return new THREE.SphereGeometry(s[0], 24, 16);
    case 'ellipsoid': {
      const g = new THREE.SphereGeometry(1, 24, 16);
      g.scale(s[0], s[1], s[2]);
      return g;
    }
    case 'capsule': return alongZ(new THREE.CapsuleGeometry(s[0], 2 * s[1], 6, 16));
    case 'cylinder': return alongZ(new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 24));
    default: return null;                    // plane and mesh handled by the caller
  }
}

async function buildRobot(scene3, sceneData) {
  const objects = sceneData.bodies.map(() => new THREE.Object3D());
  sceneData.bodies.forEach((body, i) => {
    if (body.parent >= 0) objects[body.parent].add(objects[i]);
    else scene3.add(objects[i]);
  });
  const meshCache = new Map();
  for (const g of sceneData.geoms) {
    let geometry = null;
    if (g.type === 'plane') continue;                       // the floor grid stands in
    if (g.type === 'mesh') {
      if (!meshCache.has(g.mesh)) {
        const resp = await fetch('/api/mesh?i=' + g.mesh + '&r=' + encodeURIComponent(data.robot || ''), { cache: 'force-cache' });
        if (!resp.ok) continue;
        meshCache.set(g.mesh, meshGeometry(await resp.arrayBuffer()));
      }
      geometry = meshCache.get(g.mesh);
    } else {
      geometry = shapeFor(g);
    }
    if (!geometry) continue;
    const colour = new THREE.Color(g.rgba[0], g.rgba[1], g.rgba[2]);
    const material = new THREE.MeshStandardMaterial({
      color: colour, metalness: 0.15, roughness: 0.65,
      transparent: g.rgba[3] < 1, opacity: g.rgba[3],
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(g.pos[0], g.pos[1], g.pos[2]);
    mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);   // w,x,y,z -> x,y,z,w
    objects[g.body].add(mesh);
  }
  return objects;
}

// ── posing ────────────────────────────────────────────────────────────────
// A body's joints move it relative to its parent, about each joint's own
// position and axis — the same rule MuJoCo uses (tested against it in
// test_web_scene.py, which mirrors this in Python).
function poseBodies(values) {
  const byBody = new Map();
  data.joints.forEach((j, n) => {
    if (!byBody.has(j.body)) byBody.set(j.body, []);
    byBody.get(j.body).push(n);
  });
  data.bodies.forEach((body, i) => {
    const obj = bodies[i];
    const pos = new THREE.Vector3(body.pos[0], body.pos[1], body.pos[2]);
    const quat = new THREE.Quaternion(body.quat[1], body.quat[2], body.quat[3], body.quat[0]);
    for (const n of byBody.get(i) || []) {
      const j = data.joints[n];
      const value = values[n] || 0;
      const axis = new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize();
      if (j.type === 'slide') {
        pos.add(axis.clone().multiplyScalar(value));
        continue;
      }
      const rot = new THREE.Quaternion().setFromAxisAngle(axis, value);
      const anchor = new THREE.Vector3(j.pos[0], j.pos[1], j.pos[2]);
      const offset = anchor.clone().sub(anchor.clone().applyQuaternion(rot));
      pos.add(offset.applyQuaternion(quat));
      quat.multiply(rot);
    }
    obj.position.copy(pos);
    obj.quaternion.copy(quat);
  });
}

// ── public ────────────────────────────────────────────────────────────────
export async function mount(container) {
  if (renderer) return true;
  const resp = await fetch('/api/scene');
  const body = await resp.json();
  if (!resp.ok || !body.ok) throw new Error((body && body.error) || 'no scene');
  data = body.scene;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x14161a);
  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  container.appendChild(renderer.domElement);
  host = container;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x404050, 2.0));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(1.5, 1.0, 2.5);
  scene.add(sun);
  const grid = new THREE.GridHelper(4, 20, 0x3a4150, 0x262b33);
  grid.rotateX(Math.PI / 2);                                 // Z-up floor
  scene.add(grid);
  gridHelper = grid;

  bodies = await buildRobot(scene, data);
  poseBodies({});
  addPointerControls(renderer.domElement);
  resize();
  makeOverlays();
  const tick = (now) => {
    frame = requestAnimationFrame(tick);
    if (document.hidden || now - lastDraw < 1000 / maxFps - 2) return;
    lastDraw = now;
    placeCamera();
    renderer.render(scene, camera);
  };
  tick();
  return true;
}

/** Re-colour the stage (the new web UI follows its light / dark theme). CSS colour strings. */
export function setPalette({ background, grid, gridSoft }) {
  if (!scene) return;
  scene.background = background ? new THREE.Color(background) : null;
  if (gridHelper) {
    gridHelper.material.dispose && gridHelper.material.dispose();
    scene.remove(gridHelper);
    gridHelper = new THREE.GridHelper(4, 20, new THREE.Color(grid), new THREE.Color(gridSoft || grid));
    gridHelper.rotateX(Math.PI / 2);
    scene.add(gridHelper);
  }
}

export function resize() {
  if (!renderer || !host) return;
  const w = host.clientWidth || 320;
  const h = host.clientHeight || 240;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

/** Pose the robot from the state stream's joint rows (degrees, robot order).
 *
 * `actual` is where the arm is: the controller's measurement on hardware, the
 * simulator's state in simulation. It is null until feedback arrives (a session
 * just started, the arm is not powered), and the commanded pose is a better
 * thing to draw then than a heap at zero.
 */
export function setJoints(rows) {
  if (!data || !rows) return;
  data.drive_joints.forEach((n, k) => {
    const row = rows[k];
    if (!row) return;
    const deg = typeof row.actual === 'number' ? row.actual
      : (typeof row.commanded === 'number' ? row.commanded : null);
    if (deg !== null) pose[n] = deg * DEG;
  });
  poseBodies(pose);
}

/** Open the fingers as far as the state stream says (the same maths as scene.py).
 *
 * `measured` is the arm's own reading of its fingers, so on hardware the view
 * shows where they really are; the slider is the fallback in simulation, where
 * nothing measures them.
 */
export function setGripper(g) {
  const spec = data && data.gripper;
  if (!spec || !g) return;
  let fraction = null;
  if (typeof g.measured === 'number' && g.max_width) fraction = g.measured / g.max_width;
  else if (typeof g.percent === 'number') fraction = g.percent / 100;
  if (fraction === null) return;
  fraction = Math.min(1, Math.max(0, fraction));
  const angle = spec.closed + fraction * (spec.open - spec.closed);
  pose[spec.joint] = angle;
  for (const c of spec.coupled) pose[c.joint] = angle * c.factor;
  poseBodies(pose);
}

// ── overlays (the Dashboard draws the path, target, zones and force with these) ──
function lineOverlay(name, colour) {
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(OVERLAY_POINTS * 3), 3));
  geom.setDrawRange(0, 0);
  const line = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: colour, depthTest: false, transparent: true }));
  line.renderOrder = 10; line.frustumCulled = false; line.visible = false;
  scene.add(line); overlay[name] = line;
}

function makeOverlays() {
  lineOverlay('trail', 0x1f6c9f);
  lineOverlay('path', 0x9b9a97);
  const marker = new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xc2410c, depthTest: false }));
  marker.renderOrder = 11; marker.visible = false; scene.add(marker); overlay.target = marker;
  const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)),
    new THREE.LineBasicMaterial({ color: 0x956400, depthTest: false, transparent: true, opacity: 0.8 }));
  box.renderOrder = 9; box.visible = false; scene.add(box); overlay.zones = box;
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.1, 0x9f2f2d, 0.04, 0.025);
  arrow.line.material.depthTest = false; arrow.cone.material.depthTest = false;
  arrow.renderOrder = 12; arrow.visible = false; scene.add(arrow); overlay.force = arrow;
}

/** Colours for the overlays, CSS strings: { trail, path, target, zones, force }. */
export function setOverlayColors(map) {
  for (const [k, v] of Object.entries(map || {})) {
    const o = overlay[k];
    if (!o || !v) continue;
    if (k === 'force') { o.setColor(new THREE.Color(v)); continue; }
    o.material.color.set(v);
  }
}

/** Draw a polyline from flat x,y,z values (the newest OVERLAY_POINTS are kept). Name: trail or path. */
export function setLine(name, flat) {
  const o = overlay[name];
  if (!o) return;
  const n = Math.min(OVERLAY_POINTS, Math.floor(flat.length / 3));
  const from = flat.length - n * 3;
  const attr = o.geometry.attributes.position;
  for (let i = 0; i < n * 3; i++) attr.array[i] = flat[from + i];
  attr.needsUpdate = true;
  o.geometry.setDrawRange(0, n);
  o.visible = n > 1 && o.userData.on !== false;
  o.userData.has = n > 1;
}

/** The target marker at p (x,y,z), or hidden when p is null. */
export function setTarget(p) {
  const o = overlay.target;
  if (!o) return;
  o.userData.has = !!p;
  if (p) o.position.set(p[0], p[1], p[2]);
  o.visible = !!p && o.userData.on !== false;
}

/** Safety-zone outline from {min:[x,y,z], max:[x,y,z]} or null. */
export function setZones(b) {
  const o = overlay.zones;
  if (!o) return;
  o.userData.has = !!b;
  if (b) {
    o.position.set((b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2);
    o.scale.set(Math.max(1e-3, b.max[0] - b.min[0]), Math.max(1e-3, b.max[1] - b.min[1]), Math.max(1e-3, b.max[2] - b.min[2]));
  }
  o.visible = !!b && o.userData.on !== false;
}

/** An arrow of `length` m from `origin` along `dir` (any length), or hidden when dir is null. */
export function setForce(origin, dir, length) {
  const o = overlay.force;
  if (!o) return;
  const d = dir ? new THREE.Vector3(dir[0], dir[1], dir[2]) : null;
  const ok = !!d && d.length() > 1e-9 && length > 1e-3;
  o.userData.has = ok;
  if (ok) {
    o.position.set(origin[0], origin[1], origin[2]);
    o.setDirection(d.normalize());
    o.setLength(length, Math.min(0.05, length * 0.4), Math.min(0.03, length * 0.25));
  }
  o.visible = ok && o.userData.on !== false;
}

/** Show or hide one overlay (trail, path, target, zones, force). Hidden ones keep their data. */
export function setOverlayVisible(name, on) {
  const o = overlay[name];
  if (!o) return;
  o.userData.on = !!on;
  o.visible = !!on && o.userData.has !== false && o.userData.has !== undefined;
}

/** Frame the camera: look at `target` [x,y,z] from `radius` m away (optionally angles, radians). */
export function setView(target, radius, theta, phi) {
  orbit.target.set(target[0], target[1], target[2]);
  if (radius) orbit.radius = radius;
  if (theta != null) orbit.theta = theta;
  if (phi != null) orbit.phi = phi;
}

/** Cap the redraw rate (frames per second); the Dashboard uses 30. */
export function setMaxFps(n) { maxFps = Math.max(1, Math.min(120, n || 60)); }

export function unmount() {
  if (frame) cancelAnimationFrame(frame);
  frame = 0;
  if (renderer) {
    renderer.dispose();
    if (renderer.domElement.parentNode) renderer.domElement.parentNode.removeChild(renderer.domElement);
  }
  renderer = scene = camera = data = host = gridHelper = null;
  for (const k of Object.keys(overlay)) delete overlay[k];
  maxFps = 60;
  bodies = [];
  pose = {};
}

export function ready() {
  return renderer !== null;
}
