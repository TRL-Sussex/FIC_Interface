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
    shapeEd.press = pointers.size === 1 ? { x: e.clientX, y: e.clientY } : null;
    if (pointers.size === 1 && shapeEd.begin(canvas, e)) { dragging = null; return; }
    if (pointers.size === 1 && faceDrag.begin(canvas, e)) { dragging = null; return; }
    dragging = { x: e.clientX, y: e.clientY };
  });
  canvas.addEventListener('pointermove', (e) => {
    if (shapeEd.drag) { shapeEd.move(canvas, e); return; }
    if (faceDrag.active) { faceDrag.move(canvas, e); return; }
    if (!pointers.has(e.pointerId)) { faceDrag.hover(canvas, e); shapeEd.hover(canvas, e); return; }
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
    if (shapeEd.drag) shapeEd.end(canvas, e);
    else if (shapeEd.press && pointers.size === 1 && Math.hypot(e.clientX - shapeEd.press.x, e.clientY - shapeEd.press.y) < 6) shapeEd.click(canvas, e);
    shapeEd.press = null;
    if (faceDrag.active) faceDrag.end(canvas, e);
    pointers.delete(e.pointerId);
    if (pointers.size < 2 && dragging) delete dragging.pinch;
    if (pointers.size === 0) dragging = null;
  };
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  canvas.addEventListener('dblclick', () => shapeEd.finishDraw());
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    orbit.radius = Math.min(6, Math.max(0.35, orbit.radius * (1 + Math.sign(e.deltaY) * 0.12)));
  }, { passive: false });
}

// ── draggable safety-zone faces (the Safety page) ─────────────────────────
// A face moves along its own axis: the new value is the point on the axis line
// through the grabbed point that is closest to the pointer's ray.
const AXIS = { x: 0, y: 1, z: 2 };
const raycaster = new THREE.Raycaster();
const faceDrag = {
  cb: null, active: null,
  ray(canvas, e) {
    const r = canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    raycaster.setFromCamera(ndc, camera);
    return raycaster.ray;
  },
  pick(canvas, e) {
    const g = overlay.faces;
    if (!this.cb || !g || !g.visible || !camera) return null;
    this.ray(canvas, e);
    const hit = raycaster.intersectObjects(g.children.filter(m => m.visible), false)[0];
    return hit ? { face: hit.object.userData.face, point: hit.point.clone() } : null;
  },
  begin(canvas, e) {
    const hit = this.pick(canvas, e);
    if (!hit) return false;
    const a = AXIS[hit.face[0]];
    this.active = { face: hit.face, a, origin: hit.point, start: hit.point.getComponent(a), value: hit.point.getComponent(a) };
    canvas.style.cursor = 'grabbing';
    return true;
  },
  move(canvas, e) {
    const d = this.active, ray = this.ray(canvas, e);
    const u = new THREE.Vector3(); u.setComponent(d.a, 1);
    const w0 = d.origin.clone().sub(ray.origin);
    const b = u.dot(ray.direction), c = ray.direction.dot(ray.direction);
    const denom = c - b * b;
    if (denom < 1e-4) return;                       // looking straight along the axis
    const s = (b * ray.direction.dot(w0) - c * u.dot(w0)) / denom;
    d.value = d.start + s;
    this.cb(d.face, d.value, false);
  },
  end(canvas) {
    const d = this.active;
    this.active = null;
    canvas.style.cursor = '';
    if (d && this.cb) this.cb(d.face, d.value, true);
  },
  hover(canvas, e) {
    if (!this.cb) return;
    const hit = this.pick(canvas, e);
    canvas.style.cursor = hit ? 'grab' : '';
    const g = overlay.faces;
    if (g) for (const m of g.children) m.material.opacity = hit && m.userData.face === hit.face ? 0.3 : m.userData.alpha;
  },
};

function makeFaces() {
  const g = new THREE.Group();
  for (const face of ['x.min', 'x.max', 'y.min', 'y.max', 'z.min', 'z.max']) {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({
      color: 0x956400, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
    if (face[0] === 'x') m.rotation.y = Math.PI / 2;
    if (face[0] === 'y') m.rotation.x = Math.PI / 2;
    m.userData = { face, alpha: 0.08 };
    m.renderOrder = 8; m.visible = false;
    g.add(m);
  }
  g.visible = false;
  scene.add(g);
  overlay.faces = g;
}

/** Zone faces to drag: b = {min:[x,y,z], max:[x,y,z], on:[x?,y?,z?]} (only enabled axes get
 *  faces; the others span lo..hi of `span`), colours = {'x.min': css, ...}, edited = set of
 *  faces drawn stronger. null hides them. */
export function setZoneFaces(b, colours, edited) {
  const g = overlay.faces;
  if (!g) return;
  g.visible = !!b;
  if (!b) return;
  const span = [[-1, 1], [-1, 1], [0, 1.5]];
  const lo = [0, 1, 2].map(i => b.on[i] ? b.min[i] : span[i][0]);
  const hi = [0, 1, 2].map(i => b.on[i] ? b.max[i] : span[i][1]);
  const mid = [0, 1, 2].map(i => (lo[i] + hi[i]) / 2), size = [0, 1, 2].map(i => Math.max(1e-3, hi[i] - lo[i]));
  for (const m of g.children) {
    const face = m.userData.face, a = AXIS[face[0]], v = face.endsWith('min') ? lo[a] : hi[a];
    m.visible = !!b.on[a];
    const p = mid.slice(); p[a] = v;
    m.position.set(p[0], p[1], p[2]);
    if (a === 0) m.scale.set(size[2], size[1], 1);
    else if (a === 1) m.scale.set(size[0], size[2], 1);
    else m.scale.set(size[0], size[1], 1);
    if (colours && colours[face]) m.material.color.set(colours[face]);
    m.userData.alpha = edited && edited.has(face) ? 0.2 : 0.08;
    if (!(faceDrag.active && faceDrag.active.face === face)) m.material.opacity = m.userData.alpha;
  }
}

/** cb(face, value, done) while a zone face is dragged ('x.min' ... 'z.max'); null stops dragging. */
export function onZoneDrag(cb) { faceDrag.cb = cb || null; faceDrag.active = null; }


// ── whole-arm safety zones as shapes: drawn, picked, moved and resized here (the Safety page) ──
// A zone is the Studio's dict (fic_studio/safety_zones.py): box (centre, size, yaw_deg), sphere (centre,
// radius), cylinder (centre [x, y], radius, z_min, z_max) or prism (points [[x, y]...], z_min, z_max), with
// keep 'in' or 'out'. The page owns the list; this draws it, and reports what the operator does through
// onShapeZones: select(i), change(i, zone, done) while a handle is dragged, place(shape, [x, y]) for a click
// with a shape tool, drawn(points) when an outline is finished, drawing(n) as corners are added.
const shapeEd = {
  zones: [], sel: -1, tool: 'orbit', cb: null, sig: '', drawZ: 0, draw: [], drag: null, press: null,
  colours: { in: '#2f7d4f', out: '#b42318', off: '#8a8a8a', warn: '#b7791f', bad: '#b42318', handle: '#ffffff' },
  state: null, group: null, handles: null, preview: null, worst: null,
  ray(canvas, e) { return faceDrag.ray(canvas, e); },
  floor(canvas, e, z) {
    const ray = this.ray(canvas, e), p = new THREE.Vector3();
    return ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), -z), p) ? p : null;
  },
  pickHandle(canvas, e) {
    if (!this.handles || !this.handles.visible || !camera) return null;
    this.ray(canvas, e);
    const hit = raycaster.intersectObjects(this.handles.children, false)[0];
    return hit ? hit.object : null;
  },
  pickZone(canvas, e) {
    if (!this.group || !camera) return -1;
    this.ray(canvas, e);
    const hit = raycaster.intersectObjects(this.group.children.filter(m => m.userData.fill), false)[0];
    return hit ? hit.object.userData.index : -1;
  },
  begin(canvas, e) {
    if (!this.cb || this.tool !== 'orbit') return false;
    const hnd = this.pickHandle(canvas, e);
    if (!hnd) return false;
    const u = hnd.userData, z = JSON.parse(JSON.stringify(this.zones[this.sel]));
    const ray = this.ray(canvas, e);
    this.drag = { u, start: z, origin: hnd.position.clone(), at: u.plane != null ? this.floor(canvas, e, u.plane) : null, ray0: ray.clone() };
    canvas.style.cursor = 'grabbing';
    return true;
  },
  along(ray, o, u) {                       // how far along u (unit, through o) the pointer's ray passes closest
    const w0 = o.clone().sub(ray.origin), b = u.dot(ray.direction), c = ray.direction.dot(ray.direction);
    const denom = c - b * b;
    return denom < 1e-4 ? null : (b * ray.direction.dot(w0) - c * u.dot(w0)) / denom;
  },
  move(canvas, e) {
    const d = this.drag, u = d.u, z = JSON.parse(JSON.stringify(d.start));
    let p = null, s = null;
    if (u.plane != null) { p = this.floor(canvas, e, u.plane); if (!p || !d.at) return; }
    else { s = this.along(this.ray(canvas, e), d.origin, u.axis); if (s == null) return; }
    applyHandle(z, u, p, d.at, s);
    this.last = z;
    this.cb.change(this.sel, z, false);
  },
  end(canvas) {
    const last = this.last;
    this.drag = null; this.last = null;
    canvas.style.cursor = '';
    if (last && this.cb) this.cb.change(this.sel, last, true);
  },
  click(canvas, e) {
    if (!this.cb) return;
    if (this.tool === 'orbit') {
      if (faceDrag.active) return;
      this.cb.select(this.pickZone(canvas, e));
      return;
    }
    const p = this.floor(canvas, e, this.tool === 'draw' ? this.drawZ : 0);
    if (!p) return;
    if (this.tool === 'draw') {
      const pt = [p.x, p.y];
      if (this.draw.length >= 3 && Math.hypot(pt[0] - this.draw[0][0], pt[1] - this.draw[0][1]) < 0.03) { this.finishDraw(); return; }
      this.draw.push(pt);
      this.drawPreview(null);
      if (this.cb.drawing) this.cb.drawing(this.draw.length);
      return;
    }
    this.cb.place(this.tool, [p.x, p.y]);
  },
  finishDraw() {
    if (this.tool !== 'draw' || this.draw.length < 3 || !this.cb) return;
    const pts = this.draw;
    this.draw = [];
    this.drawPreview(null);
    this.cb.drawn(pts);
  },
  hover(canvas, e) {
    if (!this.cb) return;
    if (this.tool === 'draw') { const p = this.floor(canvas, e, this.drawZ); this.drawPreview(p ? [p.x, p.y] : null); canvas.style.cursor = 'crosshair'; return; }
    if (this.tool !== 'orbit') { canvas.style.cursor = 'copy'; return; }
    const hnd = this.pickHandle(canvas, e);
    if (hnd) canvas.style.cursor = 'grab';
    else if (canvas.style.cursor === 'crosshair' || canvas.style.cursor === 'copy') canvas.style.cursor = '';
  },
  drawPreview(cursor) {
    const line = this.preview;
    if (!line) return;
    const pts = this.draw.concat(cursor ? [cursor] : []);
    const attr = line.geometry.attributes.position;
    pts.forEach((q, k) => { attr.array[3 * k] = q[0]; attr.array[3 * k + 1] = q[1]; attr.array[3 * k + 2] = this.drawZ; });
    attr.needsUpdate = true;
    line.geometry.setDrawRange(0, pts.length);
    line.visible = this.tool === 'draw' && pts.length > 1;
    const dots = this.previewDots;
    dots.children.forEach((m, k) => { m.visible = this.tool === 'draw' && k < this.draw.length; if (m.visible) m.position.set(this.draw[k][0], this.draw[k][1], this.drawZ); });
  },
};

// Where a dragged handle puts its zone: p and at are the pointer's and the grab's points on the handle's
// horizontal plane; s the distance along its axis.
function applyHandle(z, u, p, at, s) {
  const k = u.kind;
  if (k === 'move') {
    const dx = p.x - at.x, dy = p.y - at.y;
    if (z.shape === 'prism') z.points = z.points.map(q => [q[0] + dx, q[1] + dy]);
    else { z.centre[0] += dx; z.centre[1] += dy; }
  } else if (k === 'lift') {
    z.centre[2] += s;
  } else if (k === 'size') {                     // a box face: that side moves, the opposite one stays
    const i = u.dim, size = Math.max(0.02, z.size[i] + s), grow = size - z.size[i];
    z.size[i] = size;
    const dir = u.axis;
    z.centre[0] += dir.x * grow / 2; z.centre[1] += dir.y * grow / 2; z.centre[2] += dir.z * grow / 2;
  } else if (k === 'yaw') {
    z.yaw_deg = Math.atan2(p.y - z.centre[1], p.x - z.centre[0]) * 180 / Math.PI - u.offset;
    while (z.yaw_deg > 180) z.yaw_deg -= 360;
    while (z.yaw_deg < -180) z.yaw_deg += 360;
  } else if (k === 'radius') {
    z.radius = Math.max(0.01, z.radius + s);
  } else if (k === 'top') {
    z.z_max = Math.max(z.z_min + 0.01, z.z_max + s);
  } else if (k === 'bottom') {
    z.z_min = Math.min(z.z_max - 0.01, z.z_min - s);
  } else if (k === 'corner') {
    z.points[u.corner] = [z.points[u.corner][0] + p.x - at.x, z.points[u.corner][1] + p.y - at.y];
  }
}

function zoneMeshes(z, i, col, selected) {
  const out = [];
  const fillMat = new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: z.enabled === false ? 0.03 : selected ? 0.2 : 0.1,
    side: THREE.DoubleSide, depthWrite: false });
  const lineMat = new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: z.enabled === false ? 0.35 : selected ? 1 : 0.75 });
  let geom = null;
  const place = (o) => {
    if (z.shape === 'box') { o.position.set(z.centre[0], z.centre[1], z.centre[2]); o.rotation.z = (z.yaw_deg || 0) * Math.PI / 180; }
    else if (z.shape === 'sphere') o.position.set(z.centre[0], z.centre[1], z.centre[2]);
    else if (z.shape === 'cylinder') o.position.set(z.centre[0], z.centre[1], (z.z_min + z.z_max) / 2);
    else o.position.set(0, 0, z.z_min);
  };
  if (z.shape === 'box') geom = new THREE.BoxGeometry(z.size[0], z.size[1], z.size[2]);
  else if (z.shape === 'sphere') geom = new THREE.SphereGeometry(z.radius, 32, 16);
  else if (z.shape === 'cylinder') geom = alongZ(new THREE.CylinderGeometry(z.radius, z.radius, z.z_max - z.z_min, 40));
  else {
    const shape = new THREE.Shape(z.points.map(q => new THREE.Vector2(q[0], q[1])));
    geom = new THREE.ExtrudeGeometry(shape, { depth: z.z_max - z.z_min, bevelEnabled: false });
  }
  const fill = new THREE.Mesh(geom, fillMat);
  fill.userData = { fill: true, index: i }; fill.renderOrder = 7; place(fill); out.push(fill);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geom, z.shape === 'sphere' ? 60 : 25), lineMat);
  edges.renderOrder = 8; place(edges); out.push(edges);
  return out;
}

function zoneHandles(z) {
  const H = [];
  const add = (kind, pos, extra) => H.push(Object.assign({ kind, pos }, extra || {}));
  const Z = new THREE.Vector3(0, 0, 1);
  if (z.shape === 'box') {
    const c = new THREE.Vector3(...z.centre), yaw = (z.yaw_deg || 0) * Math.PI / 180;
    const ax = [new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0), new THREE.Vector3(-Math.sin(yaw), Math.cos(yaw), 0), Z.clone()];
    add('move', c.clone(), { plane: c.z });
    add('lift', c.clone().addScaledVector(Z, z.size[2] / 2 + 0.07), { axis: Z.clone() });
    for (let i = 0; i < 3; i++) for (const sgn of [1, -1]) {
      const dir = ax[i].clone().multiplyScalar(sgn);
      add('size', c.clone().addScaledVector(dir, z.size[i] / 2), { axis: dir, dim: i });
    }
    const yh = c.clone().addScaledVector(ax[0], z.size[0] / 2 + 0.08).addScaledVector(Z, z.size[2] / 2);
    add('yaw', yh, { plane: yh.z, offset: 0 });
  } else if (z.shape === 'sphere') {
    const c = new THREE.Vector3(...z.centre);
    add('move', c.clone(), { plane: c.z });
    add('lift', c.clone().addScaledVector(Z, z.radius + 0.07), { axis: Z.clone() });
    add('radius', c.clone().add(new THREE.Vector3(z.radius, 0, 0)), { axis: new THREE.Vector3(1, 0, 0) });
  } else {
    let cx, cy;
    if (z.shape === 'cylinder') { cx = z.centre[0]; cy = z.centre[1]; }
    else { cx = z.points.reduce((a, q) => a + q[0], 0) / z.points.length; cy = z.points.reduce((a, q) => a + q[1], 0) / z.points.length; }
    const mid = (z.z_min + z.z_max) / 2;
    add('move', new THREE.Vector3(cx, cy, mid), { plane: mid });
    add('top', new THREE.Vector3(cx, cy, z.z_max), { axis: Z.clone() });
    add('bottom', new THREE.Vector3(cx, cy, z.z_min), { axis: Z.clone().negate() });
    if (z.shape === 'cylinder') add('radius', new THREE.Vector3(cx + z.radius, cy, mid), { axis: new THREE.Vector3(1, 0, 0) });
    else z.points.forEach((q, k) => add('corner', new THREE.Vector3(q[0], q[1], z.z_max), { plane: z.z_max, corner: k }));
  }
  return H;
}

const HANDLE_SIZE = { move: 0.02, lift: 0.014, yaw: 0.014 };

function rebuildShapeZones() {
  const g = shapeEd.group, hg = shapeEd.handles;
  if (!g) return;
  for (const o of [...g.children]) { g.remove(o); o.geometry.dispose(); o.material.dispose(); }
  for (const o of [...hg.children]) { hg.remove(o); o.geometry.dispose(); o.material.dispose(); }
  const c = shapeEd.colours, st = shapeEd.state;
  shapeEd.zones.forEach((z, i) => {
    let col = z.enabled === false ? c.off : z.keep === 'in' ? c.in : c.out;
    if (st && st.zone === i && st.state === 'violation') col = c.bad;
    else if (st && st.zone === i && st.state === 'warn') col = c.warn;
    for (const m of zoneMeshes(z, i, col, i === shapeEd.sel)) g.add(m);
  });
  const z = shapeEd.zones[shapeEd.sel];
  hg.visible = !!z && shapeEd.tool === 'orbit';
  if (!z) return;
  for (const hd of zoneHandles(z)) {
    const r = HANDLE_SIZE[hd.kind] || 0.013;
    const geom = hd.kind === 'move' ? new THREE.OctahedronGeometry(r) : hd.kind === 'yaw' ? new THREE.TorusGeometry(r, r * 0.35, 8, 16) : new THREE.SphereGeometry(r, 14, 10);
    const m = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({ color: hd.kind === 'move' || hd.kind === 'lift' ? c.handle : (z.keep === 'in' ? c.in : c.out), depthTest: false }));
    m.position.copy(hd.pos); m.renderOrder = 13;
    m.userData = hd;
    hg.add(m);
  }
}

function makeShapeOverlays() {
  shapeEd.group = new THREE.Group(); scene.add(shapeEd.group);
  shapeEd.handles = new THREE.Group(); shapeEd.handles.visible = false; scene.add(shapeEd.handles);
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 64), 3));
  geom.setDrawRange(0, 0);
  shapeEd.preview = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: 0xb42318, depthTest: false }));
  shapeEd.preview.renderOrder = 12; shapeEd.preview.visible = false; shapeEd.preview.frustumCulled = false; scene.add(shapeEd.preview);
  shapeEd.previewDots = new THREE.Group(); scene.add(shapeEd.previewDots);
  for (let k = 0; k < 32; k++) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.01, 10, 8), new THREE.MeshBasicMaterial({ color: 0xb42318, depthTest: false }));
    m.renderOrder = 12; m.visible = false; shapeEd.previewDots.add(m);
  }
  shapeEd.worst = new THREE.Mesh(new THREE.SphereGeometry(0.016, 16, 12), new THREE.MeshBasicMaterial({ color: 0xb7791f, depthTest: false }));
  shapeEd.worst.renderOrder = 12; shapeEd.worst.visible = false; scene.add(shapeEd.worst);
  shapeEd.sig = '';
}

/** The whole-arm safety zones to draw (the page's list, its draft included). opts: { sel: index of the selected
 *  one (handles on it), colours: { in, out, off, warn, bad, handle } (CSS), state: { zone, state } of the arm }. */
export function setShapeZones(zones, opts) {
  opts = opts || {};
  shapeEd.zones = zones || [];
  if (opts.sel != null) shapeEd.sel = opts.sel;
  if (opts.colours) shapeEd.colours = Object.assign({}, shapeEd.colours, opts.colours);
  shapeEd.state = opts.state || null;
  const sig = JSON.stringify([shapeEd.zones, shapeEd.sel, shapeEd.colours, shapeEd.state && [shapeEd.state.zone, shapeEd.state.state], shapeEd.tool]);
  if (sig === shapeEd.sig) return;
  shapeEd.sig = sig;
  rebuildShapeZones();
}

/** 'orbit' (turn the view, pick and drag zones), 'box', 'sphere', 'cylinder' (click the floor to place one), or
 *  'draw' (click corners of an outline on the plane at height z; click the first corner or double-click to close). */
export function setZoneTool(tool, z) {
  shapeEd.tool = tool || 'orbit';
  if (z != null) shapeEd.drawZ = z;
  if (shapeEd.tool !== 'draw') shapeEd.draw = [];
  shapeEd.drawPreview(null);
  shapeEd.sig = '';
  rebuildShapeZones();
  if (renderer) renderer.domElement.style.cursor = shapeEd.tool === 'draw' ? 'crosshair' : shapeEd.tool === 'orbit' ? '' : 'copy';
}

/** Drop the corners drawn so far, or the last one (undo). */
export function undoDraw(all) {
  if (all) shapeEd.draw = []; else shapeEd.draw.pop();
  shapeEd.drawPreview(null);
  if (shapeEd.cb && shapeEd.cb.drawing) shapeEd.cb.drawing(shapeEd.draw.length);
}
export function finishDraw() { shapeEd.finishDraw(); }

/** { select, change, place, drawn, drawing } as described above; null stops editing. */
export function onShapeZones(cb) { shapeEd.cb = cb || null; shapeEd.drag = null; shapeEd.draw = []; }

/** The arm's point nearest a zone boundary (from its controller), coloured as CSS, or hidden when p is null. */
export function setWorstPoint(p, css) {
  const o = shapeEd.worst;
  if (!o) return;
  o.visible = !!p;
  if (p) { o.position.set(p[0], p[1], p[2]); if (css) o.material.color.set(css); }
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
  makeFaces();
  makeShapeOverlays();
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
  faceDrag.cb = faceDrag.active = null;
  Object.assign(shapeEd, { zones: [], sel: -1, tool: 'orbit', cb: null, sig: '', draw: [], drag: null, press: null,
    group: null, handles: null, preview: null, previewDots: null, worst: null });
  for (const k of Object.keys(overlay)) delete overlay[k];
  maxFps = 60;
  bodies = [];
  pose = {};
}
