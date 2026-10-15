/* The control bar, along the bottom of every page, as in the classic Studio:
   a row of Joint, Cart, Gripper, Home, Contact, Snapshot and Quick Actions,
   with the mode readout beside them. Joint, Cart, Gripper, Contact, Snapshot
   and Quick Actions slide a panel up from the bar (a drawer that is part of
   the bar, never a page). All robot control lives here and only here. */
(() => {
'use strict';
const { $, h, fmt, setText, setSeg, setSelect, cmd, toast } = window.FIC;   // FIC.inlineConfirm: the out-of-reach question
const FIC = window.FIC;
const OPTION_SHORT = { hpmp: 'H-PMP', postural: 'Postural', elbow: 'Elbow', admittance_planner: 'Planner' };
const OPTION_KEY = { hpmp: 'f', postural: 'p', elbow: 'e', admittance_planner: 'p' };
const TARGET_SHORT = { simulation: 'SIM', hardware: 'HW', vr: 'VR' };
const kbd = k => h('span', { class: 'kbd', text: k });
const PANELS = [['joint', 'Joint'], ['cart', 'Cart'], ['gripper', 'Gripper'], ['contact', 'Contact'], ['snapshot', 'Snapshot'], ['quick', 'Quick Actions']];
let opened = null;            // which panel is open, or null
let selSeq = '';              // the sequence chosen under Quick Actions (Snapshot appends to it)
const J = { values: [], dirty: false, until: 0, sig: '', last: 0 };       // joint draft
const C = { values: {}, dirty: false, until: 0 };                          // Cartesian draft
const W = { dirty: false, until: 0 };                                       // desired-wrench draft
// [label, unit, step]: the Cartesian admittance's desired wrench (N, N m)
const WRENCH = [['Fx', 'N', 0.5], ['Fy', 'N', 0.5], ['Fz', 'N', 0.5], ['Mx', 'N m', 0.05], ['My', 'N m', 0.05], ['Mz', 'N m', 0.05]];
// [key, label, unit, decimals, slider min, slider max (X/Y/Z: until the robot's reach is known), nudge step]
const POSE = [['x', 'X', 'm', 4, -1, 1, 0.01], ['y', 'Y', 'm', 4, -1, 1, 0.01], ['z', 'Z', 'm', 4, -0.2, 1.4, 0.01], ['angle', 'Angle', '°', 2, -180, 180, 5],
  ['ax', 'Axis X', '', 4, -1, 1, 0.05], ['ay', 'Axis Y', '', 4, -1, 1, 0.05], ['az', 'Axis Z', '', 4, -1, 1, 0.05]];

function build() {
  const dock = $('#dock');
  const buttons = h('div', { class: 'bar-btns' });
  for (const [id, label] of PANELS) {
    buttons.append(h('button', { class: 'btn tab', type: 'button', 'data-panel': id, 'aria-expanded': 'false', onclick: () => toggle(id) },
      h('span', { class: 'caret' }, window.FIC.icon('chevron-up')), h('span', { text: label })));
    if (id === 'gripper') buttons.append(h('button', { class: 'btn', id: 'd-home', type: 'button', onclick: () => cmd('home', {}) }, h('span', { icon: 'home', text: 'Home' }), kbd('⇧H')));
  }
  dock.append(
    h('div', { class: 'drawer', id: 'drawer', hidden: true },
      h('div', { class: 'drawer-inner' },
        h('div', { class: 'drawer-head' }, h('span', { class: 'eyebrow', id: 'drawer-title' }), h('div', { class: 'drawer-slot', id: 'drawer-slot' }),
          h('button', { class: 'btn btn-icon', type: 'button', title: 'Close panel (Esc)', 'aria-label': 'Close panel', onclick: close }, window.FIC.icon('x'))),
        buildMode(), buildGripper(), buildContact(), buildSnapshot(), buildQuick())),
    h('div', { class: 'bar', id: 'bar' }, buttons,
      h('div', { class: 'bar-right' },
        h('span', { class: 'bar-status', id: 'bar-status' }, h('select', { class: 'select bar-robot', id: 'bar-robot', hidden: true, 'aria-label': 'Robot the control bar commands', title: 'The robot this bar commands' }), h('b', { id: 'ds-mode' }), h('span', { class: 'faint', id: 'ds-rest' })),
        h('span', { class: 'bar-contact faint', id: 'ds-contact' }))));
}

// ── Joint / Cart panel ────────────────────────────────────────────────────
function buildMode() {
  const jointBox = h('div', { class: 'jgrid', id: 'jgrid' });
  const cartBox = h('div', { class: 'cgrid', id: 'cgrid' });
  const presets = h('div', { class: 'btn-row', id: 'presets' });
  return h('div', { class: 'dpanel', id: 'dp-mode' },
    h('div', { class: 'dp-row dp-head', id: 'dp-mode-head' },
      h('div', { class: 'seg', id: 'd-mode', role: 'group', 'aria-label': 'Mode' }),
      h('div', { class: 'group', id: 'd-opts', role: 'group', 'aria-label': 'Options' }),
      h('label', { class: 'inline-field' }, h('span', { text: 'Config' }), h('select', { class: 'select', id: 'd-prof' }))),
    jointBox, cartBox, presets, buildWrench(),
    h('div', { class: 'd-confirm', id: 'd-confirm' }),
    h('div', { class: 'dp-row dp-foot' },
      h('label', { class: 'toggle', title: 'Every edit streams a command at once. Off: a command goes when you commit a field (release, Enter).' },
        h('input', { type: 'checkbox', id: 'd-live', onchange: e => cmd('live_publish', { value: e.target.checked }) }), h('span', { class: 'track' }), h('span', { text: 'Live publish' })),
      h('button', { class: 'btn', type: 'button', text: 'Reset', title: 'Reset all control fields to the robot’s defaults', onclick: () => { J.dirty = false; C.dirty = false; cmd('reset_defaults', {}); } }),
      h('button', { class: 'btn', type: 'button', id: 'd-pause', text: 'Pause', title: 'Pause the controller (hardware)', onclick: () => cmd('robot_pause', {}) }),
      h('button', { class: 'btn btn-danger', type: 'button', id: 'd-stop', text: 'Stop', title: 'Stop the controller (hardware)', onclick: () => cmd('robot_stop', {}) }),
      h('div', { class: 'spacer' }),
      h('button', { class: 'btn btn-solid', type: 'button', id: 'd-send', text: 'Send', title: 'Send the fields above to the robot', onclick: () => sendCurrent() })));
}
// The desired wrench of the Cartesian admittance (modes 7 and 8, and H-PMP on them).
function buildWrench() {
  const fields = WRENCH.map(([label, unit, step], i) => h('label', { class: 'inline-field' }, h('span', { text: label }),
    h('input', { class: 'input num', type: 'number', step: String(step), 'data-w': String(i), 'aria-label': 'Desired ' + label + ', ' + unit, title: unit, oninput: () => { W.dirty = true; } })));
  return h('div', { class: 'dp-row', id: 'd-wrench', hidden: true,
      title: 'Desired wrench of the Cartesian admittance (modes 7 and 8, and H-PMP on them): the goal moves until the measured wrench equals it. Zero at controller start-up.' },
    h('span', { class: 'eyebrow', text: 'Desired wrench' }), ...fields,
    h('button', { class: 'btn', type: 'button', text: 'Zero', title: 'Send a zero desired wrench', onclick: () => sendWrench(true) }),
    h('button', { class: 'btn', type: 'button', id: 'd-wrench-apply', text: 'Apply', title: 'Send this desired wrench', onclick: () => sendWrench(false) }));
}
async function sendWrench(zero) {
  const inputs = [...document.querySelectorAll('#d-wrench [data-w]')];
  const values = inputs.map(el => zero ? 0 : Number(el.value || 0));
  if (values.some(v => !Number.isFinite(v))) { toast('The desired wrench is six numbers'); return; }
  const r = await cmd('desired_wrench', { values });
  W.dirty = false; W.until = Date.now() + 700;
  return r;
}
function drawWrench(s, kind, av) {
  const row = $('#d-wrench'), vals = s.desired_wrench;
  row.hidden = kind !== 'cart' || !Array.isArray(vals);
  if (row.hidden) return;
  for (const el of row.querySelectorAll('input, button')) el.disabled = !av.command || !av.cart;
  if (W.dirty || Date.now() < W.until || row.contains(document.activeElement)) return;
  row.querySelectorAll('[data-w]').forEach((el, i) => { el.value = fmt(vals[i], 2); });
}
function sendCurrent() {
  if (panelKind() === 'cart') sendPoseChecked(); else sendJoints();
}
async function sendJoints() {
  const js = FIC.state().joints || [];
  const degrees = js.map((j, i) => (j.actuated && J.values[i] != null) ? J.values[i] : null);
  const r = await cmd('send_joints', { degrees });
  J.dirty = false; J.until = Date.now() + 700;
  return r;
}
async function sendPose() {
  const args = {};
  for (const [k] of POSE) args[k] = C.values[k];
  const r = await cmd('send_pose', args);
  C.dirty = false; C.until = Date.now() + 700;
  return r;
}
function live(fn) { const t = Date.now(); if (document.getElementById('d-live').checked && t - J.last > 90) { J.last = t; fn(); } }

function drawJoints(js) {
  const box = $('#jgrid');
  const sig = js.map(j => j.label + j.lo + j.hi + j.actuated).join('|');
  if (J.sig !== sig) {
    J.sig = sig; J.dirty = false; box.textContent = '';
    js.forEach((j, i) => {
      const num = h('input', { class: 'input num', type: 'number', step: '0.1', min: j.lo, max: j.hi, disabled: !j.actuated, 'aria-label': j.label + ' target, degrees', title: 'Arrow keys: 0.1°, Shift+arrow: 1°' });
      const rng = h('input', { type: 'range', min: j.lo, max: j.hi, step: '0.1', disabled: !j.actuated, 'aria-label': j.label + ' slider' });
      const set = v => { J.values[i] = v; J.dirty = true; num.value = fmt(v, 1); rng.value = v; };
      num.addEventListener('input', () => { if (num.value !== '' && !isNaN(parseFloat(num.value))) { J.values[i] = parseFloat(num.value); J.dirty = true; rng.value = num.value; live(sendJoints); } });
      rng.addEventListener('input', () => { set(parseFloat(rng.value)); live(sendJoints); });
      num.addEventListener('keydown', e => {                       // Shift+arrow: 1° steps
        if (!e.shiftKey || (e.key !== 'ArrowUp' && e.key !== 'ArrowDown')) return;
        e.preventDefault();
        const v = Math.min(j.hi, Math.max(j.lo, (parseFloat(num.value) || 0) + (e.key === 'ArrowUp' ? 1 : -1)));
        set(v); live(sendJoints); if (!document.getElementById('d-live').checked) sendJoints();
      });
      num.addEventListener('change', () => sendJoints());
      rng.addEventListener('change', () => sendJoints());
      box.append(h('div', { class: 'jr' + (j.actuated ? '' : ' off') },
        h('div', { class: 'jr-top' }, h('span', { class: 'jr-l', text: j.label }), h('span', { class: 'num faint jr-act', 'data-act': i }),
          h('span', { class: 'jr-v' }, num, h('span', { class: 'faint', text: '°' }))),
        h('div', { class: 'jr-track' }, rng, h('i', { class: 'jr-now', 'aria-hidden': 'true' }))));
    });
  }
  js.forEach((j, i) => {
    const row = box.children[i]; if (!row) return;
    setText(row.querySelector('[data-act]'), j.actual == null ? '' : 'now ' + fmt(j.actual, 1) + '°');
    const span = (j.hi - j.lo) || 1, now = row.querySelector('.jr-now');
    now.style.display = j.actual == null ? 'none' : '';
    if (j.actual != null) now.style.setProperty('--p', Math.min(1, Math.max(0, (j.actual - j.lo) / span)));
    if (!J.dirty && Date.now() > J.until) {
      const v = j.commanded != null ? j.commanded : j.actual; J.values[i] = v;
      const num = row.querySelector('input[type=number]'), rng = row.querySelector('input[type=range]');
      if (document.activeElement !== num && document.activeElement !== rng && v != null) { num.value = fmt(v, 1); rng.value = v; }
    }
  });
}
function drawCart(s) {
  const box = $('#cgrid');
  if (!box.__built) {
    box.__built = true;
    for (const [k, label, unit, d, lo, hi, nudge] of POSE) {
      const step = k === 'angle' ? '0.5' : '0.001';
      const num = h('input', { class: 'input num', type: 'number', step, 'data-k': k, 'aria-label': label + (unit ? ', ' + unit : ''), title: 'Arrow keys: ' + step + ', Shift+arrow: ' + nudge });
      const rng = h('input', { type: 'range', min: lo, max: hi, step, 'data-r': k, 'aria-label': label + ' slider' });
      const set = v => { C.values[k] = v; C.dirty = true; num.value = v.toFixed(d); rng.value = v; markReach(); };
      const bump = sign => { set((parseFloat(num.value) || 0) + sign * nudge); if (liveOn()) live(sendPoseLive); else sendPoseChecked(); };
      num.addEventListener('input', () => { if (num.value !== '' && !isNaN(parseFloat(num.value))) { C.values[k] = parseFloat(num.value); C.dirty = true; rng.value = num.value; markReach(); live(sendPoseLive); } });
      num.addEventListener('keydown', e => {
        if (!e.shiftKey || (e.key !== 'ArrowUp' && e.key !== 'ArrowDown')) return;
        e.preventDefault(); bump(e.key === 'ArrowUp' ? 1 : -1);
      });
      rng.addEventListener('input', () => { set(parseFloat(rng.value)); live(sendPoseLive); });
      num.addEventListener('change', () => sendPoseChecked());
      rng.addEventListener('change', () => sendPoseChecked());
      const nb = (sign, text) => h('button', { class: 'btn btn-nudge', type: 'button', text, title: (sign > 0 ? '+' : '−') + nudge + (unit ? ' ' + unit : ''),
        'aria-label': label + (sign > 0 ? ' up ' : ' down ') + nudge, onclick: () => bump(sign) });
      box.append(h('div', { class: 'jr' },
        h('div', { class: 'jr-top' }, h('span', { class: 'jr-l', text: label }), h('span', { class: 'num faint jr-act', 'data-act': k }),
          nb(-1, '−'), h('span', { class: 'jr-v' }, num, h('span', { class: 'faint', text: unit })), nb(1, '+')),
        h('div', { class: 'jr-track jr-free', 'data-t': k }, rng, h('i', { class: 'jr-now', 'data-now': k, 'aria-hidden': 'true' }))));
    }
    box.append(h('div', { class: 'cf-note faint', id: 'cf-note', text: NOTE }));
  }
  const pose = s.pose || {}, cmdd = pose.commanded, act = pose.actual;
  C.reach = pose.reach && pose.reach.radius > 0 ? pose.reach : null;
  C.act = act;
  const fixed = cartRanges(C.reach);
  for (const [k, , unit, d, lo0, hi0] of POSE) {
    const [lo, hi] = fixed[k] || [lo0, hi0];
    const num = box.querySelector('[data-k="' + k + '"]'), rng = box.querySelector('[data-r="' + k + '"]');
    const a = act && act[k] != null ? act[k] : null, now = box.querySelector('[data-now="' + k + '"]');
    setText(box.querySelector('[data-act="' + k + '"]'), a == null ? '' : 'now ' + a.toFixed(k === 'angle' ? 1 : 3) + (unit === '°' ? '°' : ''));
    if (String(rng.min) !== String(lo) || String(rng.max) !== String(hi)) {    // changes only with the robot
      rng.min = lo; rng.max = hi;
      if (C.values[k] != null) rng.value = C.values[k];
    }
    now.style.display = a == null ? 'none' : '';
    if (a != null) now.style.setProperty('--p', Math.min(1, Math.max(0, (a - lo) / (hi - lo))));
    if (!C.dirty && Date.now() > C.until && cmdd && cmdd[k] != null) {
      C.values[k] = cmdd[k];
      if (document.activeElement !== num && document.activeElement !== rng) { num.value = cmdd[k].toFixed(d); rng.value = cmdd[k]; }
    }
  }
  markReach();
}

const NOTE = 'Target in the base frame, orientation as axis and angle. Release a slider or commit a field to send. The bright part of each track is within the arm’s reach.';
const AXES = ['x', 'y', 'z'];

/** Fixed X/Y/Z slider ranges for this robot: the box around its reach sphere (centre ± radius),
 *  rounded out to 5 cm, so a handle's place on its track always means the same position. */
function cartRanges(reach) {
  if (!reach) return {};
  const out = {};
  AXES.forEach((k, i) => {
    const c = reach.center[i], r = reach.radius;
    out[k] = [Math.floor((c - r) * 20) / 20, Math.ceil((c + r) * 20) / 20].map(v => +v.toFixed(2));
  });
  return out;
}
/** The target's position for reach checks (the draft, or the measured pose before any draft). */
function targetAt(k) {
  if (C.values[k] != null) return C.values[k];
  return C.act && C.act[k] != null ? C.act[k] : null;
}
/** How far the target is from the reach sphere's centre, and the radius; null without a reach. */
function reachOf() {
  if (!C.reach) return null;
  let d2 = 0;
  for (let i = 0; i < 3; i++) { const v = targetAt(AXES[i]); if (v == null) return null; d2 += (v - C.reach.center[i]) ** 2; }
  return { dist: Math.sqrt(d2), radius: C.reach.radius };
}
function inReach() { const r = reachOf(); return !r || r.dist <= r.radius * 1.001; }
/** Shade each X/Y/Z track: the stretch of that axis, through the target's other two
 *  values, that lies inside the reach sphere is drawn bright; the rest stays faint.
 *  Then say on Send when the target itself is out of reach. */
function markReach() {
  const box = $('#cgrid'); if (!box) return;
  AXES.forEach((k, i) => {
    const track = box.querySelector('[data-t="' + k + '"]'), rng = box.querySelector('[data-r="' + k + '"]');
    if (!track) return;
    if (!C.reach) { track.style.removeProperty('--a'); track.style.removeProperty('--b'); track.classList.remove('has-reach'); return; }
    let off2 = 0, ok = true;
    AXES.forEach((o, m) => { if (m === i) return; const v = targetAt(o); if (v == null) ok = false; else off2 += (v - C.reach.center[m]) ** 2; });
    const lo = parseFloat(rng.min), hi = parseFloat(rng.max), span = (hi - lo) || 1;
    const half = ok ? Math.sqrt(Math.max(0, C.reach.radius ** 2 - off2)) : 0;
    const a = Math.min(1, Math.max(0, (C.reach.center[i] - half - lo) / span)), b = Math.min(1, Math.max(0, (C.reach.center[i] + half - lo) / span));
    track.classList.add('has-reach');
    track.style.setProperty('--a', a); track.style.setProperty('--b', half > 0 ? b : a);
  });
  sendLabel();
}
/** Which controls the open mode panel shows: Joint or Cart, as opened (not the robot's current mode). */
function panelKind() { return opened === 'cart' ? 'cart' : 'joint'; }
function liveOn() { const l = document.getElementById('d-live'); return !!(l && l.checked); }
/** The Send button's words: Streaming while Live publish is on, a warning when the Cartesian target is out of reach. */
function sendLabel() {
  const btn = $('#d-send'); if (!btn) return;
  const cart = panelKind() === 'cart';
  const out = cart && !inReach();
  btn.classList.toggle('btn-warn', out);
  setText(btn, liveOn() ? 'Streaming' : (out ? 'Outside reach' : 'Send'));
  const note = $('#cf-note');
  if (note) {
    const r = reachOf();
    setText(note, out && r ? 'The target is ' + r.dist.toFixed(2) + ' m from the shoulder; the arm reaches ' + r.radius.toFixed(2) + ' m. Sent as it is, the arm stretches toward it and pushes against its limits.' : NOTE);
    note.classList.toggle('warn-text', out);
  }
}
/** Live streaming never sends a target outside the reach: it waits until the target is back inside. */
function sendPoseLive() { if (inReach()) sendPose(); }
/** Send the Cartesian target; outside the reach, ask first. */
function sendPoseChecked() {
  if (inReach()) return sendPose();
  const r = reachOf();
  FIC.inlineConfirm($('#d-confirm'), {
    title: 'Target outside the arm’s reach',
    lines: [r.dist.toFixed(2) + ' m from the shoulder, reach ' + r.radius.toFixed(2) + ' m',
      'The arm will stretch toward it and push against its limits.'],
    label: 'Send anyway', danger: true, onConfirm: () => sendPose() });
}

// ── Gripper, Contact, Snapshot, Quick Actions ─────────────────────────────
function buildGripper() {
  const rng = h('input', { type: 'range', min: 0, max: 100, step: '1', id: 'd-gr', 'aria-label': 'Gripper opening (0 closed, 100 open)' });
  rng.addEventListener('change', () => cmd('gripper', { percent: parseFloat(rng.value) }));
  rng.addEventListener('input', () => setText($('#d-gr-v'), rng.value + ' %'));
  return h('div', { class: 'dpanel', id: 'dp-gripper' },
    h('div', { class: 'dp-row' }, h('span', { class: 'faint', text: 'Width  0 % closed, 100 % open' }),
      h('div', { class: 'grow' }, rng), h('span', { class: 'num', id: 'd-gr-v' }),
      h('button', { class: 'btn', type: 'button', text: 'Close', onclick: () => cmd('gripper', { percent: 0 }) }),
      h('button', { class: 'btn', type: 'button', text: 'Open', onclick: () => cmd('gripper', { percent: 100 }) })),
    h('div', { class: 'dp-row faint', id: 'd-gr-note' }));
}
function buildContact() {
  return h('div', { class: 'dpanel', id: 'dp-contact' },
    h('div', { class: 'dp-row' },
      h('button', { class: 'btn', type: 'button', icon: 'record', text: 'Calibrate', onclick: () => cmd('contact_calibrate', {}) }),
      h('div', { class: 'field' }, h('label', { for: 'd-gt', text: 'Ground truth' }), h('select', { class: 'select', id: 'd-gt' })),
      h('span', { class: 'faint', text: 'Hold the arm still and clear of contact while calibrating.' })));
}
function buildSnapshot() {
  const kind = h('select', { class: 'select', id: 'd-snapkind', 'aria-label': 'Capture as' }, ...['cartesian', 'joint', 'gripper'].map(k => h('option', { value: k, text: k })));
  return h('div', { class: 'dpanel', id: 'dp-snapshot' },
    h('div', { class: 'dp-row' }, h('div', { class: 'field' }, h('label', { text: 'Capture as' }), kind),
      h('button', { class: 'btn', type: 'button', id: 'd-capture', icon: 'record', text: 'Capture', onclick: () => cmd('capture', { kind: kind.value, sequence: selSeq }).then(r => { if (r.ok !== false) toast('Waypoint appended to ' + selSeq); }) }),
      h('span', { class: 'faint', text: 'Appends a waypoint to the sequence selected in Quick Actions.' })));
}
function buildQuick() {
  const sel = h('select', { class: 'select', id: 'd-seq', 'aria-label': 'Sequence' });
  sel.addEventListener('change', () => { selSeq = sel.value; });
  const newName = h('input', { class: 'input', id: 'd-seq-new', type: 'text', placeholder: 'new sequence', maxlength: 60, 'aria-label': 'New sequence name' });
  const speed = h('input', { class: 'input num', id: 'd-seq-speed', type: 'number', min: 0.1, max: 3, step: 0.1, 'aria-label': 'Playback speed' });
  speed.addEventListener('change', () => cmd('sequence', { op: 'speed', value: parseFloat(speed.value) }));
  const num = (id, label, min, max, step, value) => h('input', { class: 'input num', id, type: 'number', min, max, step, value, 'aria-label': label });
  const kind = h('select', { class: 'select', id: 'd-tr-kind', 'aria-label': 'Trajectory' }, ...[['line', 'Line'], ['circle', 'Circle'], ['eight', 'Figure-eight']].map(([v, t]) => h('option', { value: v, text: t })));
  const plane = h('select', { class: 'select', id: 'd-tr-plane', 'aria-label': 'Plane' }, ...['xy', 'xz', 'yz'].map(v => h('option', { value: v, text: v })));
  const size = num('d-tr-size', 'Size (mm)', 10, 150, 5, 50), period = num('d-tr-period', 'Period (s)', 2, 60, 1, 8), cycles = num('d-tr-cycles', 'Cycles', 1, 10, 1, 2);
  const ask = h('div', { id: 'd-tr-ask' });
  const trArgs = () => ({ op: 'trajectory', kind: kind.value, size: parseFloat(size.value) / 1000, period: parseFloat(period.value), cycles: parseInt(cycles.value, 10), plane: plane.value });
  const run = () => cmd('sequence', trArgs(), true).then(r => {
    if (r.ok === false && r.error === 'confirm_hw') {
      FIC.inlineConfirm(ask, { title: 'Run the trajectory on the REAL robot?', lines: ['It goes to the Sequences target, a real robot.'], label: 'Run it', danger: true,
        onConfirm: () => cmd('sequence', Object.assign(trArgs(), { hardware_ok: true })).then(q => { if (q.ok !== false) toast('Tracing: ' + fmt(q.result.duration, 0) + ' s'); }) });
    } else if (r.ok === false) toast(r.message || 'Refused', true);
    else toast('Tracing: ' + fmt(r.result.duration, 0) + ' s — Stop ends it');
  });
  const saveTr = () => {
    const name = kind.value + '_' + size.value + 'mm';
    cmd('sequence', { op: 'save_trajectory', name, kind: kind.value, size: parseFloat(size.value) / 1000, plane: plane.value })
      .then(r => { if (r.ok !== false) { selSeq = r.result.name; toast('Saved “' + selSeq + '”: play it to compare'); } });
  };
  return h('div', { class: 'dpanel', id: 'dp-quick' },
    h('div', { class: 'dp-row' }, h('div', { class: 'field' }, h('label', { for: 'd-seq', text: 'Sequence' }), sel),
      newName, h('button', { class: 'btn', type: 'button', text: 'New', title: 'Make an empty sequence; Snapshot adds waypoints to it',
        onclick: () => cmd('sequence', { op: 'new', name: newName.value }).then(r => { if (r.ok !== false) { selSeq = r.result.name; newName.value = ''; toast('New sequence ' + selSeq + ': press Snapshot for each waypoint'); } }) }),
      h('div', { class: 'spacer' }),
      h('button', { class: 'btn', type: 'button', icon: 'play', text: 'Play', onclick: () => cmd('sequence', { op: 'play', name: selSeq }) }),
      h('button', { class: 'btn', type: 'button', text: 'Step', title: 'Move to the next waypoint', onclick: () => cmd('sequence', { op: 'step', name: selSeq }) }),
      h('button', { class: 'btn', type: 'button', icon: 'stop', text: 'Stop', onclick: () => cmd('sequence', { op: 'stop' }) }),
      h('button', { class: 'btn', type: 'button', icon: 'refresh', text: 'Loop', onclick: () => cmd('sequence', { op: 'loop', name: selSeq }) }),
      h('label', { class: 'faint', for: 'd-seq-speed', text: 'Speed ×' }), speed),
    h('div', { class: 'dp-row' }, h('div', { class: 'field' }, h('label', { for: 'd-tr-kind', text: 'Trajectory' }), kind),
      size, h('span', { class: 'faint', text: 'mm' }), period, h('span', { class: 'faint', text: 's' }), cycles, h('span', { class: 'faint', text: '×' }), plane,
      h('div', { class: 'spacer' }),
      h('button', { class: 'btn', type: 'button', icon: 'play', text: 'Run', title: 'Trace it from where the end effector is, holding its orientation', onclick: run }),
      h('button', { class: 'btn', type: 'button', text: 'Save as sequence', title: 'One cycle as waypoints, to compare a sequence with the trajectory', onclick: saveTr })),
    ask,
    h('div', { id: 'd-qa-ext' }),
    h('div', { class: 'dp-row faint', id: 'd-seq-note' }));
}
let qaSig = '';
const qaChoice = {};
function drawQuickExt(list) {
  const sig = JSON.stringify(list.map(a => [a.key, a.label, a.hint, a.button, !!a.choices]));
  if (sig !== qaSig) {
    qaSig = sig;
    $('#d-qa-ext').replaceChildren(...list.map(a => {
      const pick = a.choices ? h('select', { class: 'select', 'data-qa': a.key, 'aria-label': a.label }) : null;
      if (pick) pick.addEventListener('change', () => { qaChoice[a.key] = pick.value; });
      return h('div', { class: 'dp-row' }, h('div', { class: 'field' }, h('label', { text: a.label }), pick || h('span')),
        a.hint ? h('span', { class: 'faint', text: a.hint }) : null, h('div', { class: 'spacer' }),
        h('button', { class: 'btn', type: 'button', icon: 'play', text: a.button,
          onclick: () => cmd('quick_action', { key: a.key, choice: pick ? pick.value : null }).then(r => { if (r.ok !== false) toast(r.result.message); }) }));
    }));
  }
  for (const a of list) {
    const pick = document.querySelector('[data-qa="' + CSS.escape(a.key) + '"]');
    if (pick && a.choices) setSelect(pick, a.choices.map(v => ({ value: v, label: v })), qaChoice[a.key] || a.choices[0] || '', v => { qaChoice[a.key] = v; });
  }
}

// ── Opening and closing ───────────────────────────────────────────────────
function show(id) {
  opened = id;
  $('#drawer').hidden = id == null;
  document.body.classList.toggle('drawer-open', id != null);
  for (const b of document.querySelectorAll('.bar-btns .tab')) { const on = b.dataset.panel === id; b.setAttribute('aria-expanded', String(on)); b.classList.toggle('open', on); }
  const isMode = id === 'joint' || id === 'cart';
  $('#dp-mode').hidden = !isMode;
  const head = $('#dp-mode-head');
  if (isMode) $('#drawer-slot').append(head); else $('#dp-mode').prepend(head);
  for (const [pid] of PANELS) { const p = $('#dp-' + pid); if (p) p.hidden = pid !== id; }
  const title = (PANELS.find(p => p[0] === id) || [])[1] || '';
  setText($('#drawer-title'), title);
  FIC.measure();
  if (FIC.state()) update(FIC.state());
}
function toggle(id) {
  if (opened === id) { show(null); return; }
  const s = FIC.state() || {}, mode = s.mode || {}, av = s.availability || {};
  if ((id === 'joint' || id === 'cart') && mode.kind !== id) {          // opening a mode panel commits to that kind, as in the classic bar
    const m = (mode.menus || {})[id];
    if (m && m.modes.length && (id === 'joint' || av.cart) && av.command) cmd('set_mode', { index: m.modes[0].index });
  }
  show(id);
}
function open(id) { if (opened !== id) toggle(id); }
function close() { if (opened == null) return false; show(null); return true; }
function toggleGripper() {
  const g = (FIC.state() || {}).gripper || {};
  if (!g.available) return;
  cmd('gripper', { percent: g.percent > 50 ? 0 : 100 });
}

// ── Update from the snapshot ──────────────────────────────────────────────
function update(s) {
  const mode = s.mode || { menus: {}, index: 0, kind: 'joint' }, av = s.availability || {};
  const sess = s.session || {};
  for (const b of document.querySelectorAll('.bar-btns .tab')) {
    const id = b.dataset.panel;
    b.disabled = (id === 'joint' && !av.joint) || (id === 'cart' && !av.cart) || (id === 'gripper' && !av.gripper);
    if (id === 'quick' || id === 'snapshot') b.disabled = !(s.sequences && s.sequences.available);
    if (id === 'joint') b.classList.toggle('current', mode.kind === 'joint');
    if (id === 'cart') b.classList.toggle('current', mode.kind === 'cart');
  }
  $('#d-home').disabled = !av.home;
  const hl = (s.telemetry && s.telemetry.health) || {};
  const st = s.stations || { items: [] }, pick = $('#bar-robot');     // which robot the bar commands (several stations only)
  pick.hidden = st.items.length < 2;
  if (st.items.length > 1) setSelect(pick, st.items.map(x => ({ value: String(x.n), label: x.label })), String((st.items.find(x => x.rid === st.active) || {}).n), v => cmd('station', { op: 'select', n: Number(v) }));
  setText($('#ds-mode'), mode.name || '—');
  setText($('#ds-rest'), '  ·  ' + (TARGET_SHORT[sess.target] || '') + '  ·  ' + (hl.rate != null && hl.state !== 'offline' ? fmt(hl.rate, 0) + ' Hz' : hl.state === 'offline' || !hl.state ? 'offline' : hl.state));
  setText($('#ds-contact'), (s.status && s.status.contact) || '');
  if (!opened) return;
  if (opened === 'joint' || opened === 'cart') {
    // The panel shows what was opened (Joint or Cart), whatever mode the robot is in; picking a mode here switches it.
    const kind = opened, same = mode.kind === kind;
    const cur = (mode.menus || {})[kind] || { modes: [] };
    setSeg($('#d-mode'), cur.modes.map(m => ({ value: m.index, label: m.label, disabled: !av.command || (kind === 'cart' && !av.cart) })), same ? mode.index : null, i => cmd('set_mode', { index: Number(i) }));
    const prof = s.profiles || { items: [], current: '' };
    setSelect($('#d-prof'), prof.items.map(p => ({ value: p.name, label: p.label })), prof.current, name => cmd('set_profile', { name }));
    const opts = Object.entries(s.options || {}).filter(([, o]) => o.kind === kind);
    const box = $('#d-opts'), osig = opts.map(([k]) => k).join('|');
    if (box.__sig !== osig) {
      box.__sig = osig; box.textContent = '';
      for (const [k] of opts) {
        box.append(h('label', { class: 'toggle', title: (s.options[k].tip || s.options[k].label) + '  [' + (OPTION_KEY[k] || '').toUpperCase() + ']' },
          h('input', { type: 'checkbox', 'data-opt': k, onchange: e => cmd('set_option', { name: k, value: e.target.checked }) }),
          h('span', { class: 'track' }), h('span', { text: OPTION_SHORT[k] || s.options[k].label }), kbd((OPTION_KEY[k] || '').toUpperCase())));
      }
      if (!opts.length) box.append(h('span', { class: 'faint', text: 'none for this robot' }));
    }
    for (const [k, o] of opts) { const b = box.querySelector('[data-opt="' + k + '"]'); if (b) { b.checked = !!o.value; b.disabled = !av[o.kind]; } }
    const dr = s.drawer || {};
    $('#jgrid').hidden = kind !== 'joint'; $('#cgrid').hidden = kind !== 'cart';
    if (kind === 'joint') drawJoints(s.joints || []); else drawCart(s);
    drawWrench(s, kind, av);
    const pre = $('#presets'), plist = kind === 'cart' ? (dr.presets || []) : [];
    const psig = plist.map(p => p.label).join('|');
    if (pre.__sig !== psig) {
      pre.__sig = psig; pre.textContent = '';
      if (plist.length) pre.append(h('span', { class: 'eyebrow', text: 'Preset poses' }), ...plist.map((p, i) => h('button', { class: 'btn', type: 'button', text: p.label, onclick: () => cmd('preset', { index: i }) })));
    }
    $('#d-live').checked = !!dr.live_publish;
    $('#d-pause').hidden = $('#d-stop').hidden = !dr.hardware;
    const streaming = !!dr.live_publish;
    $('#d-send').disabled = !av.command || streaming;
    sendLabel();
    const reason = !av.command ? av.reason : (kind === 'cart' && !av.cart ? av.cart_reason
      : (!same && mode.name ? 'The robot is in ' + mode.name + ': pick a ' + (kind === 'cart' ? 'Cartesian' : 'joint') + ' mode above to drive it from here.' : ''));
    $('#dp-mode').dataset.reason = reason || '';
  } else if (opened === 'gripper') {
    const g = s.gripper || {}, rng = $('#d-gr');
    rng.disabled = !g.available;
    if (document.activeElement !== rng && g.percent != null) rng.value = g.percent;
    setText($('#d-gr-v'), g.available ? fmt(rng.value, 0) + ' %' : 'none');
    setText($('#d-gr-note'), g.available ? (g.measured != null ? 'Measured ' + fmt(g.measured * 1000, 0) + ' mm' : '') : 'This robot has no gripper');
  } else if (opened === 'contact') {
    const dr = s.drawer || {}, gt = dr.ground_truth || { items: ['none'], current: 'none' };
    setSelect($('#d-gt'), gt.items.map(v => ({ value: v, label: v })), gt.current, link => cmd('contact_truth', { link }));
  } else if (opened === 'quick' || opened === 'snapshot') {
    const items = (s.sequences && s.sequences.items) || [];
    setSelect($('#d-seq'), items.map(v => ({ value: v, label: v })), selSeq, v => { selSeq = v; });
    if (!selSeq || !items.includes(selSeq)) selSeq = items[0] || '';
    $('#d-seq').value = selSeq;
    const sq = s.sequences || {}, tr = sq.trajectory || {};
    setText($('#d-seq-note'), sq.available ? (tr.running ? 'Tracing the ' + tr.kind + ': ' + fmt(100 * tr.progress, 0) + ' %' : sq.playing ? 'Playing' : items.length + ' sequences' + (sq.hardware ? ' · sends to the REAL robot' : '')) : 'The Sequences extension is off: turn it on in Settings, Extensions.');
    $('#d-capture').disabled = !selSeq;
    const sp = $('#d-seq-speed');
    if (sp && sq.speed != null && document.activeElement !== sp) sp.value = sq.speed;
    if ($('#d-qa-ext')) drawQuickExt(s.quick_actions || []);
  }
}

/** Page padding follows the real height of the bars (they wrap at narrow widths). */
function measure() {
  const root = document.documentElement.style;
  root.setProperty('--bar-h', $('#topbar').offsetHeight + 'px');
  root.setProperty('--dock-h', $('#bar').offsetHeight + 'px');
  root.setProperty('--status-h', document.body.classList.contains('no-status') ? '0px' : $('#statusbar').offsetHeight + 'px');
}
FIC.measure = measure;

FIC.dock = { open, close, toggleGripper, update, build };
window.addEventListener('load', () => {
  build();
  window.addEventListener('resize', measure);
  // A press anywhere outside the control bar and its open panel closes the panel
  // (the bar's own buttons toggle it themselves; E-STOP still acts as usual).
  document.addEventListener('pointerdown', e => {
    if (opened == null || !e.isPrimary) return;
    if ($('#dock').contains(e.target)) return;
    close();
  }, true);
  if (window.ResizeObserver) { const ro = new ResizeObserver(measure); ro.observe($('#topbar')); ro.observe($('#bar')); ro.observe($('#statusbar')); }
  measure();
});
})();
