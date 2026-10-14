/* Dashboard live view: the robot in 3D with the end-effector path, the commanded
   target and path, the safety zone and the contact force drawn over it, and
   beside it live plots of the same signals. Display only: the Studio sends the
   samples (snapshot sections `trace`, `dashboard`, `safety`, `telemetry`); the
   page keeps a bounded ring of them and draws. No kinematics or control here.
   Redraw rates: 3D overlays 30 Hz at most, plots 15 Hz. */
(() => {
'use strict';
const { h, fmt, setText, spark } = window.FIC;

const CAP = 1024;                       // samples kept (about 100 s at the 10 Hz state stream)
const MAXJ = 12;
const WINDOWS = [5, 10, 30];
const PLOT_HZ = 15, SCENE_HZ = 30;
const CH = { ee: 0, tg: 3, err: 6, force: 7, pos: 8, cmd: 8 + MAXJ, tau: 8 + 2 * MAXJ, total: 8 + 3 * MAXJ };
const VAR = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

/** Fixed-size ring of time-stamped samples; one Float32Array per channel, NaN = no value. */
class Ring {
  constructor(channels) {
    this.t = new Float64Array(CAP); this.c = Array.from({ length: channels }, () => new Float32Array(CAP).fill(NaN));
    this.head = 0; this.n = 0;
  }
  push(t, values) {
    const i = this.head;
    this.t[i] = t;
    for (let k = 0; k < this.c.length; k++) { const v = values[k]; this.c[k][i] = v == null ? NaN : v; }
    this.head = (i + 1) % CAP; this.n = Math.min(CAP, this.n + 1);
  }
  last() { return this.n ? this.t[(this.head - 1 + CAP) % CAP] : 0; }
  /** Indices (oldest first) of the samples in the last `sec` seconds. */
  span(sec) {
    const t1 = this.last(), out = [];
    for (let j = this.n; j >= 1; j--) {
      const i = (this.head - j + CAP) % CAP;
      if (this.t[i] >= t1 - sec) out.push(i);
    }
    return out;
  }
}

/** One canvas plot of several series over the last `sec` seconds. */
class Plot {
  constructor(canvas) { this.canvas = canvas; this.ctx = canvas.getContext('2d'); }
  draw(ring, idx, series, sec, colours) {
    const cv = this.canvas, dpr = Math.min(2, window.devicePixelRatio || 1);
    const w = cv.clientWidth, hgt = cv.clientHeight;
    if (!w || !hgt) return;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(hgt * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hgt * dpr); }
    const g = this.ctx;
    g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, hgt);
    const L = 40, R = 6, T = 6, B = 16, pw = w - L - R, ph = hgt - T - B;
    let lo = Infinity, hi = -Infinity;
    for (const s of series) for (const i of idx) { const v = ring.c[s.ch][i]; if (v === v) { if (v < lo) lo = v; if (v > hi) hi = v; } }
    const t1 = ring.last();
    g.font = '10px ' + colours.mono; g.textBaseline = 'middle'; g.lineWidth = 1;
    if (!(hi >= lo)) {
      g.fillStyle = colours.faint; g.textAlign = 'center'; g.fillText('no data', L + pw / 2, T + ph / 2); return;
    }
    if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5; }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
    const X = t => L + (1 - (t1 - t) / sec) * pw, Y = v => T + (1 - (v - lo) / (hi - lo)) * ph;
    g.strokeStyle = colours.rule; g.fillStyle = colours.faint; g.textAlign = 'right';
    for (let k = 0; k <= 2; k++) {
      const v = lo + (hi - lo) * k / 2, y = Y(v);
      g.beginPath(); g.moveTo(L, y + .5); g.lineTo(L + pw, y + .5); g.stroke();
      g.fillText(Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2), L - 5, y);
    }
    g.textAlign = 'left'; g.fillText('−' + sec + ' s', L, hgt - 5); g.textAlign = 'right'; g.fillText('now', L + pw, hgt - 5);
    for (const s of series) {
      g.strokeStyle = colours[s.colour] || colours.accent; g.lineWidth = s.dashed ? 1.2 : 1.6;
      g.setLineDash(s.dashed ? [4, 3] : []); g.globalAlpha = s.alpha || 1;
      g.beginPath(); let pen = false;
      for (const i of idx) {
        const v = ring.c[s.ch][i];
        if (v !== v) { pen = false; continue; }
        const x = X(ring.t[i]), y = Y(v);
        if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; }
      }
      g.stroke();
    }
    g.setLineDash([]); g.globalAlpha = 1;
  }
}

function legend(items) {
  return h('span', { class: 'legend' }, items.map(([label, colour, dashed]) =>
    h('span', { style: 'color:var(' + colour + ')' }, h('i', { class: dashed ? 'dash' : '' }), label)));
}

/** Build the live view into `parent`. Returns { update(state), dispose() }. */
window.FIC.dashLive = function buildLive(parent) {
  const store = window.FIC.store;
  const pref = (k, d) => { const v = store.get('fic-live-' + k); return v == null ? d : v; };
  const on = { trail: pref('trail', '1') === '1', path: pref('path', '1') === '1', target: pref('target', '1') === '1',
    zones: pref('zones', '0') === '1', force: pref('force', '1') === '1' };
  let sec = WINDOWS.includes(+pref('win', 10)) ? +pref('win', 10) : 10;
  let joint = Math.max(0, +pref('joint', 0) | 0);
  const ring = new Ring(CH.total);
  let mod = null, paletteKey = '', dead = false, lastScene = 0, lastPlot = 0, dirty = false, lastSnap = null, colours = null, colourAt = 0;

  // ── 3D side ──
  const stage = h('div', { class: 'stage', 'aria-label': '3D view of the robot with path, target and force overlays' });
  const note = h('div', { class: 'faint', style: 'font-size:var(--fs-m);margin-top:8px', text: 'Loading the robot…' });
  const toggles = h('span', { class: 'seg', role: 'group', 'aria-label': 'Overlays' });
  const winSeg = h('span', { class: 'seg', role: 'group', 'aria-label': 'History length' });
  const NAMES = [['trail', 'Trail', 'Measured end-effector path'], ['path', 'Commanded', 'Commanded / trajectory path'],
    ['target', 'Target', 'Current target marker'], ['zones', 'Zones', 'Safety-zone box'], ['force', 'Force', 'Contact force arrow']];
  for (const [k, label, title] of NAMES) {
    toggles.append(h('button', { type: 'button', text: label, title, 'aria-pressed': String(on[k]), 'data-k': k, onclick: ev => {
      on[k] = !on[k]; ev.currentTarget.setAttribute('aria-pressed', String(on[k])); store.set('fic-live-' + k, on[k] ? '1' : '0');
      if (mod) mod.setOverlayVisible(k, on[k]); dirty = true;
    } }));
  }
  function paintWin() {
    winSeg.textContent = '';
    for (const w of WINDOWS) winSeg.append(h('button', { type: 'button', text: w + ' s', 'aria-pressed': String(w === sec), onclick: () => { sec = w; store.set('fic-live-win', String(w)); paintWin(); dirty = true; lastScene = 0; } }));
  }
  paintWin();
  const errNum = h('span', { class: 'big', text: '—' }), errUnit = h('span', { class: 'faint', text: 'mm' });
  const errTrend = h('span', { class: 'trend' }), srcNote = h('span', { class: 'faint', text: '' });
  const left = h('div', { class: 'live-3d' },
    h('div', { class: 'live-bar' }, h('span', null, h('span', { class: 'lbl', text: 'overlay' }), toggles), h('span', { class: 'grow' }),
      h('span', null, h('span', { class: 'lbl', text: 'last' }), winSeg)),
    stage,
    h('div', { class: 'live-read' }, h('span', null, h('span', { class: 'lbl eyebrow', text: 'tracking error ' }), errNum, ' ', errUnit), errTrend, srcNote), note);

  // ── plots side ──
  const mk = () => h('canvas', { class: 'plot' });
  const cEE = mk(), cJ = mk(), cT = mk();
  const jSeg = h('span', { class: 'seg', role: 'group', 'aria-label': 'Joint' });
  const teleRate = h('span', { class: 'spark-box' }), teleForce = h('span', { class: 'spark-box' });
  const right = h('div', { class: 'plots' },
    h('div', null, h('div', { class: 'plot-head' }, h('span', { class: 'eyebrow', text: 'end effector, m' }),
      legend([['x', '--c1'], ['y', '--c2'], ['z', '--c3']]), legend([['commanded', '--ink-muted', true]])), cEE),
    h('div', null, h('div', { class: 'plot-head' }, h('span', { class: 'eyebrow', text: 'joint, °' }), jSeg,
      legend([['measured', '--c1']]), legend([['commanded', '--c2', true]])), cJ),
    h('div', null, h('div', { class: 'plot-head' }, h('span', { class: 'eyebrow', text: 'joint effort, N·m' }), h('span', { class: 'faint', style: 'font-size:var(--fs-s)', text: 'one line per joint' })), cT),
    h('div', { class: 'tele-spark' }, h('span', null, 'update rate ', teleRate), h('span', null, '|F| ', teleForce)));
  parent.append(h('div', { class: 'live-row' }, left, right));
  const plots = [new Plot(cEE), new Plot(cJ), new Plot(cT)];

  // ── 3D module ──
  let robotKey = null, loading = false;
  const keyOf = s => (s && ((s.robots && s.robots.current) || (s.studio && s.studio.robot_display))) || '';
  async function load() {
    loading = true;
    try {
      const m = await import('/view3d.js');
      if (mod) { mod.unmount(); mod = null; }
      robotKey = keyOf(window.FIC.state());
      setText(note, 'Loading the robot…');
      await m.mount(stage);
      if (dead) { m.unmount(); return; }
      mod = m; m.setMaxFps(30); m.setView([0.2, 0, 0.6], 1.45, -0.9, 1.15);
      for (const [k] of NAMES) m.setOverlayVisible(k, on[k]);
      setText(note, 'Drag to turn · scroll or pinch to zoom · overlays are in the robot base frame');
      colourAt = 0; paletteKey = ''; if (lastSnap) { m.setJoints(lastSnap.joints); m.setGripper(lastSnap.gripper); }
      dirty = true; lastScene = 0;
    } catch (err) {
      setText(note, 'No 3D view: ' + (err && err.message ? err.message : err));
    } finally { loading = false; }
  }
  load();
  const onResize = () => { if (mod) mod.resize(); dirty = true; };
  window.addEventListener('resize', onResize);

  function theme() {
    const now = performance.now();
    if (colours && now - colourAt < 1000) return colours;
    colourAt = now;
    const key = VAR('--canvas') + VAR('--accent');
    colours = { key, rule: VAR('--rule-strong'), faint: VAR('--ink-faint'), mono: VAR('--mono'), accent: VAR('--accent'),
      c1: VAR('--c1'), c2: VAR('--c2'), c3: VAR('--c3'), c4: VAR('--c4'), c5: VAR('--c5'), c6: VAR('--c6'), c7: VAR('--c7'),
      muted: VAR('--ink-muted'), bad: VAR('--bad'), warn: VAR('--warn') };
    if (mod && paletteKey !== key) {
      paletteKey = key;
      mod.setPalette({ background: null, grid: VAR('--rule-strong'), gridSoft: VAR('--rule') });
      mod.setOverlayColors({ trail: colours.c1, path: colours.muted, target: colours.c2, zones: colours.warn, force: colours.bad });
    }
    return colours;
  }

  function sample(s) {
    const tr = s.trace || {}, d = s.dashboard || {}, joints = d.joints || [];
    const t = typeof s.t === 'number' ? s.t : Date.now() / 1000;
    if (t <= ring.last() + 0.03) return;                              // the same snapshot again
    const v = new Array(CH.total).fill(null);
    if (tr.ee) for (let k = 0; k < 3; k++) v[CH.ee + k] = tr.ee[k];
    if (tr.target) for (let k = 0; k < 3; k++) v[CH.tg + k] = tr.target[k];
    v[CH.err] = tr.error == null ? null : tr.error * 1000;
    const f = tr.force; v[CH.force] = f ? Math.hypot(f[0], f[1], f[2]) : null;
    joints.slice(0, MAXJ).forEach((j, k) => { v[CH.pos + k] = j.position; v[CH.cmd + k] = j.commanded; v[CH.tau + k] = j.torque; });
    ring.push(t, v); dirty = true;
  }

  function flat(idx, base) {
    const out = [];
    for (const i of idx) {
      const x = ring.c[base][i], y = ring.c[base + 1][i], z = ring.c[base + 2][i];
      if (x === x && y === y && z === z) out.push(x, y, z);
    }
    return out;
  }

  function drawScene(s) {
    if (!mod) return;
    theme();
    const idx = ring.span(sec), tr = s.trace || {};
    mod.setLine('trail', flat(idx, CH.ee));
    mod.setLine('path', flat(idx, CH.tg));
    mod.setTarget(tr.target || null);
    const c = (s.safety && s.safety.cartesian) || null;
    let box = null;
    if (c && ['x', 'y', 'z'].some(a => c[a] && c[a].enabled)) {
      const ax = a => c[a] && c[a].enabled ? [c[a].min, c[a].max] : [-1.5, 1.5];
      const x = ax('x'), y = ax('y'), z = ax('z');
      box = { min: [x[0], y[0], z[0]], max: [x[1], y[1], z[1]] };
    }
    mod.setZones(box);
    const f = tr.force, mag = f ? Math.hypot(f[0], f[1], f[2]) : 0;
    mod.setForce(tr.ee || [0, 0, 0], f && tr.ee && mag >= 0.5 ? f : null, Math.min(0.4, mag * 0.02));
  }

  function drawPlots(s) {
    const col = theme(), idx = ring.span(sec);
    plots[0].draw(ring, idx, [0, 1, 2].map(k => ({ ch: CH.ee + k, colour: 'c' + (k + 1) })).concat(
      [0, 1, 2].map(k => ({ ch: CH.tg + k, colour: 'muted', dashed: true, alpha: .9 }))), sec, col);
    const nj = ((s.dashboard && s.dashboard.joints) || []).length;
    if (nj && joint >= nj) joint = 0;
    const sig = String(nj);
    if (jSeg.__sig !== sig) {
      jSeg.__sig = sig; jSeg.textContent = '';
      for (let k = 0; k < Math.min(nj, MAXJ); k++) jSeg.append(h('button', { type: 'button', text: 'J' + (k + 1), onclick: () => { joint = k; store.set('fic-live-joint', String(k)); paintJ(); dirty = true; } }));
    }
    paintJ();
    plots[1].draw(ring, idx, [{ ch: CH.pos + joint, colour: 'c1' }, { ch: CH.cmd + joint, colour: 'c2', dashed: true }], sec, col);
    const tau = []; for (let k = 0; k < Math.min(nj, MAXJ); k++) tau.push({ ch: CH.tau + k, colour: 'c' + (k % 7 + 1), alpha: .85 });
    plots[2].draw(ring, idx, tau, sec, col);
    const e = ring.span(10).map(i => ring.c[CH.err][i]).filter(v => v === v);
    const last = e.length ? e[e.length - 1] : null;
    setText(errNum, last == null ? '—' : fmt(last, last < 10 ? 1 : 0));
    errTrend.replaceChildren(e.length > 1 ? spark(e.slice(-100)) : '');
    const hist = window.FIC.history;
    teleRate.replaceChildren(spark(hist.rate)); teleForce.replaceChildren(spark(hist.force));
  }
  function paintJ() { let k = 0; for (const b of jSeg.children) b.setAttribute('aria-pressed', String(k++ === joint)); }

  const timer = setInterval(() => {
    if (document.hidden || !lastSnap || !dirty) return;
    const now = performance.now();
    if (now - lastPlot >= 1000 / PLOT_HZ - 4) { lastPlot = now; drawPlots(lastSnap); }
    if (now - lastScene >= 1000 / SCENE_HZ - 4) { lastScene = now; drawScene(lastSnap); }
    if (now - lastPlot < 5) dirty = false;
  }, 1000 / 30);

  return {
    update(s) {
      lastSnap = s; sample(s);
      if (robotKey !== null && !loading && keyOf(s) !== robotKey) load();   // another robot selected: its model, not the old one
      if (mod) { mod.setJoints(s.joints); mod.setGripper(s.gripper); }
      const tr = s.trace || {};
      setText(srcNote, tr.target ? 'target from ' + (tr.source || '—') : 'no commanded target yet');
    },
    dispose() { dead = true; clearInterval(timer); window.removeEventListener('resize', onResize); if (mod) mod.unmount(); },
  };
};
})();
