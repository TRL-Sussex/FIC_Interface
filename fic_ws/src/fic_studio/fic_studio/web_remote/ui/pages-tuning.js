/* Tuning Panel: the controller parameters in config.xml, per config, mode and
   joint or axis, laid out like the classic page. The force profile redraws
   while you edit (a field, a dragged handle, the mouse wheel for K0) before
   anything is saved, and the live operating point (tracking error against the
   commanded effort, or Cartesian error against the wrench) moves on it from the
   Studio's own telemetry, with a short trail and the three feedback bars.
   Reading, validating, the curve and writing the XML are all done by the
   Studio (tuning_info / tuning_get / tuning_plot / tuning_save / tuning_profile
   / tuning_xml, through its xml_manager); this page draws what it describes.
   A live trial (tuning_trial) runs a copy of the config on the robot and
   applies each change to it as it is made, a moment after the last keystroke
   or when a dragged handle is let go; Keep writes it over the saved config,
   Discard puts the robot back. */
(() => {
'use strict';
const { $, h, fmt, setText, setSelect, setSeg, section, pageTitle, cmd, toast, inlineConfirm } = window.FIC;
const GROUPS = [['controller', 'Controller', 'Force law and saturation'], ['planner', 'Planner', 'Trajectory generation'], ['admittance', 'Admittance', 'Response to contact force']];
const NS = 'http://www.w3.org/2000/svg';
const AXIS_IDX = { x: 0, y: 1, z: 2, q: 3 };
const TRAIL = 24;                         // ~2.4 s of operating-point history, as the classic canvas
const W = 1000, H = 340, PL = 58, PR = 16, PT = 16, PB = 40;
const GRAB = 18;
const APPLY_MS = 450;
const HAPFIC = 100;                       // HAP-FIC's own tuning block: not a control mode, used by 6-8 with HAP-FIC on                     // a live trial applies this long after the last edit                          // screen px from a handle that still picks it up
let clipSeq = 0;

window.FIC.pages.tuning = function buildTuning(root) {
  const st = { info: null, sel: { profile: 'default', mode: 1, joint: 0, axis: 'x', layer: 'joint' }, data: null, edits: {}, plot: null, point: null, trail: [], inflight: false, again: false, stamp: '', view: null, drag: null, hot: null, moved: false, trial: null };
  const toolbar = h('div', { class: 'toolbar tn-sel' });
  const form = h('div', { id: 'tn-form' });
  const plotBox = h('div', { id: 'tn-plot', class: 'tn-plotbox' });
  const statsBox = h('div', { class: 'stats tn-stats' });
  const feed = h('div', { class: 'tn-feed' });
  const readout = h('div', { class: 'num faint tn-readout', id: 'tn-readout', text: 'Δx --  ·  F --  ·  dev --' });
  const applied = h('div', { class: 'faint tn-applied', id: 'tn-applied' });
  const askHost = h('div'), addHost = h('div'), xmlHost = h('div');
  const err = h('div', { class: 'bad-line' });
  const bar = h('div', { class: 'btn-row sticky-actions', hidden: true });
  const profSel = h('select', { class: 'select', 'aria-label': 'Config' }), modeSel = h('select', { class: 'select', 'aria-label': 'Control mode' });
  const jointSel = h('select', { class: 'select', 'aria-label': 'Joint' }), axisSel = h('select', { class: 'select', 'aria-label': 'Axis' });
  const layerSel = h('select', { class: 'select', 'aria-label': 'HAP-FIC layer' });
  const typeNote = h('span', { class: 'tn-type' }), infoNote = h('span', { class: 'faint', style: 'font-size:var(--fs-m)' });
  const jointField = h('div', { class: 'field' }, h('label', { text: 'Joint ID' }), jointSel), axisField = h('div', { class: 'field' }, h('label', { text: 'Axis' }), axisSel);
  const layerField = h('div', { class: 'field', hidden: true }, h('label', { text: 'Layer' }), layerSel);
  const activeNote = h('span', { class: 'faint', style: 'font-size:var(--fs-m)' });
  toolbar.append(
    h('div', { class: 'field' }, h('label', { text: 'Mode' }), modeSel), h('div', { class: 'field' }, h('label', { text: ' ' }), h('span', { class: 'tn-typeinfo' }, typeNote, infoNote)), layerField, jointField, axisField,
    h('div', { class: 'field' }, h('label', { text: 'Config' }), profSel),
    h('div', { class: 'btn-row tn-cfgbtns' },
      h('button', { class: 'btn btn-primary', type: 'button', text: '+ Config', title: 'Create a new named config for this mode by cloning the current one', onclick: openAdd }),
      h('button', { class: 'btn btn-danger', type: 'button', text: '- Config', title: 'Delete the selected config (the default config stays)', onclick: askDelete }),
      h('button', { class: 'btn', type: 'button', text: 'Make active', title: 'Use this config for control now', onclick: makeActive })));
  const cfgBtns = toolbar.querySelector('.tn-cfgbtns');
  const saveBtn = h('button', { class: 'btn btn-primary', type: 'button', text: 'Save to XML', onclick: askSave });
  bar.append(saveBtn, h('button', { class: 'btn', type: 'button', text: 'Revert', onclick: () => guard(() => { st.edits = {}; render(); replot(); }, true) }), h('span', { class: 'faint', id: 'tn-n', style: 'font-size:var(--fs-m)' }));
  const headSave = h('button', { class: 'btn btn-primary', type: 'button', text: 'Save to XML', onclick: () => { if (dirty()) askSave(); else toast('Nothing changed'); } });
  const trialBtn = h('button', { class: 'btn', type: 'button', text: 'Live trial', title: 'Try changes on the robot as you make them, on a copy of this config: the saved config is untouched until you keep it', onclick: askTrial });
  const head = h('div', { class: 'tn-head' },
    trialBtn,
    h('div', { class: 'spacer' }),
    h('button', { class: 'btn', type: 'button', text: 'Load from XML', title: 'Discard edits and read config.xml again', onclick: () => guard(() => load(), false) }),
    headSave,
    h('button', { class: 'btn', type: 'button', text: 'View XML', title: 'View and edit the raw config.xml block for this robot', onclick: openXml }));
  // the live trial banner: what the robot runs, the last apply, and the two ways out
  const trialWhat = h('span'), trialNote = h('div', { class: 'tn-trial-note num faint' });
  const keepBtn = h('button', { class: 'btn btn-solid', type: 'button', onclick: askKeep });
  const discardBtn = h('button', { class: 'btn btn-danger', type: 'button', onclick: askDiscard });
  const trialBar = h('div', { class: 'tn-trial', role: 'status', hidden: true },
    h('div', { class: 'tn-trial-h' }, h('span', { class: 'tn-trial-dot' }), h('b', { text: 'LIVE TRIAL' }), trialWhat),
    h('div', { class: 'btn-row' }, keepBtn, discardBtn), trialNote);
  root.append(...pageTitle('Tuning Panel', 'Controller parameters from config.xml. Save writes the file and tells the controller to reload.'),
    head, trialBar, toolbar, h('div', { style: 'margin-top:8px' }, activeNote), addHost, err, askHost, xmlHost,
    h('div', { class: 'tn-plotlabel faint', text: 'Force profile — drag handles · scroll = K₀ · the curve follows your edits before you save' }),
    plotBox, statsBox, feed, readout, applied,
    bar,                                          // below the plot: appearing on the first edit, it must not move the plot under a drag
    h('div', { style: 'margin-top:var(--s6)' }, form));
  window.FIC.onSave = () => { if (live()) applyNow(); else if (dirty()) askSave(); else toast('Nothing changed'); };

  const dirty = () => Object.keys(st.edits).length > 0;
  function guard(go, silent) {
    if (!dirty() || silent) { go(); return; }
    const ask = () => inlineConfirm(askHost, { title: 'Discard unsaved edits?', lines: Object.keys(st.edits), label: 'Discard and continue', danger: true, onConfirm: () => { st.edits = {}; go(); } });
    if (live()) applyNow().then(() => { if (dirty()) ask(); else go(); });   // a trial applies them first
    else ask();
  }
  const modeOf = () => st.info.modes.find(x => x.id === st.sel.mode) || st.info.modes[0];
  /** A joint set: a joint mode, or HAP-FIC's joint layer (its Cartesian layer is a set of axes). */
  const jointSet = () => modeOf().type === 'joint' || (modeOf().type === 'hapfic' && st.sel.layer === 'joint');
  const target = () => {
    const t = { mode: st.sel.mode, profile: st.sel.profile };
    if (modeOf().type === 'hapfic') t.layer = st.sel.layer;
    if (jointSet()) t.joint = st.sel.joint; else t.axis = st.sel.axis;
    return t;
  };
  const scoped = () => ['elbow', 'postural'].includes(st.sel.axis) && !jointSet();
  /** Edits here go to the robot as they are made: a trial is running and this is its config (elbow and postural are robot-wide, never in a trial). */
  const live = () => !!st.trial && st.sel.profile === st.trial.name && !scoped();

  // ── load ──
  async function init() {
    const r = await cmd('tuning_info', {}, true);
    if (!r.ok) { setText($('#lede'), r.message || 'Tuning is not available on this Studio.'); return; }
    st.info = r; st.trial = r.trial || null;
    const s = window.FIC.state();
    st.sel.profile = st.trial ? st.trial.name : r.active && r.profiles.includes(r.active) ? r.active : 'default';
    st.sel.mode = s && s.mode ? s.mode.index + 1 : 1;
    // HAP-FIC on in a Cartesian planner or admittance mode runs its own tuning, not the mode's: open on that
    if (st.sel.mode >= 6 && st.sel.mode <= 8 && s.options && s.options.hapfic && s.options.hapfic.value) st.sel.mode = HAPFIC;
    selectors(); load();
  }
  async function load() {
    setText($('#tn-n'), ''); st.trail = []; st.point = null;
    const r = await cmd('tuning_get', target(), true);
    if (!r.ok) { setText(err, r.message || 'Could not read config.xml'); return; }
    setText(err, ''); st.data = r; st.edits = {}; st.plot = r.plot; render(); drawPlot();
  }
  function selectors() {
    const i = st.info;
    setSelect(profSel, i.profiles.map(p => ({ value: p, label: p + (p === i.active ? '  (active)' : '') })), st.sel.profile, v => guard(() => { st.sel.profile = v; load(); }));
    setSelect(modeSel, i.modes.map(m => ({ value: String(m.id), label: m.type === 'hapfic' ? m.name + ' (modes 6-8 with HAP-FIC on)' : 'Mode ' + m.id + ': ' + m.name })), String(st.sel.mode), v => guard(() => { st.sel.mode = Number(v); selectors(); load(); }));
    setSelect(jointSel, Array.from({ length: i.joints }, (_, k) => ({ value: String(k), label: String(k) })), String(st.sel.joint), v => guard(() => { st.sel.joint = Number(v); load(); }));
    const m = modeOf(), isJoint = jointSet(), hap = m.type === 'hapfic';
    layerField.hidden = !hap;
    setSelect(layerSel, [{ value: 'joint', label: 'Joint layer' }, { value: 'cartesian', label: 'Cartesian layer' }], st.sel.layer, v => guard(() => { st.sel.layer = v; selectors(); load(); }));
    jointField.hidden = !isJoint; axisField.hidden = isJoint;
    setSelect(axisSel, i.axes.map(a => ({ value: a, label: a })), st.sel.axis, v => guard(() => { st.sel.axis = v; selectors(); load(); }));
    setText(typeNote, 'Type: ' + (hap ? 'HAP-FIC, ' + (isJoint ? 'joint layer' : 'Cartesian layer') : isJoint ? 'Joint' : 'Cartesian'));
    setText(infoNote, hap ? '  (used instead of the mode’s own tuning whenever HAP-FIC is on)' : m.planner ? '  (HAS damping_ratio)' : m.admittance ? '  (HAS Kv damping)' : '  (NO damping — use lower stiffness!)');
    setText(activeNote, 'Active for control: ' + (i.active || '—') + (st.sel.profile !== i.active ? '  ·  editing ' + st.sel.profile : ''));
    trialState();
  }
  /** Lock what a trial must not change, and say what the robot runs. */
  function trialState() {
    const t = st.trial;
    trialBar.hidden = !t; profSel.disabled = !!t; trialBtn.disabled = !!t;
    for (const b of cfgBtns.querySelectorAll('button')) b.disabled = !!t;
    if (!t) return;
    setText(trialWhat, 'The robot runs “' + t.name + '”: each change is applied as you make it. The saved “' + t.source + '” is not touched.');
    setText(keepBtn, 'Keep: save as “' + t.source + '”'); setText(discardBtn, 'Discard: back to “' + t.back + '”');
  }

  // ── form ──
  const shown = f => f.key in st.data.values;
  function value(key) { return key in st.edits ? st.edits[key] : st.data.values[key]; }
  const asList = v => Array.isArray(v) ? v.map(Number) : String(v).split(/[,;]/).map(x => parseFloat(x)).filter(x => !isNaN(x));
  function input(f) {
    const v = value(f.key), set = x => { st.edits[f.key] = x; if (JSON.stringify(x) === JSON.stringify(st.data.values[f.key])) delete st.edits[f.key]; touch(); };
    if (f.type === 'bool') {
      const b = h('input', { type: 'checkbox', 'data-f': f.key }); b.checked = !!v; b.addEventListener('change', () => set(b.checked));
      return h('label', { class: 'toggle' }, b, h('span', { class: 'track' }));
    }
    if (f.type === 'choice') {
      const sel = h('select', { class: 'select', 'data-f': f.key }, h('option', { value: '0', text: 'No' }), h('option', { value: '1', text: 'Yes' }));
      sel.value = String(v); sel.addEventListener('change', () => set(Number(sel.value))); return sel;
    }
    const el = h('input', { class: 'input' + (f.type === 'list' ? '' : ' num'), 'data-f': f.key, type: f.type === 'list' ? 'text' : 'number', step: f.type === 'int' ? '1' : 'any', 'aria-label': f.label });
    if (f.type !== 'list') { el.min = f.min; el.max = f.max; }
    el.value = f.type === 'list' ? v.join(', ') : String(v);
    el.addEventListener('input', () => { if (el.value === '') return; set(f.type === 'list' ? el.value : parseFloat(el.value)); });
    return el;
  }
  function render() {
    form.textContent = '';
    const d = st.data; if (!d) return;
    const t = d.target;
    setText($('#lede'), st.info.robot + ' · ' + (scoped() ? st.sel.axis + ' (robot-wide, shared by all Cartesian modes)' : st.sel.profile + ' · ' + (modeOf().type === 'hapfic' ? 'HAP-FIC' : 'mode ' + st.sel.mode) + ' · ' + (t.kind === 'joint' ? 'joint ' + t.joint : 'axis ' + t.axis)));
    if (d.fallback) form.append(h('p', { class: 'note-warn' }, 'The config “' + st.sel.profile + '” has no block for this mode yet, so these are the default values. ' + (live() ? 'The first change creates it in the trial.' : 'Saving creates it.')));
    if (d.hapfic_layers) form.append(h('p', { class: 'note-warn' }, 'HAP-FIC has no tuning of its own on this robot yet, so it runs on mode 4’s joints and mode 6’s Cartesian axes: these are those values. The first change gives HAP-FIC its own, copied from them, and leaves modes 4 and 6 as they are.'));
    if (st.trial && scoped()) form.append(h('p', { class: 'note-warn' }, st.sel.axis + ' is robot-wide, not part of a config, so it is not in the live trial: Save to XML writes it directly.'));
    for (const [g, title, hint] of GROUPS) {
      const fields = d.fields.filter(f => f.group === g && shown(f));
      if (!fields.length) continue;
      form.append(h('div', { class: 'fgroup' }, h('div', { class: 'section-head' }, h('h2', { text: title.toUpperCase() }), h('span', { class: 'aside', text: hint })),
        h('div', { class: 'fgrid' }, ...fields.map(f => h('div', { class: 'frow' + (f.key in st.edits ? ' edited' : ''), 'data-row': f.key },
          h('label', { text: f.label }), input(f), h('span', { class: 'unit faint', text: unitFor(f) }))))));
    }
    touch(true);
  }
  const unitFor = f => {
    const joint = (st.data.target || {}).kind === 'joint';
    if (f.key === 'stiffness' || f.key === 'planner_stiffness' || f.key === 'planner_stiffness_p') return joint ? 'Nm/rad' : 'N/m';
    if (f.key === 'force_max') return joint ? 'Nm' : 'N';
    if (f.key === 'error_max') return joint ? 'rad' : 'm';
    if (f.key === 'velocity' || f.key === 'planner_vel_desired') return joint ? 'rad/s' : 'm/s';
    return f.unit || '';
  };
  /** Put edited values back into the fields that are already on screen (a dragged handle moved them). */
  function syncForm() {
    for (const el of form.querySelectorAll('[data-f]')) {
      const key = el.dataset.f, v = value(key);
      if (el.type === 'checkbox') el.checked = !!v; else if (document.activeElement !== el) el.value = Array.isArray(v) ? v.map(x => +x.toFixed(4)).join(', ') : String(v);
    }
  }
  function touch(quiet) {
    for (const row of form.querySelectorAll('[data-row]')) row.classList.toggle('edited', row.dataset.row in st.edits);
    bar.hidden = !dirty() || live(); headSave.hidden = live();
    setText($('#tn-n'), Object.keys(st.edits).length + ' parameter' + (Object.keys(st.edits).length === 1 ? '' : 's') + ' changed, not saved');
    if (quiet) return;
    replot();
    if (live() && !st.drag) scheduleApply();       // a dragged handle applies when it is let go
  }

  // ── live trial ──
  let applyTimer = null, applyChain = Promise.resolve();
  function scheduleApply() {
    clearTimeout(applyTimer);
    applyTimer = setTimeout(applyNow, APPLY_MS);
    trialNote.classList.remove('bad'); setText(trialNote, 'Changed: applying in a moment…');
  }
  /** Apply the pending edits to the trial now; one at a time, in order. Resolves when they are on the robot (or refused). */
  function applyNow() {
    clearTimeout(applyTimer); applyTimer = null;
    applyChain = applyChain.then(applyOnce, applyOnce);
    return applyChain;
  }
  async function applyOnce() {
    if (!live() || !dirty()) return;
    const sent = Object.assign({}, st.edits), data = st.data;
    trialNote.classList.remove('bad'); setText(trialNote, 'Applying…');
    const r = await cmd('tuning_trial', Object.assign({ op: 'apply' }, target(), { values: sent }), true);
    if (st.data !== data) return;                 // the page has moved on to another set
    if (!r.ok) { trialNote.classList.add('bad'); setText(trialNote, (r.message || 'The Studio refused the change') + '. The robot keeps the last applied values.'); return; }
    data.values = r.values;                       // the trial holds them now: they are no longer edits
    for (const k of Object.keys(sent)) if (JSON.stringify(st.edits[k]) === JSON.stringify(sent[k])) delete st.edits[k];
    touch(true);
    setText(trialNote, 'Applied ' + new Date().toTimeString().slice(0, 8) + ' · ' + r.applied);
  }
  function askTrial() {
    const src = st.sel.profile, n = Object.keys(st.edits).length;
    inlineConfirm(askHost, { title: 'Start a live trial of “' + src + '”?',
      lines: ['The robot switches to a copy, “' + src + '~trial”, and each change you make is applied to it straight away.',
        'The saved “' + src + '” is not touched. Keep writes the trial over it; Discard puts the robot back on “' + (st.info.active || src) + '”.']
        .concat(n && !scoped() ? [n + ' unsaved change' + (n === 1 ? ' is' : 's are') + ' applied to the copy first.'] : []),
      label: 'Start live trial',
      onConfirm: async () => {
        const kept = scoped() ? {} : st.edits;
        const r = await cmd('tuning_trial', { op: 'start', profile: src, confirm: true });
        if (!r.ok) return;
        trialEnded(r, r.trial.name);
        await load();
        if (Object.keys(kept).length) { st.edits = kept; render(); touch(); }
        toast('Live trial started: changes now go to the robot');
      } });
  }
  function askKeep() {
    const t = st.trial;
    inlineConfirm(askHost, { title: 'Keep the trial as “' + t.source + '”?',
      lines: ['The trial values are written over the saved “' + t.source + '” in config.xml, for every mode the trial changed, and “' + t.source + '” becomes the active config.'],
      label: 'Keep and save',
      onConfirm: async () => {
        await applyNow();                         // the last edits go in before it is copied
        if (dirty() && live()) { setText(err, 'The last change was refused, so the trial was not kept. Fix or revert it first.'); return; }
        const r = await cmd('tuning_trial', { op: 'keep', confirm: true });
        if (r.ok) { trialEnded(r, t.source); load(); toast('Trial kept: saved as “' + t.source + '”'); }
      } });
  }
  function askDiscard() {
    const t = st.trial;
    inlineConfirm(askHost, { title: 'Discard the trial?', lines: ['The robot goes back to “' + t.back + '”. Nothing from the trial is saved.'],
      label: 'Discard trial', danger: true,
      onConfirm: async () => {
        clearTimeout(applyTimer); applyTimer = null; st.edits = {};
        const r = await cmd('tuning_trial', { op: 'discard', confirm: true });
        if (r.ok) { trialEnded(r, t.source); load(); toast('Trial discarded: the robot is back on “' + t.back + '”'); }
      } });
  }
  /** A trial started or ended: the configs, the active one and the one being edited follow the Studio. */
  function trialEnded(r, profile) {
    st.trial = r.trial || null; st.info.profiles = r.profiles; st.info.active = r.active; st.sel.profile = profile;
    trialNote.classList.remove('bad'); setText(trialNote, '');
    selectors();
  }
  /** Ask the Studio for the curve of the edited values now; one request at a time, the latest edits always win. */
  async function replot() {
    if (!st.data) return;
    if (st.inflight) { st.again = true; return; }
    st.inflight = true;
    try {
      const r = await cmd('tuning_plot', Object.assign(target(), { values: st.edits }, st.view ? { view: [st.view.xr, st.view.yr] } : {}), true);
      if (r.ok) { setText(err, ''); st.plot = r.plot; drawPlot(); } else setText(err, r.message || '');
    } finally {
      st.inflight = false;
      if (st.again) { st.again = false; replot(); }
    }
  }

  // ── plot ──
  const el = (n, a, text) => { const e = document.createElementNS(NS, n); for (const k in a) e.setAttribute(k, a[k]); if (text != null) e.textContent = text; return e; };
  // One svg for the life of the page, redrawn in place, so a drag keeps its pointer capture while new curves come back.
  const clipId = 'tn-clip-' + (++clipSeq);
  const svg = el('svg', { viewBox: '0 0 ' + W + ' ' + H, class: 'plot big', role: 'img', 'aria-label': 'Force against error' });
  const clip = el('clipPath', { id: clipId }); clip.append(el('rect', { x: PL, y: PT, width: W - PL - PR, height: H - PT - PB }));
  const defs = el('defs', {}); defs.append(clip);
  const baseG = el('g', {}), opsG = el('g', {}), hdsG = el('g', {});
  svg.append(defs, baseG, opsG, hdsG);
  /** Value to viewBox mapping. The axes follow the curve, except while a handle is dragged: then they hold still. */
  const geom = () => {
    const p = st.plot && st.plot.sym; if (!p) return null;
    const { xr, yr } = st.view || p;
    return { p, xr, yr, X: v => PL + (v + xr) / (2 * xr) * (W - PL - PR), Y: v => H - PB - (v + yr) / (2 * yr) * (H - PB - PT) };
  };
  /** viewBox units per screen pixel (1 until the plot is laid out). */
  const scale = () => { const w = svg.getBoundingClientRect().width; return w > 50 ? W / w : 1; };
  function handles() {
    if (!st.data) return [];
    const xb = asList(value('error_max')), fs = asList(value('force_max')), k0 = Number(value('stiffness')), sr = Number(value('saturation_ratio'));
    const out = [];
    xb.forEach((x, i) => {
      out.push({ id: 'xb' + i, kind: 'xb', i, x, y: fs[i] });
      const sx = sr * x; out.push({ id: 'sr' + i, kind: 'sr', i, x: sx, y: i === 0 ? k0 * sx : fs[i - 1] });
    });
    return out;
  }
  function drawPlot() {
    const g = geom();
    if (!g) { plotBox.replaceChildren(h('div', { class: 'empty', text: 'No profile for these values.' })); return; }
    if (svg.parentNode !== plotBox) plotBox.replaceChildren(svg);
    const { p, xr, yr, X, Y } = g, out = [];
    for (let i = 0; i <= 4; i++) {
      const gx = PL + i / 4 * (W - PL - PR), gy = PT + i / 4 * (H - PT - PB);
      out.push(el('line', { x1: gx, x2: gx, y1: PT, y2: H - PB, class: 'gl' }), el('line', { x1: PL, x2: W - PR, y1: gy, y2: gy, class: 'gl' }));
    }
    out.push(el('line', { x1: X(0), x2: X(0), y1: PT, y2: H - PB, class: 'ax' }), el('line', { x1: PL, x2: W - PR, y1: Y(0), y2: Y(0), class: 'ax' }));
    for (let i = -4; i <= 4; i++) {
      const vx = i * xr / 4, vy = i * yr / 4;
      if (i) { out.push(el('text', { x: X(vx), y: H - PB + 16, class: 'tx', 'text-anchor': 'middle' }, fmt(vx, 3)), el('text', { x: PL - 8, y: Y(vy) + 3, class: 'tx', 'text-anchor': 'end' }, fmt(vy, 1))); }
    }
    out.push(el('text', { x: 14, y: (PT + H - PB) / 2, class: 'tx', transform: 'rotate(-90 14 ' + ((PT + H - PB) / 2) + ')', 'text-anchor': 'middle' }, 'effort'),
      el('text', { x: (PL + W - PR) / 2, y: H - 6, class: 'tx', 'text-anchor': 'middle' }, 'error  (rad / m)'));
    // the x_b and S·x_b lines sit on the handles, so they move with a drag before the new curve is back
    const curves = el('g', { 'clip-path': 'url(#' + clipId + ')' });
    for (const hd of handles()) for (const sx of [hd.x, -hd.x]) curves.append(el('line', { x1: X(sx), x2: X(sx), y1: PT, y2: H - PB, class: hd.kind === 'xb' ? 'satline' : 'srline' }));
    const path = (xs, ys) => 'M' + xs.map((x, i) => X(x).toFixed(1) + ' ' + Y(Math.max(-yr, Math.min(yr, ys[i]))).toFixed(1)).join(' L');
    if (p.convergence.x.length) curves.append(el('path', { d: path(p.convergence.x, p.convergence.y), class: 'conv' }));
    curves.append(el('path', { d: path(p.x, p.linear), class: 'lin' }), el('path', { d: path(p.x, p.y), class: 'curve' }));
    out.push(curves);
    // legend, top left, and the K0 / N_S note, top right
    const leg = [['FIC force curve', 'curve'], ['Linear stiffness K₀', 'lin'], ['Convergence phase', 'conv'], ['x_b boundaries', 'satline']];
    leg.forEach(([t, c], i) => { out.push(el('line', { x1: PL + 12, x2: PL + 34, y1: PT + 14 + i * 15, y2: PT + 14 + i * 15, class: c + ' lg' }), el('text', { x: PL + 40, y: PT + 17 + i * 15, class: 'tx' }, t)); });
    out.push(el('text', { x: W - PR - 6, y: PT + 17, class: 'tx', 'text-anchor': 'end' }, 'K₀=' + fmt(st.plot.stats.k0, 0) + '  N_S=' + (st.plot.stats || {}).n_s + '  scroll=±10'));
    baseG.replaceChildren(...out);
    drawPlotPoint(); drawHandles(); drawStats();
  }
  /** The handles, from the values being edited (not the last curve), so a dragged one stays under the pointer. */
  function drawHandles() {
    const g = geom(); if (!g) return;
    const { X, Y } = g, r = Math.max(6.5, 6 * scale()), dots = [], labels = [];
    for (const hd of handles()) {        // the S·x_b class is 'ratio': 'sr' is the stylesheet's screen-reader-only class
      const cy = Y(clampY(hd.y)), cls = 'hd ' + (hd.kind === 'xb' ? 'xb' : 'ratio') + (hd.id === st.drag ? ' on' : hd.id === st.hot ? ' hot' : '');
      for (const sg of [1, -1]) dots.push(el('circle', { cx: X(sg * hd.x), cy, r: r.toFixed(1), class: cls, 'data-h': hd.id }));
      labels.push(el('text', { x: X(hd.x) + r + 4.5, y: cy + 3, class: 'tx hl' }, hd.kind === 'xb' ? 'xb' + (hd.i + 1) + '·F' + (hd.i + 1) : 'S·xb' + (hd.i + 1)));
    }
    hdsG.replaceChildren(...dots, ...labels);
  }
  /** Trail dots, the dotted link to the designed curve (|F| at this error) and the marker, as the classic canvas draws them. */
  function opMarks(X, Y) {
    const out = [], p = st.plot && st.plot.sym; if (!p || !st.point) return out;
    const tn = st.trail.length;
    st.trail.forEach((t, k) => out.push(el('circle', { cx: X(clampX(t[0])), cy: Y(clampY(t[1])), r: 2.2, class: 'trail', opacity: (0.12 + 0.68 * (k + 1) / tn).toFixed(2) })));
    const [ox, oy] = st.point, cx = clampX(ox), cy = clampY(oy), clamped = cx !== ox || cy !== oy;
    let i = 0, best = Infinity; p.x.forEach((v, k) => { const d = Math.abs(v - ox); if (d < best) { best = d; i = k; } });
    const curveY = p.y[i] || 0, frac = Math.min(1, Math.abs(oy - curveY) / Math.max(p.yr, 1e-6));
    out.push(el('line', { x1: X(cx), y1: Y(cy), x2: X(cx), y2: Y(clampY(curveY)), class: 'devline', style: 'opacity:' + (0.35 + 0.65 * frac).toFixed(2) }));
    if (clamped) out.push(el('circle', { cx: X(cx), cy: Y(cy), r: 4, class: 'op edge' }));
    else out.push(el('circle', { cx: X(cx), cy: Y(cy), r: 9, class: 'opring' }), el('circle', { cx: X(cx), cy: Y(cy), r: 5, class: 'op' }));
    return out;
  }
  const clampX = x => { const g = geom(); return g ? Math.max(-g.xr, Math.min(g.xr, x)) : x; };
  const clampY = y => { const g = geom(); return g ? Math.max(-g.yr, Math.min(g.yr, y)) : y; };
  function drawStats() {
    const s = st.plot && st.plot.stats; if (!s) return;
    const items = [['Stiffness K₀', fmt(s.k0, 0)], ['N_S', String(s.n_s)], ['x_b (all)', s.xb.map(v => v.toFixed(4)).join(',')], ['F (all)', s.f.map(v => v.toFixed(1)).join(',')], ['Ratio S', s.sr.map(v => v.toFixed(2)).join(',')], ['Conv. K', fmt(s.k_conv, 1)]];
    statsBox.replaceChildren(...items.map(([k, v]) => h('div', { class: 'stat' }, h('div', { class: 'k', text: k }), h('div', { class: 'v', text: v }))));
  }
  // Dragging a handle edits the same fields the form shows; the wheel edits K0. A press picks the nearest handle
  // within GRAB px (the dots themselves are small), and the axes hold still until the button is released.
  const toBox = ev => new DOMPoint(ev.clientX, ev.clientY).matrixTransform(svg.getScreenCTM().inverse());
  function nearest(ev) {
    const g = geom(); if (!g) return null;
    const q = toBox(ev); let best = null, bd = GRAB * scale();
    for (const hd of handles()) for (const sg of [1, -1]) {
      const d = Math.hypot(g.X(sg * hd.x) - q.x, g.Y(clampY(hd.y)) - q.y);
      if (d < bd) { bd = d; best = hd; }
    }
    return best;
  }
  svg.addEventListener('pointerdown', ev => {
    if (ev.button !== 0) return;
    const hd = nearest(ev), g = geom(); if (!hd || !g) return;
    st.drag = hd.id; st.hot = null; st.moved = false; st.view = { xr: g.xr, yr: g.yr };
    svg.setPointerCapture(ev.pointerId); svg.style.cursor = 'grabbing'; ev.preventDefault();
    drawHandles();
  });
  svg.addEventListener('pointermove', ev => {
    if (!st.drag) {                     // hover: show which handle a press would take
      const hd = ev.pointerType === 'mouse' ? nearest(ev) : null, id = hd ? hd.id : null;
      if (id !== st.hot) { st.hot = id; svg.style.cursor = id ? 'grab' : ''; drawHandles(); }
      return;
    }
    const hd = handles().find(x => x.id === st.drag), g = geom(); if (!hd || !g) return;
    const q = toBox(ev), wx = (q.x - PL) / (W - PL - PR) * 2 * g.xr - g.xr, wy = (H - PB - q.y) / (H - PB - PT) * 2 * g.yr - g.yr;
    const vx = Math.min(Math.abs(wx), g.xr), vy = Math.min(Math.abs(wy), g.yr), xs = asList(value('error_max')), n = xs.length, i = hd.i;
    if (hd.kind === 'xb') {
      const fs = asList(value('force_max'));
      const lo = i > 0 ? xs[i - 1] * 1.05 : 0.001, hi = i < n - 1 ? xs[i + 1] * 0.95 : g.xr * 0.92;
      xs[i] = +Math.max(lo, Math.min(vx, hi)).toFixed(5);
      const fl = i > 0 ? fs[i - 1] * 1.02 : 0.1, fh = i < n - 1 ? fs[i + 1] * 0.98 : 500;
      fs[i] = +Math.max(fl, Math.min(vy, fh)).toFixed(3);
      setEdit('error_max', xs); setEdit('force_max', fs);
    } else {
      setEdit('saturation_ratio', +Math.max(0.4, Math.min(vx / Math.max(xs[i], 1e-6), 0.95)).toFixed(4));
    }
    st.moved = true;
    syncForm(); drawPlot(); touch();
  });
  const endDrag = () => {
    if (!st.drag) return;
    st.drag = null; st.view = null; svg.style.cursor = '';
    if (st.moved) { replot(); if (live()) applyNow(); }   // the curve over its own range again; a trial applies now
    drawPlot();
  };
  svg.addEventListener('pointerup', endDrag); svg.addEventListener('pointercancel', endDrag); svg.addEventListener('lostpointercapture', endDrag);
  svg.addEventListener('pointerleave', () => { if (!st.drag && st.hot) { st.hot = null; svg.style.cursor = ''; drawHandles(); } });
  svg.addEventListener('wheel', ev => {
    ev.preventDefault();
    const step = ev.shiftKey ? 50 : 10, k0 = Number(value('stiffness'));
    setEdit('stiffness', Math.max(1, Math.min(k0 + (ev.deltaY < 0 ? step : -step), 5000)));
    syncForm(); drawHandles(); touch();
  }, { passive: false });
  function setEdit(key, v) { if (JSON.stringify(v) === JSON.stringify(st.data.values[key])) delete st.edits[key]; else st.edits[key] = v; }

  // ── live telemetry ──
  const bars = {};
  for (const [k, label, unit] of [['err', 'Tracking error', 'rad / m'], ['eff', 'Effort', 'Nm / N'], ['wr', 'Wrench', 'N / Nm']]) {
    const fill = h('i'), val = h('span', { class: 'num', text: '--' });
    bars[k] = { fill, val };
    feed.append(h('div', { class: 'cva' }, h('div', { class: 'cva-h' }, h('span', { class: 'eyebrow', text: label + ' · ' + unit }), val), h('div', { class: 'cva-bar' }, fill, h('b'))));
  }
  function setBar(b, v, range) {
    if (v == null || !isFinite(v)) { setText(b.val, '--'); b.fill.style.width = '0'; return; }
    const f = Math.max(-1, Math.min(1, v / Math.max(range, 1e-9)));
    setText(b.val, (v >= 0 ? '+' : '') + (Math.abs(v) < 1 ? v.toFixed(4) : v.toFixed(2)));
    b.fill.style.left = f >= 0 ? '50%' : (50 + f * 50) + '%'; b.fill.style.width = Math.abs(f) * 50 + '%'; b.fill.className = f >= 0 ? 'pos' : 'neg';
  }
  function liveUpdate(s) {
    if (!st.data || !st.info) return;
    const live = s.live || {}, isJoint = jointSet();
    let x = null, y = null, errRange = 0.1, effRange = 39, wrRange = 39;
    const em = asList(value('error_max')), fm = asList(value('force_max'));
    if (isJoint) {
      const j = st.sel.joint;
      x = (live.joint_error || [])[j]; y = (live.joint_effort || [])[j];
      errRange = Math.max(em[0] || 0.1, 1e-4); effRange = Math.max(fm[fm.length - 1] || 39, 1e-4);   // the top of the curve
      setBar(bars.err, x, errRange); setBar(bars.eff, y, effRange); setBar(bars.wr, null, 1);
    } else {
      const idx = AXIS_IDX[st.sel.axis], w = live.wrench, ce = live.cart_error;
      setBar(bars.err, null, 1); setBar(bars.eff, null, 1);
      wrRange = Math.max(fm[fm.length - 1] || 39, 1e-4);
      y = idx != null && w ? w[idx] : null; x = idx != null && ce ? ce[idx] : null;
      setBar(bars.wr, y, wrRange);
    }
    const ok = x != null && y != null && isFinite(x) && isFinite(y);
    if (ok) { st.point = [x, y]; st.trail.push([x, y]); if (st.trail.length > TRAIL) st.trail.shift(); } else { st.point = null; st.trail = []; }
    if (ok && st.plot) {
      const p = st.plot.sym, xs = p.x, ys = p.y;
      let i = 0, bd = Infinity; xs.forEach((v, k) => { const d = Math.abs(v - x); if (d < bd) { bd = d; i = k; } });
      const curveY = ys[i] != null ? ys[i] : 0;
      setText(readout, 'Δx ' + (x >= 0 ? '+' : '') + x.toFixed(4) + '  ·  F ' + (y >= 0 ? '+' : '') + y.toFixed(2) + '  ·  dev ' + ((y - curveY) >= 0 ? '+' : '') + (y - curveY).toFixed(2));
    } else setText(readout, 'Δx --  ·  F --  ·  dev --');
    if (st.plot) drawPlotPoint();
  }
  /** Move only the operating point and its trail: the rest of the plot did not change. */
  function drawPlotPoint() {
    const g = geom(); if (!g || svg.parentNode !== plotBox) return;
    opsG.replaceChildren(...opMarks(g.X, g.Y));
  }

  // ── actions ──
  function askSave() {
    setText(err, '');
    inlineConfirm(askHost, { title: 'Write ' + Object.keys(st.edits).length + ' change' + (Object.keys(st.edits).length === 1 ? '' : 's') + ' to config.xml?',
      lines: Object.keys(st.edits).map(k => k + ': ' + JSON.stringify(st.data.values[k]) + ' -> ' + JSON.stringify(st.edits[k])).concat(['The controller reloads its parameters when it is told the file changed.']),
      label: 'Save to config.xml',
      onConfirm: async () => {
        const r = await cmd('tuning_save', Object.assign(target(), { values: st.edits, confirm: true }), true);
        if (r.ok) { toast('Saved ' + r.saved); st.data.values = r.values; st.data.fallback = false; st.edits = {}; st.plot = r.plot; render(); drawPlot(); const d = new Date(); setText(applied, 'Applied ' + d.toTimeString().slice(0, 8) + ' · ' + r.saved); }
        else setText(err, (r.message || 'The Studio refused the save') + '. Nothing was written.');
      } });
  }
  async function makeActive() { const r = await cmd('set_profile', { name: st.sel.profile }); if (r.ok !== false) { st.info.active = st.sel.profile; selectors(); toast('Active config: ' + st.sel.profile); } }
  function openAdd() {
    addHost.textContent = '';
    const name = h('input', { class: 'input', type: 'text', placeholder: 'New config name', 'aria-label': 'New config name', maxlength: '48' });
    const go = h('button', { class: 'btn btn-primary', type: 'button', text: 'Create', onclick: () => {
      if (!name.value.trim()) return;
      inlineConfirm(askHost, { title: 'Create config “' + name.value.trim() + '” for mode ' + st.sel.mode + '?', lines: ['It starts as a copy of “' + st.sel.profile + '”. Other modes fall back to default until you edit them.'],
        label: 'Create config', onConfirm: async () => {
          const r = await cmd('tuning_profile', { op: 'add', name: name.value.trim(), from: st.sel.profile, mode: st.sel.mode, confirm: true });
          if (r.ok) { st.info.profiles = r.profiles; st.sel.profile = name.value.trim(); addHost.textContent = ''; selectors(); load(); toast('Config added'); }
        } });
    } });
    addHost.append(h('div', { class: 'btn-row', style: 'margin-top:16px' }, name, go, h('button', { class: 'btn', type: 'button', text: 'Cancel', onclick: () => { addHost.textContent = ''; } })));
    name.focus();
  }
  function askDelete() {
    if (st.sel.profile === 'default') { toast('The default config cannot be deleted', true); return; }
    inlineConfirm(askHost, { title: 'Delete config “' + st.sel.profile + '”?', lines: ['All its per-mode blocks are removed from config.xml.'], label: 'Delete config', danger: true,
      onConfirm: async () => {
        const r = await cmd('tuning_profile', { op: 'delete', name: st.sel.profile, confirm: true });
        if (r.ok) { st.info.profiles = r.profiles; st.sel.profile = 'default'; selectors(); load(); toast('Config deleted'); }
      } });
  }
  async function openXml() {
    xmlHost.textContent = '';
    const r = await cmd('tuning_xml', { op: 'read' }, true);
    if (!r.ok) { toast(r.message || 'Cannot read the XML', true); return; }
    const area = h('textarea', { class: 'xmlarea', spellcheck: 'false', rows: '18', 'aria-label': 'config.xml block' }); area.value = r.text;
    const note = h('div', { class: 'bad-line' });
    xmlHost.append(h('div', { class: 'section' }, h('div', { class: 'section-head' }, h('h2', { text: 'config.xml · ' + r.robot }), h('span', { class: 'aside', text: r.path })), area, note,
      h('div', { class: 'btn-row', style: 'margin-top:12px' },
        h('button', { class: 'btn btn-primary', type: 'button', text: 'Save this block', onclick: () => inlineConfirm(xmlHost, { title: 'Replace the ' + r.robot + ' block in config.xml?', lines: ['Written exactly as shown, after it parses as XML.'], label: 'Replace block', danger: true,
          onConfirm: async () => { const w = await cmd('tuning_xml', { op: 'save', text: area.value, confirm: true }, true); if (w.ok) { toast('config.xml updated'); xmlHost.textContent = ''; load(); } else setText(note, w.message || 'Refused'); } }) }),
        h('button', { class: 'btn', type: 'button', text: 'Close', onclick: () => { xmlHost.textContent = ''; } }))));
  }
  init();
  return s => liveUpdate(s);
};
})();
