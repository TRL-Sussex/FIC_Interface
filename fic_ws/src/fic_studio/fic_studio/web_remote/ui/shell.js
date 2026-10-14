/* The window chrome, laid out like the classic Studio: menu bar (File View
   Robot Simulation Tools Help), toolbar (target, Start, EXP BAG PLT VIEW,
   station and robot, E-STOP), the page list on the left, and the
   status bar along the bottom. Every control sends a command; every figure
   comes from the snapshot. */
(() => {
'use strict';
const { $, h, fmt, setText, setSeg, setSelect, cmd, post, toast, armed, store, inlineConfirm } = window.FIC;
const FIC = window.FIC;
const TARGET_LABELS = { simulation: 'Simulation', hardware: 'Hardware', vr: 'VR' };
const TARGET_SHORT = { simulation: 'SIM', hardware: 'HW', vr: 'VR' };
let pendingSession = null;      // { op, until } between a confirmed click and the state catching up
let sessionArm = null;
let EXTENSIONS = [];

// ── View preferences: the classic View menu's toggles, kept in this browser ─
const VIEW = Object.assign({ toolbar: true, bar: true, side: true, status: true, compact: false, minimal: false, scale: 100 },
  (() => { try { return JSON.parse(store.get('fic-view') || '{}'); } catch (e) { return {}; } })());
if (matchMedia('(max-width: 860px)').matches) VIEW.side = false;      // on a phone the page list is a menu, closed until asked
window.FIC.applyRootFont = () => { document.documentElement.style.fontSize = (16 * VIEW.scale / 100 * (window.FIC.fontK || 1)) + 'px'; };
function applyView() {
  const b = document.body.classList;
  b.toggle('no-toolbar', !VIEW.toolbar || VIEW.minimal);
  b.toggle('no-bar', !VIEW.bar);
  b.toggle('side-collapsed', !VIEW.side || VIEW.minimal);
  b.toggle('no-status', !VIEW.status || VIEW.minimal);
  b.toggle('compact', !!VIEW.compact);
  FIC.applyRootFont();
  store.set('fic-view', JSON.stringify(VIEW));
  $('#nav-toggle').setAttribute('aria-expanded', String(VIEW.side && !VIEW.minimal));
  if (FIC.measure) FIC.measure();
}
const flip = key => () => { VIEW[key] = !VIEW[key]; if (key === 'minimal' && VIEW.minimal) VIEW.compact = VIEW.compact; applyView(); };

// ── Menus ─────────────────────────────────────────────────────────────────
const go = page => () => { location.hash = '#/' + page; };
const need = (fn) => async () => { const r = await fn(); return r; };
function download(name, mime, text) {
  const url = URL.createObjectURL(new Blob([text], { type: mime }));
  const a = h('a', { href: url, download: name }); document.body.append(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(url), 2000);
}
async function menuCmd(id, extra, okText) {
  const r = await cmd('menu', Object.assign({ id }, extra || {}));
  if (r.ok !== false && okText) toast(okText);
  return r;
}
function askTop(spec) { inlineConfirm($('#topconfirm'), spec); }
async function exportFile(what) {
  const r = await cmd('export', { what });
  if (r.ok) { download(r.filename, r.mime, r.text); toast('Saved ' + r.filename + ' to your downloads'); }
}
function openSnapshot() {
  const input = h('input', { type: 'file', accept: '.xml,text/xml,application/xml' });
  input.addEventListener('change', async () => {
    const f = input.files[0]; if (!f) return;
    const text = await f.text();
    askTop({ title: 'Replace config.xml with ' + f.name + '?', lines: ['The current file is kept next to it with a .web-backup ending.', f.size + ' bytes'],
      label: 'Replace config.xml', danger: true,
      onConfirm: async () => { const r = await cmd('import_snapshot', { text, confirm: true }); if (r.ok) toast('Config snapshot loaded'); } });
  });
  input.click();
}
function save() {
  if (FIC.onSave) FIC.onSave();
  else toast('Nothing on this page to save. Settings and Tuning have their own Save.');
}
async function showInfo() {
  const r = await cmd('menu', { id: 'robot.info' });
  if (!r.ok) return;
  askTop({ title: 'Robot info', lines: r.info.rows.map(([k, v]) => k + ': ' + (Array.isArray(v) ? v.map(x => typeof x === 'number' ? x.toFixed(1) : x).join(', ') : v)), label: 'Close', onConfirm: () => {} });
}
function askProcess(label, id, extra) {
  askTop({ title: label + '?', lines: [], label, onConfirm: async () => { await menuCmd(id, Object.assign({ confirm: true }, extra)); } });
}
const state = () => FIC.state() || {};
const MENUS = [
  ['File', () => [
    { label: 'New Script', run: async () => { const r = await menuCmd('file.new_script'); if (r.ok !== false) location.hash = '#/extensions?id=script_editor'; } },
    { label: 'Open', sub: [
      { label: 'Experiment Spec…', run: go('extensions?id=experiments') },
      { label: 'Sequence File…', run: go('extensions?id=sequences') },
      { label: 'Config Snapshot…', run: openSnapshot }] },
    { sep: true },
    { label: 'Save', run: save },
    { label: 'Export', sub: [
      { label: 'Current Config as Snapshot…', run: () => exportFile('snapshot') },
      { label: 'FIC Parameters as YAML…', run: () => exportFile('params') }] },
    { sep: true },
    { label: 'Recent Experiments', sub: 'recent' },
    { sep: true },
    { label: 'Exit', run: () => askTop({ title: 'Close the Studio?', lines: ['Running processes are stopped, as when the window is closed.'], label: 'Exit', danger: true,
      onConfirm: () => menuCmd('file.exit', { confirm: true }) }) },
  ]],
  ['View', () => [
    { label: 'Dashboard', run: go('dashboard'), hint: 'Ctrl+0' },
    { sep: true },
    { label: 'Robot Control', run: () => FIC.dock && FIC.dock.open('joint'), hint: 'J / C' },
    { label: 'Tuning Panel', run: go('tuning') },
    { sep: true },
    { label: 'Show Toolbar', checked: () => VIEW.toolbar && !VIEW.minimal, run: flip('toolbar') },
    { label: 'Keyboard Control', checked: () => !!(state().toolbar || {}).keyboard, run: () => cmd('toolbar', { op: 'keyboard', value: !(state().toolbar || {}).keyboard }) },
    { label: 'Show Control Bar', checked: () => VIEW.bar, run: flip('bar') },
    { label: 'Show Sidebar', checked: () => VIEW.side && !VIEW.minimal, run: flip('side') },
    { label: 'Logs', run: go('log') },
    { label: 'Show Status Bar', checked: () => VIEW.status && !VIEW.minimal, run: flip('status') },
    { sep: true },
    { label: 'Compact Mode', checked: () => VIEW.compact, run: flip('compact') },
    { label: 'Minimal Mode', checked: () => VIEW.minimal, run: flip('minimal') },
    { label: 'UI Scale', sub: [80, 90, 100, 110, 125, 150].map(n => ({ label: n + '%', checked: () => VIEW.scale === n, run: () => { VIEW.scale = n; applyView(); } })) },
    { label: 'Toggle Fullscreen', run: () => { if (document.fullscreenElement) document.exitFullscreen(); else document.documentElement.requestFullscreen && document.documentElement.requestFullscreen(); }, hint: 'F11' },
    { sep: true },
    { label: '3D view', run: go('view3d') },
  ]],
  ['Robot', () => {
    const s = state(), robots = (s.robots && s.robots.items) || [], modes = ((s.mode || {}).menus) || {};
    const all = Object.values(modes).flatMap(m => m.modes);
    return [
      { label: 'Move to Home', run: () => cmd('home', {}), hint: '⇧H' },
      { label: 'Emergency Stop', run: FIC.fireEstop, hint: 'Space' },
      { sep: true },
      { label: 'Select Robot', sub: robots.map(r => ({ label: r.label, checked: () => r.name === (state().robots || {}).current, run: () => cmd('set_robot', { name: r.name }) })) },
      { label: 'Control Mode', sub: all.map(m => ({ label: (m.index + 1) + ': ' + m.name, checked: () => m.index === (state().mode || {}).index, run: () => cmd('set_mode', { index: m.index }) })) },
      { sep: true },
      { label: 'Robot Info…', run: showInfo },
    ];
  }],
  ['Simulation', () => [
    { label: 'Toggle Simulation', run: () => askProcess('Toggle simulation', 'sim.toggle_simulation') },
    { label: 'Toggle FIC Controller', run: () => askProcess('Toggle FIC controller', 'sim.toggle_fic') },
    { label: 'Launch Sim + FIC', run: () => askProcess('Launch or stop simulation and FIC', 'sim.launch_all') },
    { label: 'Stop All Processes', run: () => askTop({ title: 'Stop all running processes?', lines: ['SIM, FIC, controller, driver, bags and plotter.'], label: 'Stop all', danger: true,
      onConfirm: () => menuCmd('sim.stop_all', { confirm: true }, 'All processes stopped') }) },
    { header: 'Hardware' },
    { label: 'Toggle Controller', run: () => askProcess('Toggle controller', 'sim.toggle_controller') },
    { label: 'Toggle Robot Driver', run: () => askProcess('Toggle robot driver', 'sim.toggle_driver') },
    { sep: true },
    { label: 'Quick Experiment…', run: () => cmd('toolbar', { op: 'exp', confirm: true }) },
    { label: 'Toggle Plotter', run: () => cmd('toolbar', { op: 'plotter', confirm: true }) },
    { sep: true },
    { label: 'Processes…', run: go('session') },
  ]],
  ['Tools', () => [
    { label: 'Topic Monitor', run: go('topic_monitor') },
    { label: 'Robot Configuration', run: go('robot_config') },
    { sep: true },
    { label: 'Open Experiments Folder', run: () => menuCmd('tools.open_experiments_folder') },
    { sep: true },
    { label: 'Compile Workspace', run: go('settings?build=workspace') },
    { label: 'Compile Package…', run: go('settings?build=package') },
    { label: 'Clean Build', run: go('settings?build=clean') },
  ]],
  ['Help', () => [
    { label: 'Help', run: go('help') },
    { label: 'Open Documentation (web)', run: () => cmd('help', { op: 'docs', page: 'home' }) },
    { label: 'Keyboard Shortcuts', run: go('help?s=keyboard_shortcuts') },
    { sep: true },
    { label: 'Settings', run: go('settings') },
    { label: 'Re-run Setup Wizard…', run: () => menuCmd('help.wizard') },
    { sep: true },
    { label: 'About FIC Studio', run: go('help?s=about') },
  ]],
];

let openMenu = null;
function closeMenus() {
  if (!openMenu) return false;
  openMenu.pop.hidden = true; openMenu.btn.setAttribute('aria-expanded', 'false'); openMenu = null; return true;
}
// Menu entries that need the Studio window itself (configuration, files, build): a remote client does not see them.
const LOCAL_LABELS = new Set(['Config Snapshot…', 'Exit', 'Recent Experiments', 'Open Experiments Folder', 'Robot Configuration', 'Compile Workspace',
  'Compile Package…', 'Clean Build', 'Settings', 'Re-run Setup Wizard…', 'Open Documentation (web)']);
async function renderItems(pop, items) {
  pop.textContent = '';
  for (const it of items) {
    if (!FIC.local && LOCAL_LABELS.has(it.label)) continue;
    if (it.sep) { pop.append(h('div', { class: 'mi-sep' })); continue; }
    if (it.header) { pop.append(h('div', { class: 'mi-head', text: it.header })); continue; }
    if (it.sub) {
      let subs = it.sub;
      if (subs === 'recent') {
        const r = await cmd('recent_experiments', {}, true);
        subs = (r.items || []).map(e => ({ label: e.name, run: () => cmd('recent_experiments', { open: e.id }) }));
        if (!subs.length) subs = [{ label: '(no experiments found)', disabled: true }];
      }
      const holder = h('div', { class: 'mi has-sub', tabindex: '0', role: 'menuitem', 'aria-haspopup': 'true' }, h('span', { text: it.label }), h('span', { class: 'kbd sub-caret' }, FIC.icon('chevron-right')));
      const sub = h('div', { class: 'menu-pop sub' });
      sub.hidden = false;
      holder.append(sub);
      await renderItems(sub, subs);
      pop.append(holder);
      continue;
    }
    const checked = it.checked && it.checked();
    const b = h('button', { class: 'mi', type: 'button', role: it.checked ? 'menuitemcheckbox' : 'menuitem', 'aria-checked': it.checked ? String(!!checked) : null, disabled: !!it.disabled,
      onclick: () => { closeMenus(); if (it.run) it.run(); } },
    h('span', { class: 'mi-check' }, checked ? FIC.icon('check') : null), h('span', { text: it.label }), it.hint ? h('span', { class: 'kbd', text: it.hint }) : null);
    pop.append(b);
  }
}
function buildMenus() {
  const bar = $('#menus'); bar.textContent = '';
  for (const [name, items] of MENUS) {
    const btn = h('button', { class: 'menu-btn', type: 'button', 'aria-haspopup': 'true', 'aria-expanded': 'false', text: name });
    const pop = h('div', { class: 'menu-pop', role: 'menu' }); pop.hidden = true;
    const open = async () => {
      closeMenus();
      await renderItems(pop, items());
      pop.hidden = false; btn.setAttribute('aria-expanded', 'true'); openMenu = { btn, pop };
    };
    btn.addEventListener('click', () => { if (openMenu && openMenu.btn === btn) closeMenus(); else open(); });
    btn.addEventListener('mouseenter', () => { if (openMenu && openMenu.btn !== btn) open(); });
    bar.append(h('div', { class: 'menu' }, btn, pop));
  }
  document.addEventListener('click', e => { if (openMenu && !e.target.closest('.menu')) closeMenus(); });
}

// ── Toolbar ───────────────────────────────────────────────────────────────
function sessionView(sess) {
  const st = sess.state || (sess.running ? 'running' : 'idle');
  if (pendingSession) {
    const done = pendingSession.op === 'start' ? st === 'running' : st === 'idle';
    if (done || Date.now() > pendingSession.until) pendingSession = null;
  }
  if (pendingSession) return { text: pendingSession.op === 'start' ? 'Starting' : 'Stopping', cls: 'warn live', stop: sess.running };
  if (st === 'running') return { text: 'Running', cls: 'ok live', stop: true };
  if (st === 'partial') return { text: 'Partial ' + sess.parts_up + '/' + sess.parts_total, cls: 'warn', stop: true };
  return { text: 'Idle', cls: '', stop: false };
}
function onSessionClick() {
  const btn = $('#sess'), running = btn.dataset.stop === '1', op = running ? 'stop' : 'start';
  if (sessionArm && sessionArm.op === op) {
    clearTimeout(sessionArm.timer); sessionArm = null;
    pendingSession = { op, until: Date.now() + 25000 };
    cmd('session', { op, confirm: true }).then(r => { if (r.ok === false) pendingSession = null; else toast('Session ' + (op === 'start' ? 'starting' : 'stopping')); });
    return;
  }
  btn.firstChild.textContent = 'Confirm ' + op + '?';
  sessionArm = { op, timer: setTimeout(cancelArm, 4000) };
}
function cancelArm() {
  if (!sessionArm) return false;
  clearTimeout(sessionArm.timer); sessionArm = null;
  if (FIC.state()) updateToolbar(FIC.state());
  return true;
}
function toolBtn(id, label, on, visible, op) {
  const b = $(id);
  b.hidden = !visible;
  const sig = (on ? 'stop ' : 'play ') + label;
  if (!b.classList.contains('armed') && b.dataset.sig !== sig) { b.dataset.sig = sig; b.replaceChildren(FIC.icon(on ? 'stop' : 'play'), label); }
  b.classList.toggle('running', !!on);
  b.dataset.label = label;
}
function updateToolbar(s) {
  const robots = (s.robots && s.robots.items) || [];
  setSelect($('#robot'), robots.map(r => ({ value: r.name, label: r.label })), s.robots ? s.robots.current : '', name => cmd('set_robot', { name }));
  const tb = s.toolbar || {};
  const sess = s.session || {};
  const stations = (tb.stations || []).map(x => ({ value: String(x.n), label: x.label }));
  setSelect($('#station'), stations, String(tb.station), v => {
    const label = (stations.find(x => x.value === v) || {}).label || v;
    askTop({ title: 'Switch this Studio to ' + label + '?', lines: ['The app restarts to re-point its ROS topics.'], label: 'Switch and restart', danger: true,
      onConfirm: async () => { const r = await cmd('station', { op: 'set', n: Number(v), confirm: true }); if (r.ok) toast('Restarting as ' + label); else $('#station').value = String(tb.station); } });
    $('#station').value = String(tb.station);
  });
  $('#station').disabled = !!sess.running; $('#robot').disabled = !!sess.running;
  setSeg($('#target'), (sess.targets || []).map(t => ({ value: t, label: TARGET_LABELS[t] || t, disabled: !!sess.running })), sess.target, target => cmd('set_target', { target }));
  const view = sessionView(sess), btn = $('#sess');
  btn.dataset.stop = view.stop ? '1' : '';
  if (!sessionArm) { btn.firstChild.textContent = view.stop ? 'Stop' : 'Start'; btn.className = 'btn ' + (view.stop ? 'btn-danger' : 'btn-primary'); }
  $('#sess-dot').className = 'dot ' + view.cls;
  setText($('#sess-state'), view.text);
  $('#sess-state').title = (sess.process_list || []).filter(p => p.running).map(p => p.label).join(', ') || 'Nothing is running';
  toolBtn('#tb-exp', 'EXP', tb.exp && tb.exp.running, tb.exp && tb.exp.visible);
  toolBtn('#tb-bags', 'BAG', tb.bags, true);
  toolBtn('#tb-plot', 'PLT', tb.plotter, true);
  toolBtn('#tb-view', 'VIEW', tb.viewer, true);
}

// ── Status bar ────────────────────────────────────────────────────────────
const LED = { connected: 'ok', disconnected: 'bad', warning: 'warn', active: 'info', idle: '' };
const SB_ITEMS = [['ros', 'ROS2'], ['fic', 'FIC'], ['sim', 'SIM'], ['bags', 'BAGS'], ['inner', 'INNER'], ['teleop', 'TELEOP'], ['gamepad', 'GAMEPAD'], ['web', 'WEB']];
function buildStatusbar() {
  const bar = $('#statusbar'); bar.textContent = '';
  for (const [k, label] of SB_ITEMS) bar.append(h('span', { class: 'sb-item', id: 'sb-' + k }, h('span', { class: 'dot' }), h('span', { class: 'sb-l', text: label }), h('span', { class: 'sb-t faint' })));
  bar.append(h('span', { class: 'spacer' }), h('span', { class: 'sb-item', id: 'sb-link' }, h('span', { class: 'dot' }), h('span', { class: 'sb-l', text: 'STUDIO' }), h('span', { class: 'sb-t faint' })),
    h('span', { class: 'sb-robot', id: 'sb-robot' }));
}
function updateStatusbar(s) {
  const sb = s.statusbar || {};
  for (const [k, label] of SB_ITEMS) {
    const el = $('#sb-' + k), v = sb[k] || { status: 'idle', text: '' };
    el.firstChild.className = 'dot ' + (LED[v.status] || '');
    const extra = (v.text || '').replace(new RegExp('^' + label + '\\s*'), '');
    setText(el.lastChild, k === 'ros' || k === 'fic' || k === 'sim' || k === 'bags' ? '' : extra);
    el.title = v.text || label;
  }
  setText($('#sb-robot'), sb.robot || '');
  updateLink(s);
}
/** One glance: is the Studio talking to us, and is the robot talking to the Studio. */
function updateLink(s) {
  const hl = (s && s.telemetry && s.telemetry.health) || {};
  const L = FIC.link, live = L.open && Date.now() - L.last < 3000;
  let cls, text;
  if (!live) { cls = 'bad'; text = L.last ? 'offline' : 'connecting'; }
  else if (hl.state === 'live') { cls = 'ok live'; text = fmt(hl.rate, 0) + ' Hz'; }
  else if (hl.state === 'stale') { cls = 'warn'; text = 'stale ' + (hl.age_ms != null ? fmt(hl.age_ms / 1000, 1) + ' s' : ''); }
  else { cls = ''; text = 'no robot data'; }
  const el = $('#sb-link');
  if (el) { el.firstChild.className = 'dot ' + cls; setText(el.lastChild, text); el.title = live ? 'Studio connected · robot telemetry: ' + (hl.state || 'offline') : 'The Studio is not answering'; }
}

// ── Navigation: the classic sidebar's pages, in its order ─────────────────
async function loadExtensions() {
  // `hidden` is this account's own choice on the Extensions page, so it only
  // keeps a link out of the sidebar. The page itself still lists them all,
  // which is how you get one back.
  try { const r = await (await fetch('/api/panels')).json(); EXTENSIONS = ((r.ok && r.panels) || []).filter(p => p.section !== 'Studio' && !p.hidden); } catch (e) { EXTENSIONS = []; }
  buildSide();
}
function navLink(href, label, page, ext) {
  return h('a', { href, 'data-page': page, 'data-ext': ext || null, onclick: () => { if (matchMedia('(max-width: 860px)').matches) { VIEW.side = false; applyView(); } } }, label);
}
let sideFilter = '';
function buildSide() {
  const side = $('#side'); side.textContent = '';
  const search = h('input', { class: 'input side-search', type: 'search', placeholder: 'Search', 'aria-label': 'Search pages', value: sideFilter });
  search.addEventListener('input', () => {
    sideFilter = search.value.toLowerCase().trim();
    for (const a of side.querySelectorAll('a')) a.classList.toggle('filtered', !!sideFilter && !a.textContent.toLowerCase().includes(sideFilter));
    for (const e of side.querySelectorAll('.eyebrow')) e.classList.toggle('filtered', !!sideFilter);
  });
  side.append(search);
  const main = FIC.pageList().filter(p => p.nav === 'main'), system = FIC.pageList().filter(p => p.nav === 'system'), bottom = FIC.pageList().filter(p => p.nav === 'bottom');
  let group = null;
  const addPages = (pages) => { for (const p of pages) {
    if (p.group && p.group !== group) side.append(h('div', { class: 'eyebrow', text: p.group }));
    group = p.group;
    const a = navLink('#/' + p.id, p.label, p.id); a.dataset.legged = p.id === 'locomotion' ? '1' : ''; side.append(a);
  } };
  addPages(main);
  const sections = [...new Set(EXTENSIONS.map(e => e.section))];
  for (const sec of sections) {
    side.append(h('div', { class: 'eyebrow', text: sec }));
    for (const e of EXTENSIONS.filter(x => x.section === sec)) side.append(navLink('#/extensions?id=' + encodeURIComponent(e.id), e.label, 'extensions', e.id));
  }
  addPages(system);       // SYSTEM sits last, directly above Settings / Logs / Help
  side.append(h('div', { class: 'side-spacer' }));
  for (const p of bottom) side.append(navLink('#/' + p.id, p.label, p.id));
  markNav();
}
function markNav() {
  const id = FIC.currentPage && FIC.currentPage();
  const ext = new URLSearchParams((location.hash.split('?')[1] || '')).get('id');
  for (const a of document.querySelectorAll('#side a')) {
    const on = a.dataset.page === id && (!a.dataset.ext || a.dataset.ext === ext);
    if (on) a.setAttribute('aria-current', 'page'); else a.removeAttribute('aria-current');
  }
}
function updateNavVisibility(s) {
  const legged = !!(s.locomotion && s.locomotion.available);
  for (const a of document.querySelectorAll('#side a[data-legged="1"]')) a.hidden = !legged;
}

// ── Keyboard: the classic shortcuts, plus mode keys ───────────────────────
const OPTION_KEY = { hapfic: 'f', postural: 'p', elbow: 'e', admittance_planner: 'p' };
function onKey(e) {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const t = e.target, tag = t && t.tagName;
  const typing = tag === 'TEXTAREA' || tag === 'SELECT' || (tag === 'INPUT' && !['checkbox', 'range', 'button'].includes(t.type)) || (t && t.isContentEditable);
  if (e.key === 'Escape') {
    if (cancelArm()) return;
    if (closeMenus()) return;
    if (FIC.dock && FIC.dock.close()) return;
    if (FIC.dismiss && FIC.dismiss()) return;
    if (typing) { t.blur(); return; }
    FIC.fireEstop(); return;
  }
  if (typing) return;
  const onControl = tag === 'BUTTON' || tag === 'A';
  if (e.key === ' ') { if (!onControl) { e.preventDefault(); FIC.fireEstop(); } return; }
  const S = FIC.state();
  if (!S) return;
  const k = e.key, mode = S.mode || {}, av = S.availability || {};
  if (k === 'j' || k === 'c') {
    const kind = k === 'j' ? 'joint' : 'cart', m = (mode.menus || {})[kind];
    if (m && m.modes.length && (kind === 'joint' || av.cart)) cmd('set_mode', { index: m.modes[0].index });
  } else if (/^[1-4]$/.test(k)) {
    const m = ((mode.menus || {})[mode.kind] || { modes: [] }).modes[Number(k) - 1];
    if (m && av.command) cmd('set_mode', { index: m.index });
  } else if (k === 'g') FIC.dock && FIC.dock.toggleGripper();
  else if (k === 'H') cmd('home', {});
  else if (k === 's') onSessionClick();
  else if (k === 't') FIC.cycleTheme();
  else if (k === '[' || k === ']') {
    const ready = FIC.pageList().filter(p => p.ready && p.nav !== 'hidden'), i = ready.findIndex(p => p.id === FIC.currentPage());
    const next = ready[(i + (k === ']' ? 1 : -1) + ready.length) % ready.length];
    if (next) location.hash = '#/' + next.id;
  } else {
    for (const [name, o] of Object.entries(S.options || {})) {
      if (o.kind === mode.kind && OPTION_KEY[name] === k && av[o.kind]) cmd('set_option', { name, value: !o.value });
    }
  }
}

// ── Wiring ────────────────────────────────────────────────────────────────
function build() {
  buildMenus(); buildStatusbar(); buildSide(); applyView();
  $('#sess').addEventListener('click', onSessionClick);
  $('#nav-toggle').addEventListener('click', () => { VIEW.minimal = false; VIEW.side = !VIEW.side; applyView(); });
  for (const [id, op] of [['#tb-exp', 'exp'], ['#tb-bags', 'bags'], ['#tb-plot', 'plotter'], ['#tb-view', 'viewer']]) {
    const b = $(id);
    armed(b, 'Confirm?', () => cmd('toolbar', { op, confirm: true }).then(r => { if (r.ok !== false) toast(b.dataset.label + (b.classList.contains('running') ? ' stopping' : ' starting')); }));
  }
  document.addEventListener('keydown', onKey);
  loadExtensions();
  window.addEventListener('hashchange', markNav);
}
FIC.shell = {
  build,
  update(s) { updateToolbar(s); updateStatusbar(s); updateNavVisibility(s); if (FIC.dock) FIC.dock.update(s); },
  route(id) { markNav(); },
};
FIC.view = VIEW;
FIC.reloadExtensions = loadExtensions;
})();
