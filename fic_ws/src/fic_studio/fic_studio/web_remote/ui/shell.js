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
      { label: 'Move to Home', run: () => goHome(), hint: '⇧H' },
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
    { label: 'Session and processes…', run: go('session') },
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
// The open list hangs from the menu bar's bottom edge (not the button's, which is shorter than the bar) and
// stays inside the window; a submenu lines up with its item and opens to the left when the right is short.
function placeMenu(btn, pop) {
  const bar = btn.closest('.menubar'), menu = pop.parentElement;
  pop.style.left = '0px';
  if (bar) pop.style.top = (bar.getBoundingClientRect().bottom - menu.getBoundingClientRect().top - 1) + 'px';
  const r = pop.getBoundingClientRect();
  if (r.right > window.innerWidth - 8) pop.style.left = (window.innerWidth - 8 - r.right) + 'px';
  for (const holder of pop.querySelectorAll('.mi.has-sub')) {
    holder.addEventListener('mouseenter', () => {
      const sub = holder.querySelector('.menu-pop.sub');
      if (!sub) return;
      sub.classList.remove('flip');
      const sr = sub.getBoundingClientRect();
      if (sr.right > window.innerWidth - 8) sub.classList.add('flip');
    });
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
      placeMenu(btn, pop);
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
  if (!sessionArm) { btn.firstChild.textContent = view.stop ? 'Stop' : 'Start'; btn.className = 'btn ' + (view.stop ? 'btn-danger' : 'btn-go'); }
  $('#sess-dot').className = 'dot ' + view.cls;
  setText($('#sess-state'), view.text);
  $('#sess-state').title = (sess.process_list || []).filter(p => p.running).map(p => p.label).join(', ') || 'Nothing is running';
  toolBtn('#tb-exp', 'Experiment', tb.exp && tb.exp.running, tb.exp && tb.exp.visible);
  toolBtn('#tb-bags', 'Record bag', tb.bags, true);
  toolBtn('#tb-plot', 'Plotter', tb.plotter, true);
  toolBtn('#tb-view', 'Contact viewer', tb.viewer, true);
}

// ── Hardware band, latched E-STOP, Home ───────────────────────────────────
/** Home moves a real arm on hardware: ask first there; in simulation it just goes. */
function goHome() {
  const s = state(), sess = s.session || {};
  if (sess.target !== 'hardware') { cmd('home', {}); return; }
  const robot = ((s.studio || {}).robot_display) || 'the robot';
  askTop({ title: 'Move ' + robot + ' to Home?', lines: ['This is the real arm. Check that the path to Home is clear.'], label: 'Move to Home',
    onConfirm: () => cmd('home', { confirm: true }) });
}
FIC.goHome = goHome;
const two = n => String(n).padStart(2, '0');
const clock = t => { const d = new Date(t * 1000); return two(d.getHours()) + ':' + two(d.getMinutes()) + ':' + two(d.getSeconds()); };
function resetEstop() {
  askTop({ title: 'Reset the E-STOP?', lines: ['Check that the robot and the space around it are safe.', 'Reset starts nothing: start the session again yourself.'],
    label: 'Reset E-STOP', danger: true,
    onConfirm: async () => { const r = await cmd('estop_reset', { confirm: true }); if (r.ok !== false) toast('E-STOP reset'); } });
}
function updateSafetyChrome(s) {
  const sess = s.session || {}, tb = s.toolbar || {}, st = s.studio || {};
  const band = $('#hw-band'), hw = sess.target === 'hardware';
  band.hidden = !hw;
  document.body.classList.toggle('on-hardware', hw);
  if (hw) {
    const station = ((tb.stations || []).find(x => String(x.n) === String(tb.station)) || {}).label || ('Station ' + (st.station != null ? st.station : '?'));
    const text = 'HARDWARE · ' + (st.robot_display || st.robot || 'robot') + ' · ' + station + ' · ' + (sess.running ? 'running: commands move the real arm' : 'idle');
    if (band.textContent !== text) band.textContent = text;
    band.classList.toggle('live', !!sess.running);
  }
  const e = s.estop || {}, banner = $('#estop-banner');
  banner.hidden = !e.latched;
  $('#estop').classList.toggle('latched', !!e.latched);
  if (e.latched) {
    const sig = e.at + '|' + e.by;
    if (banner.__sig !== sig) {
      banner.__sig = sig; banner.textContent = '';
      banner.append(h('b', { text: 'E-STOP latched' }), h('span', { text: ' at ' + clock(e.at) + ' by ' + e.by + '. Controllers were told to ramp down and hold; this Studio\'s processes were stopped. Start, motion and teleoperation are refused until Reset.' }),
        h('span', { class: 'spacer' }), h('button', { class: 'btn', type: 'button', text: 'Reset…', onclick: resetEstop }));
    }
  } else banner.__sig = '';
  if (FIC.applyTitle) FIC.applyTitle();
}

// ── Notices: failures and requests the Studio has for its operator ────────
// A new notice pops up (information goes by itself after 12 s; errors and warnings stay until
// closed, except a passing one, as a robot touching a safety zone: it drops down and goes after 6 s). Closing a pop-up only hides it: every notice stays in the history under Notices in
// the menu bar until someone presses Clear all, which clears it for every viewer.
const NOTE = { popped: Number((() => { try { return sessionStorage.getItem('fic-notice-popped'); } catch (e) { return 0; } })() || 0), first: true, shown: new Map(), sig: '' };
function hidePopup(id) {
  const el = NOTE.shown.get(id); if (el) el.remove(); NOTE.shown.delete(id);
}
function fadePopup(id) {
  const el = NOTE.shown.get(id); if (!el) return;
  el.classList.add('going');
  setTimeout(() => hidePopup(id), 400);
}
function noticeCard(n, onClose) {
  return h('div', { class: 'notice ' + n.level, role: n.level === 'error' ? 'alert' : 'status' },
    h('div', { class: 'notice-head' }, h('b', { text: n.title }), h('span', { class: 'faint num', text: clock(n.t) }),
      onClose ? h('button', { class: 'btn btn-icon', type: 'button', 'aria-label': 'Close', title: 'Close (it stays under Notices)', onclick: onClose }, FIC.icon('x')) : null),
    n.text ? h('div', { class: 'notice-text', text: n.text }) : null);
}
// Notices hang below the whole header (toolbar, hardware band, E-STOP banner): they never cover E-STOP.
function placeNotices() {
  const top = Math.round($('#topbar').getBoundingClientRect().bottom) + 8 + 'px';
  for (const id of ['#notices', '#notices-panel']) { const el = $(id); if (el.style.top !== top) el.style.top = top; }
}
function toggleNoticePanel(open) {
  const p = $('#notices-panel'), btn = $('#notices-btn');
  p.hidden = open === undefined ? !p.hidden : !open;
  btn.setAttribute('aria-expanded', String(!p.hidden));
  if (!p.hidden) { for (const id of [...NOTE.shown.keys()]) hidePopup(id); placeNotices(); NOTE.sig = ''; refreshNoticePanel(); }
}
// The snapshot has only the latest few; the panel fetches the whole history when it changes.
async function refreshNoticePanel() {
  const p = $('#notices-panel'), rev = ((state().notices || {}).rev) || '';
  if (p.hidden || rev === NOTE.sig) return;
  NOTE.sig = rev;
  const r = await cmd('notices_list', {}, true);
  if (r.ok !== false) renderNoticePanel(r.items || []);
}
function renderNoticePanel(list) {
  const p = $('#notices-panel');
  p.textContent = '';
  p.append(h('div', { class: 'np-head' }, h('b', { text: 'Notices' }), h('span', { class: 'faint', text: list.length ? list.length + ' kept until cleared' : '' }), h('span', { class: 'spacer' }),
    h('button', { class: 'btn', type: 'button', text: 'Clear all', disabled: !list.length,
      onclick: async () => { const r = await cmd('notices_clear', {}); if (r.ok !== false) { for (const id of [...NOTE.shown.keys()]) hidePopup(id); toast('Notices cleared'); } } }),
    h('button', { class: 'btn btn-icon', type: 'button', 'aria-label': 'Close', onclick: () => toggleNoticePanel(false) }, FIC.icon('x'))));
  if (!list.length) { p.append(h('p', { class: 'faint np-empty', text: 'No notices. Failures and warnings are kept here until Clear all.' })); return; }
  const body = h('div', { class: 'np-list' });
  for (const n of [...list].reverse()) body.append(noticeCard(n));
  p.append(body);
}
function updateNotices(s) {
  const ns = s.notices || {}, list = ns.recent || [], box = $('#notices');
  if (NOTE.first) {                          // a reload: the history has them; pop up only what is new
    NOTE.first = false;
    const cutoff = Date.now() / 1000 - 120;
    for (const n of list) if (n.t < cutoff) NOTE.popped = Math.max(NOTE.popped, n.id);
  }
  placeNotices();
  const panelOpen = !$('#notices-panel').hidden;
  for (const n of list) {
    if (n.id <= NOTE.popped) continue;
    NOTE.popped = n.id;
    if (panelOpen) continue;                     // the open history shows it already
    try { sessionStorage.setItem('fic-notice-popped', String(NOTE.popped)); } catch (e) { /* ignore */ }
    const el = noticeCard(n, () => hidePopup(n.id));
    if (n.passing) el.classList.add('passing');
    box.append(el); NOTE.shown.set(n.id, el);
    if (n.passing) setTimeout(() => fadePopup(n.id), 6000);
    else if (n.level === 'info') setTimeout(() => hidePopup(n.id), 12000);
  }
  while (NOTE.shown.size > 4) hidePopup(NOTE.shown.keys().next().value);       // the rest are under Notices
  if (!ns.count) for (const id of [...NOTE.shown.keys()]) hidePopup(id);        // cleared elsewhere
  const bad = (ns.errors || 0) + (ns.warnings || 0), btn = $('#notices-btn');
  setText(btn, bad ? 'Notices · ' + bad : 'Notices');
  btn.classList.toggle('has-bad', !!ns.errors);
  btn.classList.toggle('has-warn', !!bad);
  refreshNoticePanel();
  const req = ((s.teleop || {}).pending || []).length, badge = $('#tb-requests');
  badge.hidden = !req;
  if (req) setText(badge, req === 1 ? '1 teleop request' : req + ' teleop requests');
}

// ── Status bar ────────────────────────────────────────────────────────────
const LED = { connected: 'ok', disconnected: 'bad', warning: 'warn', active: 'info', idle: '' };
// [key, the label the Studio's text starts with, the label shown]
const SB_ITEMS = [['ros', 'ROS2', 'ROS 2'], ['fic', 'FIC', 'Controller'], ['sim', 'SIM', 'Simulator'], ['bags', 'BAGS', 'Bag'], ['inner', 'INNER', 'Inner loop'],
  ['teleop', 'TELEOP', 'Teleop'], ['gamepad', 'GAMEPAD', 'Gamepad'], ['web', 'WEB', 'Web']];
function buildStatusbar() {
  const bar = $('#statusbar'); bar.textContent = '';
  for (const [k, , label] of SB_ITEMS) bar.append(h('span', { class: 'sb-item', id: 'sb-' + k }, h('span', { class: 'dot' }), h('span', { class: 'sb-l', text: label }), h('span', { class: 'sb-t faint' })));
  bar.append(h('span', { class: 'spacer' }),
    h('button', { class: 'sb-item sb-ctl', id: 'sb-ctl', type: 'button', hidden: true, title: 'You hold control of this Studio. Click to release it (you keep watching).',
      onclick: () => FIC.releaseControl() }, h('span', { class: 'dot ok' }), h('span', { class: 'sb-l', text: 'In control · Release' })),
    h('span', { class: 'sb-item', id: 'sb-kbd', hidden: true, title: 'Keyboard Control is on (View menu): what the keys drive. Tab swaps joint / Cartesian, 1–9 picks a joint.' },
      h('span', { class: 'dot ok' }), h('span', { class: 'sb-l', text: 'KBD' }), h('span', { class: 'sb-t faint' })),
    h('span', { class: 'sb-item', id: 'sb-link' }, h('span', { class: 'dot' }), h('span', { class: 'sb-l', text: 'STUDIO' }), h('span', { class: 'sb-t faint' })),
    h('span', { class: 'sb-robot', id: 'sb-robot' }));
}
function updateStatusbar(s) {
  const sb = s.statusbar || {};
  for (const [k, label] of SB_ITEMS) {          // label: the prefix the Studio's text carries
    const el = $('#sb-' + k), v = sb[k] || { status: 'idle', text: '' };
    el.firstChild.className = 'dot ' + (LED[v.status] || '');
    const extra = (v.text || '').replace(new RegExp('^' + label + '\\s*'), '');
    setText(el.lastChild, k === 'ros' || k === 'fic' || k === 'sim' || k === 'bags' ? '' : extra);
    el.title = v.text || label;
  }
  setText($('#sb-robot'), sb.robot || '');
  const c = FIC.control && FIC.control();
  $('#sb-ctl').hidden = FIC.local || !(c && c.you);     // a remote holder can hand control back
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
  const es = $('#estop'), note = $('#estop-note');
  es.classList.toggle('offline', !live);
  if (note) { setText(note, live ? 'Space' : 'unavailable: use the physical stop'); note.classList.toggle('bad-text', !live); }
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

// ── Keyboard: Keyboard Control, the classic shortcuts, the mode keys ──────
const OPTION_KEY = { hpmp: 'f', postural: 'p', elbow: 'e', admittance_planner: 'p' };
function isTyping(t) {
  const tag = t && t.tagName;
  return tag === 'TEXTAREA' || tag === 'SELECT' || (tag === 'INPUT' && !['checkbox', 'range', 'button'].includes(t.type)) || !!(t && t.isContentEditable);
}
// Keyboard Control (View menu): the held drive keys go to the Studio (keys.js says how).
const KBD = FIC.keys.Driver((keys, tap) => cmd('keyboard_keys', tap ? { keys, tap } : { keys }, true).then(r => {
  if (r && r.ok === false && (keys.length || tap)) {
    KBD.clear();                                  // refused (E-STOP, no control): stop sending
    if (Date.now() - (KBD.told || 0) > 4000) { KBD.told = Date.now(); toast(r.message || 'Keyboard Control refused', true); }
  }
}));
/** Take a drive key while Keyboard Control is on; true when the shortcuts must not see it. */
function driveKey(e) {
  const S = FIC.state();
  if (!S || !(S.toolbar || {}).keyboard || e.altKey || e.metaKey || FIC.keys.modeKey(e) || isTyping(e.target)) return false;
  const n = FIC.keys.name(e);
  if (!n || !FIC.keys.isOurs(n)) return false;
  if (FIC.keys.MODS.has(n)) { KBD.down(n); return false; }      // Shift / Ctrl: speed, and still Shift+H
  e.preventDefault();                             // Tab must not move the focus, nor the arrows scroll
  if (!e.repeat) KBD.down(n);
  return true;
}
let kbdWasOn = false;
function updateKeyboard(s) {
  const tb = s.toolbar || {}, d = tb.keyboard_drive || {}, on = !!tb.keyboard;
  if (!on) KBD.clear();
  if (on && !kbdWasOn) toast('Keyboard Control on: W/S A/D Q/E move, J/L yaw, 1–9 then ↑/↓ a joint, Tab joint/Cartesian, O/C gripper, Shift faster, Ctrl slower');
  kbdWasOn = on;
  const el = $('#sb-kbd');
  if (!el) return;
  el.hidden = !on;
  setText(el.lastChild, d.space === 'joint' ? 'J' + ((d.joint || 0) + 1) : 'CART');
}
function onKey(e) {
  if (driveKey(e)) return;
  if ((e.ctrlKey || e.metaKey) && !e.altKey && !e.shiftKey && FIC.keys.name(e) === 'K') {     // Ctrl+K, as in the classic window
    e.preventDefault();
    const on = !!((FIC.state() || {}).toolbar || {}).keyboard;
    cmd('toolbar', { op: 'keyboard', value: !on });
    return;
  }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const t = e.target, typing = isTyping(t), tag = t && t.tagName;
  if (e.key === 'Escape') {
    if (cancelArm()) return;
    if (closeMenus()) return;
    if (FIC.dock && FIC.dock.close()) return;
    if (FIC.dismiss && FIC.dismiss()) return;
    if (typing) t.blur();
    return;                          // Esc closes things; it never E-STOPs (Space does)
  }
  if (typing) return;
  const onControl = tag === 'BUTTON' || tag === 'A';
  if (e.key === ' ') { if (!onControl) { e.preventDefault(); FIC.fireEstop(); } return; }
  const S = FIC.state();
  if (!S) return;
  const k = e.key, mode = S.mode || {}, av = S.availability || {};
  const mk = FIC.keys.modeKey(e);                  // Shift+J / C / 1-4 / F / P / E
  if (mk && !(S.toolbar || {}).keyboard) {          // a stray key must not change how a robot is driven
    if (Date.now() - (onKey.hinted || 0) > 8000) { onKey.hinted = Date.now(); toast('Mode keys are off. Turn on View → Keyboard Control to use them.'); }
    return;
  }
  if (mk === 'j' || mk === 'c') {
    const kind = mk === 'j' ? 'joint' : 'cart', m = (mode.menus || {})[kind];
    if (m && m.modes.length && (kind === 'joint' || av.cart)) cmd('set_mode', { index: m.modes[0].index });
  } else if (mk && /^[1-4]$/.test(mk)) {
    const m = ((mode.menus || {})[mode.kind] || { modes: [] }).modes[Number(mk) - 1];
    if (m && av.command) cmd('set_mode', { index: m.index });
  } else if (mk) {
    for (const [name, o] of Object.entries(S.options || {})) {
      if (o.kind === mode.kind && OPTION_KEY[name] === mk && av[o.kind]) cmd('set_option', { name, value: !o.value });
    }
  } else if (k === 'g') FIC.dock && FIC.dock.toggleGripper();
  else if (k === 'H') goHome();
  else if (k === 's') onSessionClick();
  else if (k === 't') FIC.cycleTheme();
  else if (k === '[' || k === ']') {
    const ready = FIC.pageList().filter(p => p.ready && p.nav !== 'hidden'), i = ready.findIndex(p => p.id === FIC.currentPage());
    const next = ready[(i + (k === ']' ? 1 : -1) + ready.length) % ready.length];
    if (next) location.hash = '#/' + next.id;
  }
}

// ── Wiring ────────────────────────────────────────────────────────────────
function build() {
  buildMenus(); buildStatusbar(); buildSide(); applyView();
  $('#sess').addEventListener('click', onSessionClick);
  $('#notices-btn').addEventListener('click', () => toggleNoticePanel());
  $('#nav-toggle').addEventListener('click', () => { VIEW.minimal = false; VIEW.side = !VIEW.side; applyView(); });
  for (const [id, op] of [['#tb-exp', 'exp'], ['#tb-bags', 'bags'], ['#tb-plot', 'plotter'], ['#tb-view', 'viewer']]) {
    const b = $(id);
    armed(b, 'Confirm?', () => cmd('toolbar', { op, confirm: true }).then(r => { if (r.ok !== false) toast(b.dataset.label + (b.classList.contains('running') ? ' stopping' : ' starting')); }));
  }
  document.addEventListener('keydown', onKey);
  document.addEventListener('keyup', e => { const n = FIC.keys.name(e); if (n) KBD.up(n); });     // letting go is always taken
  window.addEventListener('blur', () => KBD.clear());
  document.addEventListener('visibilitychange', () => { if (document.hidden) KBD.clear(); });
  loadExtensions();
  window.addEventListener('hashchange', markNav);
}
FIC.shell = {
  build,
  update(s) { updateKeyboard(s); updateToolbar(s); updateSafetyChrome(s); updateNotices(s); updateStatusbar(s); updateNavVisibility(s); if (FIC.dock) FIC.dock.update(s); },
  route(id) { markNav(); },
};
})();
