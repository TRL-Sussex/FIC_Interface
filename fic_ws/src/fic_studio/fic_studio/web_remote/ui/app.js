/* FIC Studio web UI: core. Plain ES, no build step.
   Talks to the Python Studio only through its local API:
     GET /api/ui, GET /api/state, GET /api/events (SSE), POST /api/control,
     POST /api/command. No robot logic lives here.
   This file holds what every page shares (API, state, small builders, the
   router, the control lease). The window chrome (menus, toolbar, status bar)
   is shell.js and the control bar is dock.js; each page is pages-*.js. */
(() => {
'use strict';

// ── Tiny helpers ──────────────────────────────────────────────────────────
const $ = (sel, root = document) => root.querySelector(sel);
function h(tag, props, ...kids) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(props || {})) {
    if (v == null || v === false) continue;
    if (k === 'class') node.className = v;
    else if (k === 'text') node.textContent = v;
    else if (k.startsWith('on')) node.addEventListener(k.slice(2), v);
    else if (k === 'icon') continue;
    else node.setAttribute(k, v === true ? '' : v);
  }
  for (const kid of kids.flat()) if (kid != null) node.append(kid.nodeType ? kid : document.createTextNode(kid));
  if (props && props.icon && window.FIC && window.FIC.icon) node.prepend(window.FIC.icon(props.icon));
  return node;
}
const fmt = (v, d = 1) => (v == null || !Number.isFinite(v)) ? '—' : v.toFixed(d);
const setText = (node, text) => { if (node && node.textContent !== text && node.__raw !== text) { node.__raw = undefined; node.textContent = text; } };
const params = new URLSearchParams(location.search);
const store = {
  get(k) { try { return localStorage.getItem(k); } catch (e) { return null; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private window */ } },
};

// ── Theme: auto follows the system; the toggle overrides ─────────────────
const THEMES = ['auto', 'light', 'dark'];
let themeMode = params.get('theme') || store.get('fic-theme') || 'auto';
const APPEAR = { rev: null, a: null };
const prefersDark = () => window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;
function applyTheme() {
  if (themeMode === 'auto') document.documentElement.removeAttribute('data-theme');
  else document.documentElement.setAttribute('data-theme', themeMode);
  setText($('#theme'), 'Theme: ' + themeMode);
  paintAppearance();
}
// Appearance settings (palette, font, size) restyle the page in place: CSS custom
// properties and the root font size, no reload. Brightness follows the setting
// whenever it changes; the menu's own switch still overrides it until then.
function paintAppearance() {
  const a = APPEAR.a, root = document.documentElement;
  if (!a) return;
  const bright = themeMode === 'auto' ? (prefersDark() ? 'dark' : 'light') : themeMode;
  const accent = a.accent && a.accent[bright];
  if (accent) {
    root.style.setProperty('--accent', accent);
    root.style.setProperty('--accent-soft', 'color-mix(in srgb, ' + accent + ' 12%, transparent)');
    root.style.setProperty('--accent-tint-hover', 'color-mix(in srgb, ' + accent + ' 20%, transparent)');
    root.style.setProperty('--accent-tint-press', 'color-mix(in srgb, ' + accent + ' 28%, transparent)');
  }
  if (a.font_family) root.style.setProperty('--mono', "'" + a.font_family + "', ui-monospace, 'SF Mono', Menlo, monospace");
  root.dataset.palette = a.palette || '';
  window.FIC.fontK = (a.font_size || 12) / 12;
  if (window.FIC.applyRootFont) window.FIC.applyRootFont();
}
function applyAppearance(a) {
  if (!a || a.rev === APPEAR.rev) return;
  const first = APPEAR.rev === null;
  APPEAR.rev = a.rev; APPEAR.a = a;
  if (a.brightness === 'light' || a.brightness === 'dark') {
    if (!first || (!store.get('fic-theme') && !params.get('theme'))) { themeMode = a.brightness; store.set('fic-theme', themeMode); }
  }
  applyTheme();
}
if (window.matchMedia) window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', paintAppearance);
function cycleTheme() { themeMode = THEMES[(THEMES.indexOf(themeMode) + 1) % 3]; store.set('fic-theme', themeMode); applyTheme(); }

// ── API ───────────────────────────────────────────────────────────────────
const CLIENT = (() => {
  let id = null;
  try { id = sessionStorage.getItem('fic-client'); } catch (e) { /* ignore */ }
  if (!id) { id = 'ui-' + Math.random().toString(36).slice(2, 10); try { sessionStorage.setItem('fic-client', id); } catch (e) { /* ignore */ } }
  return id;
})();
async function post(path, body) {
  let res, data = {};
  try {
    res = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(Object.assign({ client: CLIENT, name: window.FIC && window.FIC.local === false ? 'Web remote' : 'Studio window' }, body)) });
  } catch (e) { return { ok: false, error: 'offline', message: 'The Studio is not reachable' }; }
  try { data = await res.json(); } catch (e) { /* not json */ }
  if (res.status === 401 && path !== '/api/login' && window.FIC.remote) window.FIC.remote.signedOut('Sign in again.');
  return data;
}
let toastTimer = null;
function toast(msg, bad) {
  const t = $('#toast');
  t.textContent = msg; t.className = 'toast show' + (bad ? ' bad' : '');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('show'), 3800);
}
/** One command, through the Studio's own dispatch. Returns the reply. */
async function command(action, args, quiet) {
  if (window.FIC.local === false && INFO.local_only.includes(action)) {   // never a round trip that is bound to be refused
    if (!quiet) toast('Only available in the Studio window', true);
    return { ok: false, error: 'local_only', message: 'Only available in the Studio window' };
  }
  const reply = await post('/api/command', { action, args: Object.assign({ robot: S && S.robots && S.robots.current }, args) });
  if (reply.ok === false && !quiet) toast(reply.message || reply.error || 'Refused', true);
  return reply;
}
const cmd = command;

// ── State ─────────────────────────────────────────────────────────────────
let S = null;            // latest snapshot
let CONTROL = null;      // lease view
let PAGES = [];
let INFO = { local: true, local_only: [] };   // /api/ui: is this the desktop window, and what only it may do
const history = { rate: [], force: [] };
const LINK = { last: 0, open: false };

function pushHistory(s) {
  const t = s.telemetry || {};
  const rate = t.health && t.health.rate;
  const w = t.wrench;
  const force = w ? Math.hypot(w[0], w[1], w[2]) : null;
  for (const [k, v] of [['rate', rate], ['force', force]]) {
    history[k].push(v == null ? 0 : v); if (history[k].length > 120) history[k].shift();
  }
}

// ── Reusable builders (patch in place; never rebuild what did not change) ─
function setSeg(box, items, active, onPick) {
  const sig = items.map(i => i.value + (i.disabled ? '!' : '')).join('|');
  if (box.__sig !== sig) {
    box.__sig = sig; box.textContent = '';
    for (const it of items) {
      const b = h('button', { type: 'button', text: it.label, 'data-v': it.value, disabled: !!it.disabled,
        title: it.title || null, onclick: () => onPick(it.value) });
      box.append(b);
    }
  }
  for (const b of box.children) b.setAttribute('aria-pressed', String(b.dataset.v === String(active)));
}
function setSelect(sel, items, value, onPick) {
  const sig = items.map(i => i.value + '/' + i.label).join('|');
  if (sel.__sig !== sig) {
    sel.__sig = sig; sel.textContent = '';
    for (const it of items) sel.append(h('option', { value: it.value, text: it.label, disabled: !!it.disabled }));
    if (!sel.__bound) { sel.__bound = true; sel.addEventListener('change', () => onPick(sel.value)); }
  }
  if (document.activeElement !== sel && sel.value !== value) sel.value = value;
}
function spark(values) {
  const n = values.length;
  if (n < 2) return h('svg', { class: 'spark', viewBox: '0 0 100 40' });
  const max = Math.max(...values), min = Math.min(...values);
  const pts = values.map((v, i) => (i / (n - 1) * 100).toFixed(2) + ' ' + (38 - (v - min) / ((max - min) || 1) * 34).toFixed(2));
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('class', 'spark'); svg.setAttribute('viewBox', '0 0 100 40'); svg.setAttribute('preserveAspectRatio', 'none');
  const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
  path.setAttribute('d', 'M' + pts.join(' L')); svg.append(path);
  return svg;
}

// ── Control lease: the desktop window is the trusted local session ───────
async function takeControl(force) {
  const r = await post('/api/control', { op: 'take', force: !!force });
  CONTROL = r.control || CONTROL;
  return r.ok;
}
setInterval(() => { if (document.body.classList.contains('signed-out')) return; post('/api/control', { op: 'heartbeat' }).then(r => { if (r.control) { CONTROL = r.control; renderBanner(); } }); }, 1000);
let hadControl = null;
async function releaseControl() {
  const r = await post('/api/control', { op: 'release' });
  CONTROL = r.control || CONTROL; renderBanner();
  toast('Control released: you are watching');
}
function renderBanner() {
  const mine = CONTROL && CONTROL.you;
  if (hadControl === true && CONTROL && !mine) {        // someone else took it: say who, not silently
    const who = CONTROL.holder && (CONTROL.holder.name || CONTROL.holder.addr);
    toast((who || 'Someone') + ' took control. You are watching.', true);
  }
  if (CONTROL) hadControl = !!mine;
  const b = $('#banner');
  if (!b) return;
  b.hidden = !!mine || !CONTROL;
  if (!b.hidden && !b.__built) {
    b.__built = true;
    b.append(h('span', { id: 'banner-text' }), h('button', { class: 'btn btn-primary', type: 'button', text: window.FIC.local ? 'Take control' : 'Request control',
      onclick: async () => { if (!(await takeControl(window.FIC.local))) toast('Control is held by someone else. Ask them to release it.', true); } }),
      // An admin may take over from a remote holder (they are told, and it is in the audit log).
      h('button', { class: 'btn btn-danger', type: 'button', id: 'banner-takeover', text: 'Take over', hidden: true,
        title: 'Take control from whoever holds it. They are told, and it goes in the audit log.',
        onclick: async () => { if (await takeControl(true)) toast('You took control'); else toast('Could not take control', true); } }));
  }
  if (!b.hidden) {
    const who = CONTROL.holder && (CONTROL.holder.name || CONTROL.holder.addr);
    const view = window.FIC.remote && window.FIC.remote.viewOnly();
    setText($('#banner-text'), view ? 'View only: this account cannot command the robot.' : who ? who + ' is in control. You are watching.' : 'You are watching. Take control to command the robot.');
    $('#banner button').hidden = !!view;
    const R = window.FIC.remote, admin = !window.FIC.local && R && R.auth && R.auth.admin;
    $('#banner-takeover').hidden = !(admin && CONTROL.holder);
  }
}

// ── Page furniture ────────────────────────────────────────────────────────
const pageTitle = (title, lede) => [h('h1', { class: 'title', text: title }), h('p', { class: 'lede', id: 'lede', text: lede || '' }),
  h('div', { class: 'banner', id: 'banner', hidden: true })];
const section = (title, aside, ...body) => h('section', { class: 'section' },
  h('div', { class: 'section-head' }, h('h2', { text: title }), aside ? h('span', { class: 'aside', id: aside.id, text: aside.text || '' }) : null), ...body);

// ── Collapsible sections: a header that toggles, state kept per viewer ─────
// csection(key, title, aside, ...body). The header is a real button (Enter and
// Space work), with aria-expanded / aria-controls. A section that holds an
// unsaved change (.edited) or an invalid value ([aria-invalid=true]) opens
// itself and shows a dot. Nothing here knows what a page's fields are.
const SEC = { map: null, n: 0 };
function secLoad() {
  if (SEC.map) return SEC.map;
  try { SEC.map = JSON.parse(store.get('fic-sections') || '{}') || {}; } catch (e) { SEC.map = {}; }
  return SEC.map;
}
function secSave() { try { store.set('fic-sections', JSON.stringify(SEC.map || {})); } catch (e) { /* no storage */ } }
function csection(key, title, aside, ...body) {
  const id = 'sec-' + (++SEC.n), saved = secLoad()[key];
  const btn = h('button', { class: 'sec-toggle', type: 'button', 'aria-expanded': 'true', 'aria-controls': id },
    window.FIC.icon('chevron-down', 'sec-chev'), h('span', { class: 'sec-title', text: title }),
    h('span', { class: 'sec-dot', hidden: true, title: 'Unsaved or invalid value in this section' }));
  const head = h('div', { class: 'section-head sec-head' }, h('h2', null, btn),
    aside ? h('span', { class: 'aside', id: aside.id, text: aside.text || '' }) : null);
  const inner = h('div', { class: 'sec-inner' }, ...body);
  const panel = h('div', { class: 'sec-body', id }, inner);
  const el = h('section', { class: 'section csec', 'data-sec': key }, head, panel);
  let open = true, flagged = false;
  const set = (v, persist) => {
    open = !!v; el.classList.toggle('collapsed', !open);
    btn.setAttribute('aria-expanded', String(open));
    if (open) panel.removeAttribute('inert'); else panel.setAttribute('inert', '');
    if (persist) { secLoad()[key] = open; secSave(); }
  };
  set(saved === undefined ? !key.startsWith('settings/') : !!saved, false);   // Settings starts folded; its other pages start open
  head.addEventListener('click', e => { if (e.target.closest('a, input, select, textarea') || (e.target.closest('button') && e.target.closest('button') !== btn)) return; set(!open, true); });
  const scan = () => {
    const f = !!el.querySelector('.edited, [aria-invalid="true"]');
    btn.querySelector('.sec-dot').hidden = !f;
    if (f && !flagged && !open) set(true, false);     // opens for a change; not remembered
    flagged = f;
  };
  new MutationObserver(scan).observe(el, { subtree: true, childList: true, attributes: true, attributeFilter: ['class', 'aria-invalid'] });
  el.csec = { set, get open() { return open; }, scan };
  queueMicrotask(scan);
  return el;
}
function csectionAll(root, open) { for (const el of root.querySelectorAll('.csec')) el.csec.set(open, true); }
// "Expand all / Collapse all", quiet secondary actions for the top of a page.
const csectionTools = root => h('div', { class: 'sec-tools' },
  h('button', { class: 'btn', type: 'button', text: 'Expand all', onclick: () => csectionAll(root, true) }),
  h('button', { class: 'btn', type: 'button', text: 'Collapse all', onclick: () => csectionAll(root, false) }));

function buildSoon(id) {
  return root => {
    const p = PAGES.find(x => x.id === id) || { label: id };
    root.append(...pageTitle(p.label, 'No such page.'));
    return () => {};
  };
}
const BUILDERS = {};

// ── Inline confirmation: a hairline-ruled question under the button that asked ─
let openConfirm = null;
function dismissConfirm() {
  if (!openConfirm) return false;
  openConfirm.remove(); openConfirm = null; return true;
}
/** Ask again before something that matters. No dialog box: the question opens in place. */
function inlineConfirm(host, { title, lines, label, danger, onConfirm }) {
  dismissConfirm();
  const box = h('div', { class: 'confirm', role: 'alertdialog', 'aria-label': title },
    h('div', { class: 'confirm-title', text: title }),
    ...(lines || []).map(l => h('div', { class: 'confirm-line num', text: l })),
    h('div', { class: 'btn-row' },
      h('button', { class: 'btn ' + (danger ? 'btn-danger' : 'btn-primary'), type: 'button', text: label || 'Confirm',
        onclick: async () => { dismissConfirm(); await onConfirm(); } }),
      h('button', { class: 'btn', type: 'button', text: 'Cancel', onclick: dismissConfirm })));
  host.append(box); openConfirm = box;
  box.querySelector('button').focus({ focusVisible: false });
}

/** A text button that asks twice: the first click changes its label, the second (within 3.5 s) acts. */
function armed(button, label, act) {
  let timer = null, original = null;
  button.addEventListener('click', () => {
    if (timer) { clearTimeout(timer); timer = null; button.textContent = original; button.classList.remove('armed'); act(); return; }
    original = button.textContent; button.textContent = label; button.classList.add('armed');
    timer = setTimeout(() => { timer = null; button.textContent = original; button.classList.remove('armed'); }, 3500);
  });
  return () => { if (timer) { clearTimeout(timer); timer = null; button.textContent = original; button.classList.remove('armed'); return true; } return false; };
}

// ── E-STOP: one click, no confirmation, no lease ──────────────────────────
async function fireEstop() {
  const r = await post('/api/command', { action: 'estop', args: {} });
  toast(r.ok === false ? (r.message || 'E-STOP failed') + '. Use the physical emergency stop.' : 'E-STOP sent (software stop): latched until Reset', r.ok === false);
}

// ── What the page and shell scripts build on ──────────────────────────────
window.FIC = {
  $, h, fmt, setText, setSeg, setSelect, spark, section, csection, csectionTools, pageTitle, toast, post, cmd, inlineConfirm, armed,
  pages: BUILDERS, state: () => S, history, store,
  dismiss: dismissConfirm, pageList: () => PAGES, fireEstop, link: LINK, cycleTheme, local: true,
  control: () => CONTROL, releaseControl, takeControl,
  shell: { build() {}, update() {}, route() {} },
};

// ── Router ────────────────────────────────────────────────────────────────
/** The tab title: page name, with "HW ·" in front while the Studio drives real hardware and
 *  "E-STOP ·" while one is latched, so a tab in the background still says so. */
function applyTitle() {
  const s = S || {}, pre = (s.estop && s.estop.latched ? 'E-STOP · ' : '') + ((s.session || {}).target === 'hardware' ? 'HW · ' : '');
  const t = pre + (window.FIC.pageName || 'FIC Studio') + ' · FIC Studio';
  if (document.title !== t) document.title = t;
}
window.FIC.applyTitle = applyTitle;
let current = null;
function route() {
  const id = (location.hash.replace(/^#\//, '') || 'dashboard').split('?')[0];
  const main = $('#main');
  if (current && current.dispose) { try { current.dispose(); } catch (e) { /* page already gone */ } }
  main.textContent = '';
  window.FIC.onSave = null;                  // a page that can save sets it while it builds
  const R = window.FIC.remote, blocked = R && window.FIC.local === false && !R.allows(id);
  const build = blocked ? R.notAllowed(id) : (BUILDERS[id] || buildSoon(id));
  const made = build(main);
  current = { id, update: typeof made === 'function' ? made : (made && made.update) || (() => {}), dispose: made && made.dispose };
  window.FIC.pageName = (PAGES.find(p => p.id === id) || {}).label || 'FIC Studio';
  applyTitle();
  window.FIC.shell.route(id);
  if (S) current.update(S);
  renderBanner();
  main.scrollTop = 0;
}
function refresh(s) {
  if (s && s.appearance) applyAppearance(s.appearance);
  S = s; pushHistory(s);
  window.FIC.shell.update(s);
  if (current) current.update(s);
}
window.FIC.currentPage = () => current && current.id;

// A script error should be seen, not silent: say so once in a while.
let lastErr = 0;
function reportError(msg) { window.__errs = (window.__errs || []).concat(String(msg)); if (Date.now() - lastErr > 2000) { lastErr = Date.now(); toast('UI error: ' + String(msg).slice(0, 160), true); } }
window.addEventListener('error', e => reportError(e.message + (e.filename ? ' (' + e.filename.split('/').pop() + ':' + e.lineno + ')' : '')));
window.addEventListener('unhandledrejection', e => reportError(e.reason && e.reason.message ? e.reason.message : e.reason));

// ── Boot ──────────────────────────────────────────────────────────────────
function reveal() { document.documentElement.classList.remove('booting'); }
async function boot() {
  setTimeout(reveal, 5000);                  // never stay hidden, whatever fails below
  applyTheme();
  $('#theme').addEventListener('click', cycleTheme);
  // E-STOP: one click, no confirmation, no lease needed.
  $('#estop').addEventListener('click', fireEstop);
  try { INFO = await (await fetch('/api/ui')).json(); PAGES = INFO.pages; } catch (e) { PAGES = []; }
  window.FIC.local = INFO.local !== false;
  if (!window.FIC.local) {                  // a remote client: sign in first, then only what the account may use
    document.body.classList.add('remote');
    reveal();                                // the sign-in screen
    if (!(await window.FIC.remote.gate())) return;
    PAGES = window.FIC.remote.filterPages(PAGES);
  }
  window.FIC.shell.build();
  window.addEventListener('hashchange', route);
  try {
    const first = await (await fetch('/api/state?client=' + CLIENT)).json();
    if (first.ok) { if (first.state && first.state.appearance) applyAppearance(first.state.appearance); CONTROL = first.control; S = first.state; LINK.last = Date.now(); LINK.open = true; }
  } catch (e) { /* the stream will fill it in */ }
  await takeControl(window.FIC.local);       // the desktop takes it; a remote client asks, and may be refused
  if (S) window.FIC.shell.update(S);
  route();
  requestAnimationFrame(() => requestAnimationFrame(reveal));   // after the first page has laid out
  const es = new EventSource('/api/events?client=' + CLIENT + '&name=Studio%20window');
  es.addEventListener('open', () => { LINK.open = true; });
  es.addEventListener('error', () => { LINK.open = false; if (window.FIC.remote) window.FIC.remote.tick(); if (S) window.FIC.shell.update(S); });
  es.addEventListener('state', e => { LINK.open = true; LINK.last = Date.now(); refresh(JSON.parse(e.data)); });
  es.addEventListener('settings', e => { try { const d = JSON.parse(e.data); applyAppearance(d.appearance); } catch (err) { /* ignore */ } });
  es.addEventListener('control', e => { CONTROL = JSON.parse(e.data); renderBanner(); });
  setInterval(() => { if (S) window.FIC.shell.update(S); }, 1000);
}
window.addEventListener('load', boot);
})();

/* Wheel over a slider or number field must scroll the panel, never change the value. */
document.addEventListener('wheel', ev => {
  const t = ev.target;
  if (t && t.tagName === 'INPUT' && (t.type === 'range' || t.type === 'number') && document.activeElement === t) t.blur();
}, { capture: true, passive: true });
