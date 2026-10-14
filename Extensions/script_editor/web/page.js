// Script Editor — web page (manifest "web"). The Studio side is web_state() /
// web_action() in ../extension.py; the editor is ./editor.js. The script being
// edited lives in this browser, kept across page changes and reloads; the
// Studio keeps the saved scripts, runs them and holds their output.
import { createEditor, scriptLine } from './editor.js';

const DRAFT_KEY = 'fic-script-editor-draft';
const STARTER = '# Robot control script. Examples and the robot API: the buttons on the right.\n\nrobot.motion.home()\n';
const STATUS = { idle: 'Idle', running: 'Running', finished: 'Finished', failed: 'Failed', stopped: 'Stopped' };
let kept = null;                  // the draft while this tab is open: survives leaving the page

/** The draft: { name ('' is Untitled), code, saved (the text as last opened or saved; null if never) }. */
function loadDraft() {
  if (kept) return kept;
  try {
    const d = JSON.parse(localStorage.getItem(DRAFT_KEY) || 'null');
    if (d && typeof d.code === 'string' && typeof d.name === 'string') return d;
  } catch (e) { /* nothing kept, or a private window */ }
  return { name: '', code: STARTER, saved: STARTER };
}
function keepDraft(d) {
  kept = d;
  try { localStorage.setItem(DRAFT_KEY, JSON.stringify(d)); } catch (e) { /* private window: this tab still keeps it */ }
}
/** Where an API entry goes in the drawer: robot.motion, robot, or the flat aliases. */
function groupOf(name) {
  if (!name.startsWith('robot.')) return 'Flat aliases';
  const parts = name.split('.');
  return parts.length > 2 ? parts[0] + '.' + parts[1] : 'robot';
}
/** What choosing an API entry inserts: a property as it is, a function with brackets. */
const insertText = e => (e.signature ? e.name + '()' : e.name);

export default function build(root, api) {
  const { ui, h } = api;
  let s = null;                                    // the last web_state()
  let draft = loadDraft();
  let drawer = null;                               // 'examples', 'api' or null
  let entries = [];                                // api_reference.json; null if it did not load
  let lastStatus = null;
  let outSeq = -1;
  let examplesSig = '';
  const apiNames = new Set();
  const dirty = () => draft.code !== draft.saved;
  const label = () => draft.name || 'Untitled';

  const editor = createEditor({
    api: apiNames,
    onChange: code => { draft = Object.assign({}, draft, { code }); keepDraft(draft); drawFile(); },
    onSave: () => save(),
    onRun: () => run(),
  });

  const pick = ui.select({ options: [{ value: '', label: 'Untitled' }], value: draft.name, onChange: v => choose(v) });
  const dot = h('span', { class: 'se-dot', title: 'Unsaved changes', text: '●' });
  const deleteBtn = ui.button({ label: 'Delete', kind: 'danger', onClick: () => remove() });
  const runsOn = h('span', { class: 'se-runs' });
  const runBtn = ui.button({ label: 'Run', icon: 'play', kind: 'solid', title: 'Ctrl+Enter', onClick: () => run() });
  const stopBtn = ui.button({ label: 'Stop', icon: 'stop', kind: 'danger', disabled: true, onClick: () => api.call('stop') });
  const status = h('span', { class: 'se-status' });
  const examplesBtn = ui.button({ label: 'Examples', onClick: () => toggle('examples') });
  const apiBtn = ui.button({ label: 'API', onClick: () => toggle('api') });
  const ask = h('div', { class: 'se-ask' });       // inline confirmations and the Save as box
  const out = h('div', { class: 'se-out', role: 'log' });
  const drawerBox = h('aside', { class: 'se-drawer', hidden: true });

  root.append(h('div', { class: 'se-page' },
    h('div', { class: 'se-bar' }, pick, dot,
      ui.buttons(
        ui.button({ label: 'New', onClick: () => discardThen(() => setDraft({ name: '', code: STARTER, saved: STARTER })) }),
        ui.button({ label: 'Save', title: 'Ctrl+S', onClick: () => save() }),
        ui.button({ label: 'Save as…', onClick: () => saveAs() }),
        deleteBtn),
      h('span', { class: 'se-grow' }), runsOn),
    h('div', { class: 'se-bar' }, ui.buttons(runBtn, stopBtn), status, h('span', { class: 'se-grow' }),
      ui.buttons(examplesBtn, apiBtn)),
    ask,
    h('div', { class: 'se-work' }, editor.el,
      h('div', { class: 'se-out-box' },
        h('div', { class: 'se-out-head' }, h('span', { class: 'se-out-title', text: 'Output' }),
          ui.button({ label: 'Clear', onClick: () => api.call('clear') })),
        out),
      drawerBox)));
  root.addEventListener('keydown', e => { if (e.key === 'Escape' && drawer) { drawer = null; drawDrawer(); } });
  editor.setValue(draft.code);
  drawFile();

  fetch(new URL('api_reference.json', import.meta.url)).then(r => r.json()).then(j => {
    entries = j.entries || [];
    for (const e of entries) if (e.name.indexOf('.') < 0) apiNames.add(e.name);
    editor.refresh();
    if (drawer === 'api') drawDrawer();
  }).catch(() => { entries = null; if (drawer === 'api') drawDrawer(); });

  // ── The script ──
  function setDraft(d) { draft = d; keepDraft(d); editor.setValue(d.code); drawFile(); editor.focus(); }
  function saved(name, code) { draft = Object.assign({}, draft, { name, saved: code }); keepDraft(draft); drawFile(); }
  function drawFile() {
    dot.hidden = !dirty();
    deleteBtn.disabled = !draft.name;
    if (document.activeElement !== pick) pick.value = draft.name;
  }
  /** Do `then` now, or once the reader agrees to lose the unsaved changes. */
  function discardThen(then) {
    if (!dirty()) return then();
    api.confirm(ask, { title: 'Discard the unsaved changes to “' + label() + '”?', label: 'Discard', danger: true, onConfirm: then });
  }
  function choose(name) {
    pick.value = draft.name;                       // stays on the open script until the other one is open
    if (!name) return discardThen(() => setDraft({ name: '', code: STARTER, saved: STARTER }));
    discardThen(async () => {
      const r = await api.call('open', { name });
      if (r) setDraft({ name: r.name, code: r.code, saved: r.code });
    });
  }
  async function save() {
    if (!draft.name) return saveAs();
    const code = editor.getValue();
    const r = await api.call('save', { name: draft.name, code });
    if (r) { saved(r.name, code); api.toast('Saved ' + r.name); }
  }
  function saveAs() {
    const box = ui.text({ value: draft.name, placeholder: 'Name: letters, digits, _ or -' });
    const close = () => { ask.textContent = ''; editor.focus(); };
    const go = async () => {
      const code = editor.getValue();
      const r = await api.call('save_as', { name: box.value.trim(), code });
      if (r) { close(); saved(r.name, code); api.toast('Saved as ' + r.name); }
    };
    box.addEventListener('keydown', e => {
      if (e.key === 'Enter') { e.preventDefault(); go(); } else if (e.key === 'Escape') { e.stopPropagation(); close(); }
    });
    ask.replaceChildren(h('div', { class: 'se-saveas' }, h('label', { text: 'Save as' }), box,
      ui.buttons(ui.button({ label: 'Save', kind: 'primary', onClick: go }), ui.button({ label: 'Cancel', onClick: close }))));
    box.focus();
    box.select();
  }
  function remove() {
    if (!draft.name) return;
    const name = draft.name;
    api.confirm(ask, {
      title: 'Delete the script “' + name + '”?', lines: ['Its text stays in the editor, as Untitled.'], label: 'Delete', danger: true,
      onConfirm: async () => { const r = await api.call('delete', { name }); if (r) { saved('', null); api.toast('Deleted ' + name); } },
    });
  }

  // ── Running ──
  function run() {
    if (!s || s.running) return;
    const code = editor.getValue();
    const go = hardware => api.call('run', { code, hardware_ok: hardware });
    if (s.target === 'hardware') {
      api.confirm(ask, { title: 'Run on the real robot?', lines: [s.robot], label: 'Run on hardware', danger: true, onConfirm: () => go(true) });
    } else {
      go(false);
    }
  }
  function drawOutput() {
    if (s.output_seq === outSeq) return;
    outSeq = s.output_seq;
    const atEnd = out.scrollTop + out.clientHeight >= out.scrollHeight - 4;
    out.replaceChildren(...s.output.map(line => {
      const n = scriptLine(line);
      return n ? h('div', { class: 'se-jump', title: 'Show line ' + n + ' in the editor', text: line, onclick: () => editor.mark(n) })
        : h('div', { text: line || ' ' });
    }));
    if (atEnd) out.scrollTop = out.scrollHeight;
  }

  // ── Drawers ──
  function toggle(which) { drawer = drawer === which ? null : which; drawDrawer(); }
  function openExample(name) {
    discardThen(async () => {
      const r = await api.call('example', { name });
      if (r) { setDraft({ name: '', code: r.code, saved: r.code }); drawer = null; drawDrawer(); }
    });
  }
  function drawDrawer() {
    examplesBtn.classList.toggle('on', drawer === 'examples');
    apiBtn.classList.toggle('on', drawer === 'api');
    drawerBox.hidden = !drawer;
    if (!drawer) return;
    const head = title => h('div', { class: 'se-drawer-head' }, h('h3', { text: title }),
      ui.button({ label: 'Close', onClick: () => toggle(drawer) }));
    if (drawer === 'examples') {
      const list = s ? s.examples : [];
      examplesSig = JSON.stringify(list);
      drawerBox.replaceChildren(head('Examples'), ui.note('Opens a copy as Untitled; the example itself is not changed.'),
        h('div', { class: 'se-list' }, ...(list.length ? list.map(x => h('button', { class: 'se-item', type: 'button', onclick: () => openExample(x.name) },
          h('span', { class: 'se-item-name', text: x.name }), h('span', { class: 'se-item-doc', text: x.description }))) : [ui.note('No examples found.')])));
      return;
    }
    const search = ui.text({ placeholder: 'Search the robot API' });
    const list = h('div', { class: 'se-list' });
    const fill = () => {
      if (entries === null) { list.replaceChildren(ui.note('The API list could not be loaded.', 'bad')); return; }
      const q = search.value.trim().toLowerCase();
      const rows = [];
      let group = null;
      for (const e of entries) {
        if (q && (e.name + e.signature + ' ' + e.doc).toLowerCase().indexOf(q) < 0) continue;
        const g = groupOf(e.name);
        if (g !== group) { group = g; rows.push(h('div', { class: 'se-group', text: g })); }
        rows.push(h('button', { class: 'se-item', type: 'button', title: 'Insert at the cursor', onclick: () => editor.insert(insertText(e)) },
          h('span', { class: 'se-item-name', text: e.name + e.signature }), h('span', { class: 'se-item-doc', text: e.doc })));
      }
      list.replaceChildren(...(rows.length ? rows : [ui.note(entries.length ? 'Nothing matches.' : 'Loading…')]));
    };
    search.addEventListener('input', fill);
    drawerBox.replaceChildren(head('Robot API'), search, list);
    fill();
    search.focus();
  }

  return {
    update(state) {
      s = state;
      pick.setOptions([{ value: '', label: 'Untitled' }].concat(s.scripts.map(n => ({ value: n, label: n }))), draft.name);
      const target = s.target || 'simulation';
      runsOn.textContent = 'Runs on: ' + target.charAt(0).toUpperCase() + target.slice(1) + ' · ' + s.robot;
      runsOn.classList.toggle('hw', target === 'hardware');
      runBtn.disabled = s.running;
      stopBtn.disabled = !s.running;
      let text = STATUS[s.status] || s.status;
      if (s.status === 'running' || s.status === 'finished') text += ' · ' + Math.round(s.elapsed_s) + ' s';
      if (s.status === 'failed' && s.error_line) text += ' at line ' + s.error_line;
      status.textContent = text;
      status.className = 'se-status ' + s.status;
      if (s.status === 'failed' && lastStatus !== null && lastStatus !== 'failed' && s.error_line) editor.mark(s.error_line);
      lastStatus = s.status;
      drawOutput();
      if (drawer === 'examples' && JSON.stringify(s.examples) !== examplesSig) drawDrawer();
    },
    dispose() {},
  };
}
