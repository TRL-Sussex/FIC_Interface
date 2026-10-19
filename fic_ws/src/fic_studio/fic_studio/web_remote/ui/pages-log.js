/* Log page: the Studio's log, filtered by level, source and text, optionally
   followed. Lines arrive in the state stream (each has a sequence number); the
   first screen is back-filled with one `log_read`. Filtering is only display. */
(() => {
'use strict';
const { $, h, setText, section, pageTitle, cmd, post, toast, inlineConfirm } = window.FIC;
const LEVELS = ['ERROR', 'WARN', 'INFO', 'DEBUG'];
const MAX_KEEP = 1000;

window.FIC.pages.log = function buildLog(root) {
  const st = { lines: [], last: 0, levels: new Set(['ERROR', 'WARN', 'INFO']), query: '', source: '', follow: true, filling: false };
  try { const saved = JSON.parse(localStorage.getItem('fic-log') || 'null'); if (saved) { st.levels = new Set(saved.levels); st.follow = saved.follow !== false; } } catch (e) { /* none */ }
  const remember = () => { try { localStorage.setItem('fic-log', JSON.stringify({ levels: [...st.levels], follow: st.follow })); } catch (e) { /* none */ } };

  const view = h('div', { class: 'logview', id: 'logview', tabindex: '0', role: 'log', 'aria-live': 'off' });
  const levelBox = h('div', { class: 'seg', role: 'group', 'aria-label': 'Levels' });
  const query = h('input', { class: 'input', type: 'search', placeholder: 'Filter text', 'aria-label': 'Filter text', style: 'width:14rem' });
  const source = h('select', { class: 'select', 'aria-label': 'Source' }, h('option', { value: '', text: 'All sources' }));
  const follow = h('label', { class: 'toggle' }, h('input', { type: 'checkbox', id: 'log-follow' }), h('span', { class: 'track' }), h('span', { text: 'Follow' }));
  const ask = h('div');
  // Two views: the Studio's log (kept in memory, can be cleared) and the audit log
  // (a file of what was done to the robots, by whom; it cannot be cleared from here).
  const tabs = h('div', { class: 'seg log-tabs', role: 'tablist', 'aria-label': 'Which log' });
  const logPane = h('div', { id: 'log-pane' });
  const auditPane = h('div', { id: 'audit-pane', hidden: true });
  root.append(
    ...pageTitle('Log', 'Everything the Studio reports, newest at the bottom.'),
    tabs, logPane, auditPane);
  logPane.append(
    h('div', { class: 'toolbar' },
      h('div', { class: 'field' }, h('label', { text: 'Levels' }), levelBox),
      h('div', { class: 'field' }, h('label', { text: 'Text' }), query),
      h('div', { class: 'field' }, h('label', { text: 'Source' }), source),
      h('div', { class: 'field' }, h('label', { text: 'Scroll' }), h('div', { style: 'min-height:31px;display:flex;align-items:center' }, follow)),
      h('div', { class: 'spacer' }),
      h('div', { class: 'btn-row' },
        h('button', { class: 'btn', type: 'button', text: 'Save', onclick: save }),
        h('button', { class: 'btn btn-danger', type: 'button', text: 'Clear', onclick: askClear }))),
    ask, h('div', { class: 'aside-line faint', id: 'log-count' }), view);
  const audit = buildAudit(auditPane);
  let tab = 'log';
  try { tab = localStorage.getItem('fic-log-tab') === 'audit' ? 'audit' : 'log'; } catch (e) { /* none */ }
  for (const [id, label] of [['log', 'Studio log'], ['audit', 'Audit']]) {
    tabs.append(h('button', { type: 'button', role: 'tab', text: label, 'data-v': id, onclick: () => showTab(id) }));
  }
  function showTab(id) {
    tab = id;
    try { localStorage.setItem('fic-log-tab', id); } catch (e) { /* none */ }
    for (const b of tabs.children) { b.setAttribute('aria-pressed', String(b.dataset.v === id)); b.setAttribute('aria-selected', String(b.dataset.v === id)); }
    logPane.hidden = id !== 'log'; auditPane.hidden = id !== 'audit';
    if (id === 'audit') audit.load(); else if (st.follow) toBottom();
  }
  showTab(tab);

  for (const lv of LEVELS) {
    levelBox.append(h('button', { type: 'button', text: lv, 'data-v': lv, onclick: () => {
      st.levels.has(lv) ? st.levels.delete(lv) : st.levels.add(lv); remember(); paintLevels(); rebuild();
    } }));
  }
  function paintLevels() { for (const b of levelBox.children) b.setAttribute('aria-pressed', String(st.levels.has(b.dataset.v))); }
  paintLevels();
  const followBox = follow.querySelector('input'); followBox.checked = st.follow;
  followBox.addEventListener('change', () => { st.follow = followBox.checked; remember(); if (st.follow) toBottom(); });
  query.addEventListener('input', () => { st.query = query.value.toLowerCase(); rebuild(); });
  source.addEventListener('change', () => { st.source = source.value; rebuild(); });
  view.addEventListener('scroll', () => {
    const atEnd = view.scrollHeight - view.scrollTop - view.clientHeight < 24;
    if (!atEnd && st.follow && !st.programmatic) { st.follow = false; followBox.checked = false; remember(); }
    st.programmatic = false;
  });

  const shown = e => st.levels.has(e.level) && (!st.source || e.source === st.source) &&
    (!st.query || (e.message + ' ' + e.source).toLowerCase().includes(st.query));
  const row = e => h('div', { class: 'logline ' + e.level },
    h('span', { class: 'lt', text: e.time }), h('span', { class: 'll', text: e.level }),
    h('span', { class: 'ls', text: e.source }), h('span', { class: 'lm', text: e.message }));
  function toBottom() { st.programmatic = true; view.scrollTop = view.scrollHeight; }
  function count() {
    const n = [...view.children].length;
    setText($('#log-count'), n + ' of ' + st.lines.length + ' lines shown');
  }
  function rebuild() {
    view.replaceChildren(...st.lines.filter(shown).map(row));
    if (!view.children.length) view.append(h('div', { class: 'empty', text: st.lines.length ? 'No line matches the filter.' : 'No log lines yet.' }));
    if (st.follow) toBottom(); count();
  }
  function sources() {
    const names = [...new Set(st.lines.map(e => e.source).filter(Boolean))].sort();
    if (source.__sig === names.join('|')) return;
    source.__sig = names.join('|');
    source.replaceChildren(h('option', { value: '', text: 'All sources' }), ...names.map(n => h('option', { value: n, text: n })));
    source.value = st.source;
  }
  function add(entries) {
    const fresh = entries.filter(e => (e.seq || 0) > st.last);
    if (!fresh.length) return;
    st.last = Math.max(st.last, ...fresh.map(e => e.seq || 0));
    st.lines.push(...fresh);
    if (st.lines.length > MAX_KEEP) st.lines.splice(0, st.lines.length - MAX_KEEP);
    const keep = fresh.filter(shown);
    if (keep.length) {
      if (view.firstChild && view.firstChild.classList.contains('empty')) view.textContent = '';
      for (const e of keep) view.append(row(e));
      while (view.children.length > MAX_KEEP) view.removeChild(view.firstChild);
      if (st.follow) toBottom();
    }
    sources(); count();
  }
  async function backfill() {
    st.filling = true;
    const initial = !st.last;
    const r = await cmd('log_read', { after: st.last || null, limit: 500 }, true);
    st.filling = false;
    if (r.ok) { if (initial) { st.lines = []; st.last = 0; } add(r.entries); if (initial) rebuild(); sources(); }
  }
  async function save() {
    const text = st.lines.filter(shown).map(e => '[' + e.time + '] ' + e.level.padEnd(5) + ' ' + e.source + ' ' + e.message).join('\n') + '\n';
    const a = h('a', { href: URL.createObjectURL(new Blob([text], { type: 'text/plain' })), download: 'fic_studio_log.txt' });
    document.body.append(a); a.click(); a.remove(); toast('Saved ' + st.lines.filter(shown).length + ' lines');
  }
  function askClear() {
    inlineConfirm(ask, { title: 'Clear the Studio log?', lines: ['This empties the log for everyone, including the desktop window.'], label: 'Clear log', danger: true,
      onConfirm: async () => { const r = await cmd('log_clear', { confirm: true }); if (r.ok) { st.lines = []; st.last = 0; rebuild(); backfill(); } } });
  }
  backfill();
  rebuild();
  const timer = setInterval(() => { if (tab === 'audit' && document.visibilityState === 'visible') audit.load(true); }, 5000);
  return { dispose() { clearInterval(timer); }, update: s => {
    const incoming = s.log || [];
    // A gap (more new lines than the stream carries) is filled by one read.
    if (incoming.length && st.last && incoming[0].seq > st.last + 1 && !st.filling) { backfill(); return; }
    add(incoming);
    if (tab === 'log') setText($('#lede'), (s.studio && s.studio.host ? s.studio.host + ' · ' : '') + st.lines.length + ' lines kept');
  } };
};

// ── Audit: E-STOP and reset, sessions, processes, targets, teleop, control, users ──
const EVENT_LABEL = {
  estop: 'E-STOP', estop_reset: 'E-STOP reset', target: 'Target', control_taken: 'Control',
  session_start: 'Session start', session_stop: 'Session stop', process_start: 'Process start',
  process_stop: 'Process stop', teleop_request: 'Teleop', user_changed: 'Users',
  estop_all: 'E-STOP all', pause_all: 'Pause all', resume_all: 'Resume all',
};
const BAD_EVENTS = new Set(['estop', 'estop_all']), WARN_EVENTS = new Set(['estop_reset', 'target', 'user_changed']);
const SKIP = new Set(['t', 'event', 'who', 'detail']);

function buildAudit(pane) {
  const st = { rows: [], query: '', event: '', path: '' };
  const query = h('input', { class: 'input', type: 'search', placeholder: 'Filter text', 'aria-label': 'Filter audit text', style: 'width:14rem' });
  const event = h('select', { class: 'select', 'aria-label': 'Event' }, h('option', { value: '', text: 'All events' }));
  const list = h('div', { class: 'logview audit-view', role: 'log', tabindex: '0', 'aria-live': 'off' });
  const note = h('div', { class: 'aside-line faint' });
  pane.append(
    h('div', { class: 'toolbar' },
      h('div', { class: 'field' }, h('label', { text: 'Event' }), event),
      h('div', { class: 'field' }, h('label', { text: 'Text' }), query),
      h('div', { class: 'spacer' }),
      h('div', { class: 'btn-row' },
        h('button', { class: 'btn', type: 'button', text: 'Refresh', onclick: () => load() }),
        h('button', { class: 'btn', type: 'button', text: 'Save', onclick: save }))),
    note, list);
  query.addEventListener('input', () => { st.query = query.value.toLowerCase(); paint(); });
  event.addEventListener('change', () => { st.event = event.value; paint(); });

  const extra = e => Object.keys(e).filter(k => !SKIP.has(k)).map(k => k + '=' + (typeof e[k] === 'object' ? JSON.stringify(e[k]) : e[k])).join('  ');
  const text = e => [e.t, e.event, e.who, e.detail, extra(e)].join(' ').toLowerCase();
  const shown = e => (!st.event || e.event === st.event) && (!st.query || text(e).includes(st.query));
  const when = t => { const m = /^(\d{4}-\d\d-\d\d)T(\d\d:\d\d:\d\d)/.exec(t || ''); return m ? m[1] + ' ' + m[2] : (t || ''); };
  const row = e => h('div', { class: 'auditline' + (BAD_EVENTS.has(e.event) ? ' ERROR' : WARN_EVENTS.has(e.event) ? ' WARN' : '') },
    h('span', { class: 'lt', text: when(e.t) }), h('span', { class: 'll', text: EVENT_LABEL[e.event] || e.event }),
    h('span', { class: 'ls', text: e.who || '' }), h('span', { class: 'lm', text: [e.detail, extra(e)].filter(Boolean).join('  ·  ') }));

  function events() {
    const names = [...new Set(st.rows.map(e => e.event))].sort();
    if (event.__sig === names.join('|')) return;
    event.__sig = names.join('|');
    event.replaceChildren(h('option', { value: '', text: 'All events' }), ...names.map(n => h('option', { value: n, text: EVENT_LABEL[n] || n })));
    event.value = names.includes(st.event) ? st.event : '';
  }
  function paint() {
    const keep = st.rows.filter(shown);
    const atEnd = list.scrollHeight - list.scrollTop - list.clientHeight < 24;
    list.replaceChildren(...keep.map(row));
    if (!keep.length) list.append(h('div', { class: 'empty', text: st.rows.length ? 'No entry matches the filter.' : 'Nothing recorded yet.' }));
    if (atEnd || !list.__painted) list.scrollTop = list.scrollHeight;
    list.__painted = true;
    setText(note, keep.length + ' of ' + st.rows.length + ' entries · ' + (st.path || 'audit log') + ' · read-only: it cannot be cleared from here');
    setText($('#lede'), 'What was done to the robots, and by whom. Kept on the Studio machine across restarts.');
  }
  async function load(quiet) {
    const r = await cmd('audit_read', { limit: 2000 }, true);
    if (!r.ok) { if (!quiet) setText(note, r.error === 'not_allowed' ? 'Your account does not include the log.' : (r.message || 'Could not read the audit log.')); return; }
    const sig = r.entries.length + ':' + (r.entries.length ? r.entries[r.entries.length - 1].t : '');
    st.path = r.path || '';
    if (sig === st.sig) return;
    st.sig = sig; st.rows = r.entries; events(); paint();
  }
  function save() {
    const body = st.rows.filter(shown).map(e => JSON.stringify(e)).join('\n') + '\n';
    const a = h('a', { href: URL.createObjectURL(new Blob([body], { type: 'application/x-ndjson' })), download: 'fic_studio_audit.jsonl' });
    document.body.append(a); a.click(); a.remove(); toast('Saved ' + st.rows.filter(shown).length + ' entries');
  }
  return { load };
}
})();
