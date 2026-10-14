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
  root.append(
    ...pageTitle('Log', 'Everything the Studio reports, newest at the bottom.'),
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
  return s => {
    const incoming = s.log || [];
    // A gap (more new lines than the stream carries) is filled by one read.
    if (incoming.length && st.last && incoming[0].seq > st.last + 1 && !st.filling) { backfill(); return; }
    add(incoming);
    setText($('#lede'), (s.studio && s.studio.host ? s.studio.host + ' · ' : '') + st.lines.length + ' lines kept');
  };
};
})();
