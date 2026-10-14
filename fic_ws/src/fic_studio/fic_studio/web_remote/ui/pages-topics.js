/* Topic Monitor: the ROS 2 topics the Studio can see, grouped by namespace,
   with a monitor switch on each, and the selected topic's fields and raw
   stream. The Studio does the subscribing, counting and message decoding
   (the same as the classic page); this page lists and shows. */
(() => {
'use strict';
const { $, h, fmt, setText, pageTitle, cmd, spark } = window.FIC;

window.FIC.pages.topic_monitor = function buildTopics(root) {
  const st = { topics: [], filter: '', role: 'all', selected: null, since: 0, hz: [], auto: true, paused: false, lines: [], buffer: 200, openNs: new Set() };
  const tree = h('div', { class: 'tm-tree' }), fields = h('div', { class: 'tm-fields' }), raw = h('pre', { class: 'xlog tm-raw', tabindex: '0', 'aria-label': 'Raw stream' });
  const filter = h('input', { class: 'input', type: 'search', placeholder: 'Filter by name', 'aria-label': 'Filter' });
  const role = h('select', { class: 'select', 'aria-label': 'Role' }, ...[['all', 'All topics'], ['pub', 'Publishers only'], ['sub', 'Subscribers only'], ['service', 'Services'], ['action', 'Actions']].map(([v, l]) => h('option', { value: v, text: l })));
  const auto = h('input', { type: 'checkbox', checked: true });
  const buf = h('input', { class: 'input num', type: 'number', min: '10', max: '2000', step: '10', value: '200', 'aria-label': 'Buffer' });
  const sel = h('div', { class: 'tm-sel', text: 'Select a topic to inspect' }), selMeta = h('div', { class: 'faint num', style: 'font-size:var(--fs-m)' }), sparkBox = h('div', { class: 'tm-spark' });
  const counts = h('div', { class: 'faint num tm-counts' });
  const pauseBtn = h('button', { class: 'btn', type: 'button', icon: 'pause', text: 'Pause', onclick: () => { st.paused = !st.paused; pauseBtn.replaceChildren(FIC.icon(st.paused ? 'play' : 'pause'), st.paused ? 'Resume' : 'Pause'); } });
  root.append(...pageTitle('Topic Monitor', 'Every topic on this ROS network. Monitor one to see its fields and messages.'),
    h('div', { class: 'toolbar' }, h('div', { class: 'field' }, h('label', { text: 'Filter' }), filter), h('div', { class: 'field' }, h('label', { text: 'Role' }), role),
      h('button', { class: 'btn btn-primary', type: 'button', icon: 'refresh', text: 'Refresh', onclick: () => refresh() }),
      h('label', { class: 'toggle' }, auto, h('span', { class: 'track' }), h('span', { text: 'Auto-refresh' })), h('div', { class: 'spacer' }), counts),
    h('div', { class: 'tm-grid' }, tree,
      h('div', { class: 'tm-insp' }, sel, selMeta, h('div', { class: 'eyebrow', style: 'margin-top:var(--s4)', text: 'Arrival rate' }), sparkBox, h('div', { class: 'eyebrow', style: 'margin-top:var(--s4)', text: 'Fields' }), fields,
        h('div', { class: 'toolbar', style: 'margin-top:var(--s5)' }, h('div', { class: 'eyebrow', text: 'Raw stream' }), h('div', { class: 'spacer' }), h('div', { class: 'field' }, h('label', { text: 'Buffer' }), buf), pauseBtn,
          h('button', { class: 'btn', type: 'button', text: 'Clear', onclick: () => { st.lines = []; raw.textContent = ''; cmd('topics', { op: 'clear' }, true); } })), raw)));
  filter.addEventListener('input', () => { st.filter = filter.value.toLowerCase(); drawTree(); });
  role.addEventListener('change', () => { st.role = role.value; drawTree(); });
  buf.addEventListener('change', () => { st.buffer = Math.max(10, Math.min(2000, parseInt(buf.value, 10) || 200)); });

  async function refresh() {
    const r = await cmd('topics', { op: 'list' }, true);
    if (!r.ok) { setText($('#lede'), r.message || 'The topic list is not available.'); return; }
    if (!st.seen) { st.seen = true; for (const t of r.topics) st.openNs.add(t.ns); }          // the first list opens every namespace
    st.topics = r.topics; setText(counts, 'Topics: ' + r.topics.length + '   Monitoring: ' + r.monitoring + '   Total messages: ' + r.total);
    drawTree();
  }
  const match = t => {
    if (st.filter && !t.name.toLowerCase().includes(st.filter)) return false;
    if (st.role === 'pub') return (t.pubs || 0) > 0; if (st.role === 'sub') return (t.subs || 0) > 0;
    if (st.role === 'service') return t.kind === 'service'; if (st.role === 'action') return t.kind === 'action';
    return true;
  };
  function drawTree() {
    const groups = new Map();
    for (const t of st.topics.filter(match)) { if (!groups.has(t.ns)) groups.set(t.ns, []); groups.get(t.ns).push(t); }
    tree.textContent = '';
    for (const [ns, items] of groups) {
      const open = st.openNs.has(ns) || !!st.filter || items.some(t => t.name === st.selected);
      const head = h('button', { class: 'tm-ns', type: 'button', 'aria-expanded': String(open), onclick: () => { if (st.openNs.has(ns)) st.openNs.delete(ns); else st.openNs.add(ns); drawTree(); } }, h('span', { class: 'caret tm-caret' }, FIC.icon(open ? 'chevron-down' : 'chevron-right')), h('span', { text: ns }), h('span', { class: 'faint', text: ' ' + items.length }));
      tree.append(head);
      if (!open) continue;
      for (const t of items) tree.append(h('div', { class: 'tm-row' + (t.name === st.selected ? ' on' : '') },
        h('button', { class: 'tm-name', type: 'button', title: t.type, onclick: () => selectTopic(t.name) }, h('span', { text: t.name.replace(/^\//, '') }), h('span', { class: 'faint', text: '  ' + (t.type || '').split('/').pop() })),
        h('span', { class: 'num faint tm-hz', text: t.hz ? fmt(t.hz, 1) + ' Hz' : '' }),
        h('span', { class: 'num faint', title: 'publishers / subscribers', text: (t.pubs ?? '-') + '/' + (t.subs ?? '-') }),
        h('button', { class: 'btn', type: 'button', text: t.monitored ? 'Stop' : 'Monitor', onclick: async () => { await cmd('topics', { op: 'monitor', name: t.name, value: !t.monitored }); refresh(); } })));
    }
    if (!groups.size) tree.append(h('div', { class: 'empty', text: st.topics.length ? 'No topic matches.' : 'No topics found yet.' }));
  }
  async function selectTopic(name) {
    st.selected = name; st.since = 0; st.hz = []; st.lines = []; raw.textContent = ''; fields.textContent = '';
    await cmd('topics', { op: 'select', name }); drawTree(); refresh(); inspect();
  }
  async function inspect() {
    if (!st.selected) return;
    const r = await cmd('topics', { op: 'inspect', since: st.since }, true);
    if (!r.ok || !r.topic) return;
    setText(sel, r.topic); setText(selMeta, r.type + ' · ' + fmt(r.hz, 1) + ' Hz · ' + r.count + ' messages');
    st.hz.push(r.hz); if (st.hz.length > 60) st.hz.shift(); sparkBox.replaceChildren(spark(st.hz));
    const sig = r.fields.map(f => f.name).join('|');
    if (fields.__sig !== sig) { fields.__sig = sig; fields.textContent = ''; for (const f of r.fields) fields.append(h('div', { class: 'tm-f', 'data-f': f.name }, h('span', { class: 'faint', text: f.name + ':' }), h('span', { class: 'num' }), f.number != null ? h('i', { class: 'lim' }, h('b')) : null)); }
    for (const f of r.fields) {
      const row = fields.querySelector('[data-f="' + CSS.escape(f.name) + '"]'); if (!row) continue;
      setText(row.children[1], f.value); const b = row.querySelector('.lim b'); if (b && f.number != null) b.style.width = Math.min(100, Math.abs(f.number) * 100) + '%';
    }
    if (!st.paused && r.lines.length) {
      st.lines.push(...r.lines.map(l => '[' + l.time + '] ' + l.text)); st.since = r.seq;
      if (st.lines.length > st.buffer) st.lines = st.lines.slice(-st.buffer);
      raw.textContent = st.lines.join('\n'); raw.scrollTop = raw.scrollHeight;
    } else if (st.paused) st.since = r.seq;
  }
  const t1 = setInterval(() => { if (auto.checked) refresh(); }, 2000), t2 = setInterval(inspect, 500);
  refresh();
  return { update() {}, dispose() { clearInterval(t1); clearInterval(t2); } };
};
})();
