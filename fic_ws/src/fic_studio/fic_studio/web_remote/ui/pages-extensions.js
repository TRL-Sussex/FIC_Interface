/* Extensions page: one extension at a time, chosen in the side list.

   Two kinds of extension page (fic_docs/pages/extensions.html):
   - an extension with its own web page (manifest "web") is loaded as an ES module
     from /ext/<id>/<page>; its default export build(root, api) draws it with the
     UI kit (FIC.ui) and gets web_state() every poll_ms, and api.call() runs
     web_action() through the `ext` command;
   - any other extension's Qt page is mirrored: the Studio describes its widgets
     (web_remote/qtmirror.py: /api/panel) and this file draws them with the same
     kit — group boxes as section cards, label + field rows, slider cards, button
     rows, logs and bars — and sends each change back through the `panel` command.
   A control the mirror cannot draw says so instead of vanishing. */
(() => {
'use strict';
const { $, h, fmt, setText, pageTitle, post, toast } = window.FIC;
const ui = () => window.FIC.ui;

window.FIC.pages.extensions = function buildExtensions(root) {
  const panelBox = h('div', { id: 'ext-panel' });
  root.append(...pageTitle('Extensions', 'Pages from the Studio’s extensions.'), panelBox);
  let list = [], sel = null, dead = false, page = null;       // page: the open extension's controller
  const selected = () => new URLSearchParams((location.hash.split('?')[1] || '')).get('id');

  async function get(url) { try { return await (await fetch(url)).json(); } catch (e) { return { ok: false, message: String(e) }; } }
  async function loadList() {
    const r = await get('/api/panels');
    // Which extensions the Studio loads is chosen in Settings > Extensions.
    list = ((r.ok && r.panels) || []).filter(p => p.section !== 'Studio');   // the Studio's own pages have their own web pages
    const visible = list.filter(p => !p.hidden);
    const want = selected() || (visible[0] && visible[0].id);
    if (want) open(want);
    else panelBox.replaceChildren(h('div', { class: 'empty', text: r.ok === false ? (r.message || 'Extensions are not available.')
      : 'No extensions are on. Turn some on in Settings > Extensions.' }));
  }
  window.addEventListener('hashchange', onHash);
  function onHash() { if (!dead && location.hash.startsWith('#/extensions')) { const id = selected(); if (id && id !== sel) open(id); } }

  async function open(id) {
    if (page) { page.dispose(); page = null; }
    sel = id;
    const meta = list.find(p => p.id === id) || { id };
    const title = $('.title'); if (title) setText(title, meta.label || 'Extensions');
    setText($('#lede'), meta.description || '');
    panelBox.replaceChildren(h('div', { class: 'empty', text: 'Loading…' }));
    page = meta.web ? await webPage(meta) : mirrorPage(meta);
  }

  // ── An extension's own web page ────────────────────────────────────────
  async function webPage(meta) {
    const id = meta.id, box = h('div', { class: 'ext-web' });
    let css = null, timer = null, mod = null, last = null, gone = false, busy = false;
    if (meta.web.css) { css = h('link', { rel: 'stylesheet', href: meta.web.css }); document.head.append(css); }
    const api = {
      id,
      ui: ui(), h, toast, fmt,
      state: () => last,
      /** Ask before something that matters, in place under `host` (no pop-up): confirm(host, { title, lines, label, danger, onConfirm }). */
      confirm: (host, spec) => window.FIC.inlineConfirm(host, spec),
      studio: () => window.FIC.state(),
      /** Run one of the extension's web_action()s; resolves to its result, or null when refused (already toasted). */
      async call(name, args) {
        const r = await post('/api/command', { action: 'ext', args: { extension: id, name, args: args || {} } });
        if (r.ok === false) { toast(r.message || 'The Studio refused that', true); return null; }
        poll();
        return r.result === undefined ? null : r.result;
      },
    };
    async function poll() {
      if (gone || busy) return;
      busy = true;
      const r = await get('/api/ext_state?id=' + encodeURIComponent(id));
      busy = false;
      if (gone || !r.ok) return;
      last = r.state;
      if (mod && mod.update) { try { mod.update(last, window.FIC.state()); } catch (e) { console.error(e); } }
    }
    try {
      const m = await import(meta.web.page);
      if (gone) return { dispose() {} };
      if (typeof m.default !== 'function') throw new Error(meta.web.page + ' has no default export build(root, api)');
      panelBox.replaceChildren(box);
      mod = m.default(box, api) || {};
    } catch (e) {
      panelBox.replaceChildren(ui().section('This page could not load', ui().note(String(e && e.message ? e.message : e), 'bad'),
        h('p', { class: 'faint', text: 'Check the browser console and the extension’s web/ folder.' })));
    }
    poll(); timer = setInterval(poll, meta.web.poll_ms || 250);
    return {
      update() { if (mod && mod.studio) { try { mod.studio(window.FIC.state()); } catch (e) { /* page's own */ } } },
      dispose() { gone = true; clearInterval(timer); if (css) css.remove(); if (mod && mod.dispose) { try { mod.dispose(); } catch (e) { /* already gone */ } } },
    };
  }

  // ── A mirrored Qt page ─────────────────────────────────────────────────
  function mirrorPage(meta) {
    const id = meta.id;
    let last = '', timer = null, busy = false, gone = false;
    async function refresh(force) {
      if (busy || gone) return;
      const r = await get('/api/panel?id=' + encodeURIComponent(id));
      if (gone) return;
      if (!r.ok) { if (force) panelBox.replaceChildren(h('div', { class: 'empty', text: r.message || 'This page is not available.' })); return; }
      draw(r.panel, force);
    }
    function draw(panel, force) {
      const text = JSON.stringify(panel.nodes);
      if (text === last && !force) return;
      if (!force && panelBox.contains(document.activeElement) && /^(INPUT|SELECT)$/.test(document.activeElement.tagName)) return;   // do not pull a field out from under a hand
      last = text;
      panelBox.replaceChildren(h('div', { class: 'ext-form' }, ...cards(panel.nodes, panel)));
    }
    async function act(panel, n, kind, value) {
      busy = true;
      const r = await post('/api/command', { action: 'panel', args: { extension: panel.id, path: n.path, kind, value } });
      busy = false;
      if (r.ok === false) { toast(r.message || 'The Studio refused that', true); if (r.error === 'stale_panel') refresh(true); return; }
      if (r.panel) draw(r.panel, true);
    }

    /** Top level, as section cards. Outer frames are unwrapped; each group box, frame of
     *  controls, tab widget and heading label (bold or large on the Qt page) starts a card,
     *  titled when it has a title; loose controls share the card before them. The page's own
     *  title, repeated as the Qt page's first label, is left out. */
    function cards(list, panel) {
      const isBox = n => n.kind === 'group' || n.kind === 'box';
      while (list.length === 1 && isBox(list[0]) && !list[0].label) list = list[0].nodes || [];
      // An untitled frame that only holds other frames: its frames are cards of their own.
      const lift = l => l.flatMap(n => isBox(n) && !n.label && (n.nodes || []).length > 1 && n.nodes.every(c => isBox(c) || c.kind === 'other') ? lift(n.nodes) : [n]);
      list = lift(list);
      const pageTitle = String(meta.label || '').trim().toLowerCase();
      list = list.filter((n, i) => !(i < 2 && n.kind === 'label' && n.label.trim().toLowerCase() === pageTitle));
      const out = []; let title = null, body = [];
      const flush = () => {
        if (!body.length && !title) return;
        const content = h('div', { class: 'xbody' }, ...nodes(body, panel));
        out.push(title ? ui().section(FIC.tidyLabel(title).text, content) : h('section', { class: 'section' }, content));
        title = null; body = [];
      };
      for (const n of list) {
        if (isBox(n) && (n.label || (n.nodes || []).length > 1)) {
          flush();
          const inner = n.nodes || [], head = !n.label && inner[0] && inner[0].kind === 'label' && inner[0].heading ? inner[0] : null;
          title = n.label || (head && head.label) || null; body = head ? inner.slice(1) : inner; flush();
        } else if (n.kind === 'tabs') { flush(); body = [n]; flush(); }
        else if (n.kind === 'label' && n.heading) { flush(); title = n.label; }
        else body.push(n);
      }
      flush();
      return out;
    }
    /** Consecutive buttons sit side by side; consecutive sliders share a grid of slider cards. */
    // A short plain label followed by another plain label is a name and its value
    // ("State" / "stopped", "Head pose:" / "yaw=… pitch=…"): one row, as a form shows it.
    const isName = n => n && n.kind === 'label' && !n.heading && n.label.trim().length >= 2 && n.label.length <= 28 && !/[.!?]$/.test(n.label.trim());
    const isValue = n => n && n.kind === 'label' && !n.heading && n.label.length <= 80;
    function pairs(list) {
      const out = [];
      for (let i = 0; i < list.length; i++) {
        if (isName(list[i]) && isValue(list[i + 1])) {
          out.push({ kind: 'kv', label: list[i].label, value: list[i + 1].label }); i++;
        } else out.push(list[i]);
      }
      return out;
    }
    function nodes(list, panel) {
      list = pairs(list);
      const out = []; let run = null, runKind = '';
      for (const n of list) {
        const k = n.kind === 'button' ? 'button' : n.kind === 'slider' ? 'slider' : '';
        if (k && k === runKind) { run.append(node(n, panel)); continue; }
        run = null; runKind = '';
        if (k) { run = h('div', { class: k === 'button' ? 'btn-row' : 'kit-grid', style: k === 'slider' ? '--cols:3' : null }); runKind = k; out.push(run); run.append(node(n, panel)); }
        else out.push(node(n, panel));
      }
      return out;
    }
    const tidy = l => String(l || '').trim().replace(/:$/, '');
    function node(n, panel) {
      const K = ui(), off = n.enabled === false, tip = n.tip || null;
      switch (n.kind) {
        case 'group': case 'box': return h('div', { class: 'xgroup' }, n.label ? h('h3', { class: 'xsub', text: n.label }) : null, h('div', { class: 'xbody' }, ...nodes(n.nodes || [], panel)));
        case 'tabs': return h('div', { class: 'xgroup' },
          h('div', { class: 'seg xtabs' }, ...n.tabs.map((t, i) => h('button', { type: 'button', text: FIC.tidyLabel(t.label).text, 'aria-pressed': String(!!t.current), onclick: () => act(panel, n, 'tabs', i) }))),
          h('div', { class: 'xbody' }, ...nodes(n.nodes || [], panel)));
        case 'button': { const t = FIC.tidyLabel(n.label); return K.button({ label: t.text, icon: t.name, kind: 'primary', title: tip, disabled: off, onClick: () => act(panel, n, 'button') }); }
        case 'toggle': { const el = K.toggle({ label: n.label, value: n.value, disabled: off, title: tip, onChange: v => act(panel, n, 'toggle', v) }); return h('div', { class: 'kit-field kit-wide' }, el); }
        case 'choice': return K.field(tidy(n.label), K.select({ options: n.options.map((o, i) => ({ value: i, label: o })), value: n.value, disabled: off, onChange: v => act(panel, n, 'choice', Number(v)) }));
        case 'number': return K.field(tidy(n.label), K.number({ value: n.value, min: n.min, max: n.max, step: n.step, unit: n.suffix, disabled: off, onChange: v => act(panel, n, 'number', v) }));
        case 'slider': { const el = K.slider({ label: n.label || 'Value', value: n.value, min: n.min, max: n.max, step: 1, digits: 0, disabled: off, onChange: v => act(panel, n, 'slider', v) }); el.title = tip || ''; return el; }
        case 'text': return K.field(tidy(n.label), K.text({ value: n.value, disabled: off, onChange: v => act(panel, n, 'text', v) }));
        case 'kv': return K.field(tidy(n.label), h('span', { class: 'xkv', text: n.value }));
        case 'label': return n.heading ? h('h3', { class: 'xsub', text: n.label }) : h('p', { class: 'xlabel', text: n.label });
        case 'list': {
          const box = h('div', { class: 'xlist' + (off ? ' off' : ''), role: 'listbox', 'aria-label': n.label || 'List' },
            ...(n.items.length ? n.items.map((t, i) => h('button', { type: 'button', class: 'xlist-row' + (i === n.value ? ' on' : ''), role: 'option',
              'aria-selected': String(i === n.value), text: t, disabled: off, onclick: () => act(panel, n, 'list', i) })) : [h('div', { class: 'faint xlabel', text: 'Empty' })]));
          return n.label ? K.field(n.label, box) : box;
        }
        case 'table': {
          const cols = n.columns.length ? n.columns : (n.rows[0] || []).map((_, i) => 'Column ' + (i + 1));
          const t = K.table({ columns: cols.map((c, i) => ({ key: String(i), label: c })), empty: 'Empty',
            rows: n.rows.map(r => Object.fromEntries(r.map((v, i) => [String(i), v]))) });
          return h('div', { class: 'xtable' }, n.label ? h('div', { class: 'xlabel', text: n.label }) : null, t);
        }
        case 'log': { const l = K.log({ lines: String(n.value || '').split('\n') }); return n.label ? K.field(n.label, l) : l; }
        case 'bar': { const pct = n.max > n.min ? (n.value - n.min) / (n.max - n.min) : 0;
          return K.bar({ label: (n.label || '').replace('%p%', fmt(pct * 100, 0) + ' %').replace('%v', String(n.value)), value: pct }); }
        default: return h('p', { class: 'faint xlabel', text: (n.label || 'A control') + ' cannot be shown in the browser.' });
      }
    }
    refresh(true);
    timer = setInterval(() => refresh(false), 1500);
    return { update() {}, dispose() { gone = true; clearInterval(timer); } };
  }

  loadList();
  return {
    update(s) { if (page && page.update) page.update(s); },
    dispose() { dead = true; if (page) page.dispose(); window.removeEventListener('hashchange', onHash); },
  };
};
})();
