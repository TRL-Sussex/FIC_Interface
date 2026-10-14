/* The UI kit: the pieces the Studio's own pages are made of, for extension web
   pages (FIC.ui) and for the mirrored Qt pages. Every builder returns a DOM
   element; the interactive ones also carry `.set(value)` (and the slider card
   `.setNow(value)`) so a page's update() can refresh them without rebuilding.
   Nothing here talks to the Studio: pages pass callbacks. See
   fic_docs/pages/extensions.html, "The UI kit". */
(() => {
'use strict';
const { h, setText, fmt } = window.FIC;

/** A framed section card with a heading; `aside` is short text on the right. */
function section(title, ...body) {
  let aside = null;
  if (body.length && body[0] && typeof body[0] === 'object' && !(body[0] instanceof Node) && 'aside' in body[0]) aside = body.shift().aside;
  const asideEl = h('span', { class: 'aside', text: aside || '' });
  const el = h('section', { class: 'section' }, h('div', { class: 'section-head' }, h('h2', { text: title }), asideEl), ...body.filter(Boolean));
  el.setAside = t => setText(asideEl, t == null ? '' : String(t));
  return el;
}

/** Cards in a responsive grid: `cols` across on a wide window, fewer on narrow ones. */
function grid(cols, ...children) {
  return h('div', { class: 'kit-grid', style: '--cols:' + (cols || 3) }, ...children.filter(Boolean));
}

/** Big readouts: stats([{ key, label, value, unit, note }]). `.set(key, value, note?)` updates one. */
function stats(items) {
  const cells = {};
  const el = h('div', { class: 'stats kit-stats' }, ...items.map(it => {
    const v = h('span', { text: it.value == null ? '—' : String(it.value) }), u = h('small', { text: it.unit || '' }), n = h('div', { class: 'n', text: it.note || '' });
    cells[it.key] = { v, n, d: it.digits };
    return h('div', { class: 'stat' }, h('div', { class: 'k', text: it.label }), h('div', { class: 'v' }, v, u), n);
  }));
  el.set = (key, value, note) => {
    const c = cells[key]; if (!c) return;
    setText(c.v, value == null ? '—' : (typeof value === 'number' ? fmt(value, c.d == null ? 2 : c.d) : String(value)));
    if (note !== undefined) setText(c.n, note == null ? '' : String(note));
  };
  return el;
}

/** A labelled control in a two-column row: field('Speed', control). */
function field(label, control, hint) {
  return h('div', { class: 'kit-field' }, h('label', { text: label }), h('div', { class: 'kit-ctl' }, control, hint ? h('div', { class: 'kit-hint faint', text: hint }) : null));
}

/** A button. kind: '' (quiet), 'primary' (tinted), 'solid' (the page's main action), 'danger'. */
function button({ label, icon, kind, title, disabled, onClick }) {
  const cls = 'btn' + (kind === 'primary' ? ' btn-primary' : kind === 'solid' ? ' btn-solid' : kind === 'danger' ? ' btn-danger' : '');
  const b = h('button', { class: cls, type: 'button', icon: icon || undefined, text: label, title: title || null, disabled: !!disabled, onclick: e => onClick && onClick(e) });
  b.set = ({ label: l, disabled: d } = {}) => { if (l != null) setText(b, l); if (d != null) b.disabled = !!d; };
  return b;
}
/** Buttons side by side. */
function buttons(...btns) { return h('div', { class: 'btn-row' }, ...btns.filter(Boolean)); }

/** An on/off switch. `.set(bool)`. */
function toggle({ label, value, disabled, title, onChange }) {
  const input = h('input', { type: 'checkbox', disabled: !!disabled });
  input.checked = !!value;
  input.addEventListener('change', () => onChange && onChange(input.checked));
  const el = h('label', { class: 'toggle', title: title || null }, input, h('span', { class: 'track' }), h('span', { text: label }));
  el.set = v => { if (document.activeElement !== input) input.checked = !!v; };
  el.input = input;
  return el;
}

/** A drop-down: options are strings or { value, label }. `.set(value)`, `.setOptions(list)`. */
function select({ options, value, disabled, onChange }) {
  const s = h('select', { class: 'select', disabled: !!disabled });
  const fill = list => { s.textContent = ''; for (const o of list || []) { const v = typeof o === 'object' ? o.value : o; s.append(h('option', { value: String(v), text: typeof o === 'object' ? (o.label != null ? o.label : String(v)) : String(o) })); } };
  fill(options); if (value != null) s.value = String(value);
  s.addEventListener('change', () => onChange && onChange(s.value));
  s.set = v => { if (document.activeElement !== s && v != null) s.value = String(v); };
  let sig = JSON.stringify(options || []);
  s.setOptions = (list, v) => {                      // rebuilt only when the list changes, so an open drop-down stays open
    const next = JSON.stringify(list || []);
    if (next !== sig) { sig = next; const keep = v != null ? v : s.value; fill(list); s.value = String(keep); }
    else if (v != null) s.set(v);
  };
  return s;
}

/** A number field with a unit. Sends on commit (Enter, blur). `.set(value)`. */
function number({ value, min, max, step, unit, digits, disabled, onChange }) {
  const i = h('input', { class: 'input num', type: 'number', min, max, step: step || 'any', disabled: !!disabled });
  const d = digits == null ? null : digits;
  const show = v => { if (v != null && document.activeElement !== i) i.value = d == null ? String(v) : Number(v).toFixed(d); };
  show(value);
  i.addEventListener('change', () => { const v = parseFloat(i.value); if (!isNaN(v) && onChange) onChange(v); });
  const el = h('span', { class: 'xnum' }, i, unit ? h('span', { class: 'faint', text: unit }) : null);
  el.set = show; el.input = i;
  return el;
}

/** A text field. Sends on commit. `.set(value)`. */
function text({ value, placeholder, disabled, onChange }) {
  const i = h('input', { class: 'input', type: 'text', placeholder: placeholder || '', disabled: !!disabled, maxlength: 500 });
  if (value != null) i.value = value;
  i.addEventListener('change', () => onChange && onChange(i.value));
  i.set = v => { if (document.activeElement !== i && v != null) i.value = v; };
  return i;
}

/** The slider card the Joint and Cart panels use: name, editable value, slider, and an
 *  optional "now" tick for a measured value. onChange fires on release / commit,
 *  onInput while dragging. `.set(value)`, `.setNow(value)`, `.setRange(min, max)`. */
function slider({ label, value, min, max, step, unit, digits, disabled, onChange, onInput }) {
  const d = digits == null ? 2 : digits;
  const num = h('input', { class: 'input num', type: 'number', step: step || 'any', min, max, disabled: !!disabled, 'aria-label': label });
  const rng = h('input', { type: 'range', min, max, step: step || 'any', disabled: !!disabled, 'aria-label': label + ' slider' });
  const now = h('i', { class: 'jr-now', 'aria-hidden': 'true', style: 'display:none' }), act = h('span', { class: 'num faint jr-act' });
  let lo = Number(min), hi = Number(max), held = false;
  const show = v => { if (v == null) return; if (document.activeElement !== num) num.value = Number(v).toFixed(d); if (!held) rng.value = v; };
  show(value);
  rng.addEventListener('pointerdown', () => { held = true; });
  rng.addEventListener('input', () => { num.value = Number(rng.value).toFixed(d); onInput && onInput(parseFloat(rng.value)); });
  rng.addEventListener('change', () => { held = false; onChange && onChange(parseFloat(rng.value)); });
  num.addEventListener('change', () => { const v = parseFloat(num.value); if (!isNaN(v)) { rng.value = v; onChange && onChange(v); } });
  const el = h('div', { class: 'jr kit-slider' },
    h('div', { class: 'jr-top' }, h('span', { class: 'jr-l', text: label }), act, h('span', { class: 'jr-v' }, num, h('span', { class: 'faint', text: unit || '' }))),
    h('div', { class: 'jr-track jr-free' }, rng, now));
  el.set = show;
  el.setNow = v => {
    now.style.display = v == null ? 'none' : '';
    setText(act, v == null ? '' : 'now ' + Number(v).toFixed(d) + (unit === '°' ? '°' : ''));
    if (v != null) now.style.setProperty('--p', Math.min(1, Math.max(0, (v - lo) / ((hi - lo) || 1))));
  };
  el.setRange = (a, b) => { lo = Number(a); hi = Number(b); rng.min = a; rng.max = b; num.min = a; num.max = b; };
  return el;
}

/** A progress bar 0–1 with a label. `.set(fraction, label?)`. */
function bar({ label, value }) {
  const fill = h('i'), lab = h('span', { class: 'kit-bar-l', text: label || '' });
  const el = h('div', { class: 'kit-bar' }, lab, h('div', { class: 'xbar' }, fill));
  el.set = (v, l) => { fill.style.width = (Math.max(0, Math.min(1, Number(v) || 0)) * 100) + '%'; if (l != null) setText(lab, l); };
  el.set(value);
  return el;
}

/** A scrolling log (newest last). `.set(lines)` replaces it; keeps the view at the bottom
 *  unless the reader has scrolled up. */
function log({ lines, height }) {
  const pre = h('pre', { class: 'xlog kit-log', style: height ? 'max-height:' + height + 'px' : null });
  pre.set = list => {
    const t = (list || []).join('\n');
    if (pre.textContent === t) return;
    const atEnd = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 4;
    pre.textContent = t;
    if (atEnd) pre.scrollTop = pre.scrollHeight;
  };
  pre.set(lines);
  return pre;
}

/** A one-line note: kind '' (faint), 'warn' or 'bad'. `.set(text, kind?)`. */
function note(textValue, kind) {
  const el = h('div', { class: 'kit-note' + (kind ? ' ' + kind : ''), text: textValue || '' });
  el.set = (t, k) => { setText(el, t || ''); el.hidden = !t; if (k !== undefined) el.className = 'kit-note' + (k ? ' ' + k : ''); };
  el.hidden = !textValue;
  return el;
}

/** A table: columns [{ key, label, num }]; `.set(rows)` with rows as objects. */
function table({ columns, rows, empty }) {
  const body = h('tbody');
  const el = h('table', { class: 'table kit-table' }, h('thead', null, h('tr', null, ...columns.map(c => h('th', { text: c.label, class: c.num ? 'num' : null })))), body);
  el.set = list => {
    body.textContent = '';
    if (!list || !list.length) { body.append(h('tr', null, h('td', { colspan: columns.length, class: 'faint', text: empty || 'Nothing yet' }))); return; }
    for (const r of list) body.append(h('tr', null, ...columns.map(c => h('td', { class: c.num ? 'num' : null, text: r[c.key] == null ? '' : String(r[c.key]) }))));
  };
  el.set(rows);
  return el;
}

window.FIC.ui = { section, grid, stats, field, button, buttons, toggle, select, number, text, slider, bar, log, note, table, h };
})();
