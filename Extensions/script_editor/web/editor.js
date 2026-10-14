// Script Editor web page: the code editor. A <textarea> laid over a coloured
// <pre> holding the same text (the textarea's own text is transparent, so its
// caret and selection show over the colours), a line-number gutter, and the keys
// of a code editor. The pure functions are exported for test_editor_js.py;
// createEditor() needs a browser. QtWebEngine 5.15 (Chromium 87) must run it.

export const INDENT = '    ';
export const KEYWORDS = new Set(('False None True and as assert async await break class continue def del elif ' +
  'else except finally for from global if import in is lambda nonlocal not or pass raise return try while with ' +
  'yield').split(' '));
export const BUILTINS = new Set(('abs all any bool dict dir enumerate float getattr hasattr int isinstance len ' +
  'list map max min open print range repr reversed round set sorted str sum tuple type zip').split(' '));

const WORD = /^[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*/;     // dotted, so robot.motion.home is one token
const NUMBER = /^(?:0[xXoObB][0-9a-fA-F_]+|\d[\d_]*(?:\.\d*)?(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?)j?/;
const ESC = { '&': '&amp;', '<': '&lt;', '>': '&gt;' };
const esc = s => s.replace(/[&<>]/g, ch => ESC[ch]);

/** One line as [kind, text] tokens. `open` is the triple quote an earlier line
 *  left open ('' if none); the result's `open` is the one this line leaves open.
 *  Kinds: kw, builtin, api (robot.* and the plain names in the Set `api`), def
 *  (the name after def or class), str, com, num, txt. */
export function tokenize(line, open, api) {
  const tokens = [];
  const push = (kind, text) => {
    const last = tokens[tokens.length - 1];
    if (last && last[0] === kind) last[1] += text; else tokens.push([kind, text]);
  };
  let i = 0;
  if (open) {
    const end = line.indexOf(open);
    if (end < 0) { if (line) push('str', line); return { tokens, open }; }
    push('str', line.slice(0, end + 3));
    i = end + 3;
  }
  let afterDef = false;
  while (i < line.length) {
    const c = line[i];
    const rest = line.slice(i);
    if (c === '#') { push('com', rest); break; }
    const triple = rest.startsWith('"""') ? '"""' : rest.startsWith("'''") ? "'''" : '';
    if (triple) {
      const end = line.indexOf(triple, i + 3);
      if (end < 0) { push('str', rest); return { tokens, open: triple }; }
      push('str', line.slice(i, end + 3));
      i = end + 3;
      continue;
    }
    if (c === '"' || c === "'") {
      let j = i + 1;
      while (j < line.length && line[j] !== c) j += line[j] === '\\' ? 2 : 1;
      push('str', line.slice(i, j + 1));
      i = j + 1;
      continue;
    }
    const word = WORD.exec(rest);
    if (word) {
      const w = word[0];
      let kind = 'txt';
      if (afterDef) kind = 'def';
      else if (w === 'robot' || w.startsWith('robot.') || (api && api.has(w))) kind = 'api';
      else if (KEYWORDS.has(w)) kind = 'kw';
      else if (BUILTINS.has(w)) kind = 'builtin';
      push(kind, w);
      afterDef = w === 'def' || w === 'class';
      i += w.length;
      continue;
    }
    const num = NUMBER.exec(rest);
    if (num) { push('num', num[0]); i += num[0].length; continue; }
    if (c !== ' ' && c !== '\t') afterDef = false;
    push('txt', c);
    i += 1;
  }
  return { tokens, open: '' };
}

/** The text as HTML, a <span class="se-KIND"> per coloured token; line `mark`
 *  (1-based) is wrapped in <span class="se-markline">. */
export function highlight(text, api, mark) {
  let open = '';
  return text.split('\n').map((line, n) => {
    const r = tokenize(line, open, api);
    open = r.open;
    const html = r.tokens.map(([k, t]) => (k === 'txt' ? esc(t) : '<span class="se-' + k + '">' + esc(t) + '</span>')).join('');
    return n + 1 === mark ? '<span class="se-markline">' + (html || ' ') + '</span>' : html;
  }).join('\n');
}

/** Indent (dedent false) or dedent every line the selection [start, end) touches.
 *  Returns the new text and selection, and the replaced range [from, to) of the
 *  old text with its replacement `block`. */
export function shiftLines(text, start, end, dedent) {
  const from = text.lastIndexOf('\n', start - 1) + 1;
  const last = end > start && text[end - 1] === '\n' ? end - 1 : end;   // ending at a line start leaves that line
  let to = text.indexOf('\n', last);
  if (to < 0) to = text.length;
  const deltas = [];
  const block = text.slice(from, to).split('\n').map(l => {
    if (!dedent) { deltas.push(INDENT.length); return INDENT + l; }
    const cut = /^(?: {1,4}|\t)/.exec(l);
    const k = cut ? cut[0].length : 0;
    deltas.push(-k);
    return l.slice(k);
  }).join('\n');
  const total = deltas.reduce((a, b) => a + b, 0);
  const newStart = !dedent && start === from && end > start ? from : Math.max(from, start + deltas[0]);
  const newEnd = end === start ? newStart : Math.max(newStart, end + total);
  return { text: text.slice(0, from) + block + text.slice(to), start: newStart, end: newEnd, from, to, block };
}

/** The indent of a new line typed after `before` (its line up to the caret): the
 *  same as that line's, one level more after a ':' (a trailing comment ignored). */
export function newlineIndent(before) {
  const lead = /^[ \t]*/.exec(before)[0];
  return before.replace(/#.*$/, '').trimEnd().endsWith(':') ? lead + INDENT : lead;
}

/** The script line an output line points at (File "<script>", line N), or null. */
export function scriptLine(text) {
  const m = /File "<script>", line (\d+)/.exec(text || '');
  return m ? Number(m[1]) : null;
}

/** The editor. onChange(text) after each edit, onSave() on Ctrl+S, onRun() on
 *  Ctrl+Enter. `api` is the Set of plain names coloured like robot.* calls;
 *  call refresh() after changing it. Esc then Tab leaves the editor. */
export function createEditor({ api, onChange, onSave, onRun }) {
  const make = (tag, cls) => { const n = document.createElement(tag); n.className = cls; return n; };
  const el = make('div', 'se-editor');
  const gutter = make('div', 'se-gutter');
  const code = make('div', 'se-code');
  const hl = make('pre', 'se-hl');
  const input = make('textarea', 'se-input');
  gutter.setAttribute('aria-hidden', 'true');
  hl.setAttribute('aria-hidden', 'true');
  input.setAttribute('aria-label', 'Script');
  input.setAttribute('wrap', 'off');
  input.setAttribute('autocapitalize', 'off');
  input.setAttribute('autocomplete', 'off');
  input.spellcheck = false;
  code.append(hl, input);
  el.append(gutter, code);

  let marked = null;
  let frame = 0;
  let escaped = false;
  function paint() {
    frame = 0;
    const text = input.value;
    hl.innerHTML = highlight(text, api, marked) + '\n';       // the extra line keeps a last empty line's height
    const n = text.split('\n').length;
    let rows = '';
    for (let k = 1; k <= n; k += 1) rows += k === marked ? '<div class="se-mark">' + k + '</div>' : '<div>' + k + '</div>';
    gutter.innerHTML = rows;
  }
  const schedule = () => { if (!frame) frame = requestAnimationFrame(paint); };
  function changed() {
    marked = null;
    schedule();
    if (onChange) onChange(input.value);
  }
  /** Replace [from, to) with `text` through the browser's editing, so Ctrl+Z undoes it. */
  function replace(from, to, text, selStart, selEnd) {
    input.focus();
    input.setSelectionRange(from, to);
    if (!document.execCommand('insertText', false, text)) { input.setRangeText(text, from, to, 'end'); changed(); }
    if (selStart != null) input.setSelectionRange(selStart, selEnd);
  }

  input.addEventListener('input', changed);
  input.addEventListener('keydown', e => {
    const mod = e.ctrlKey || e.metaKey;
    if (e.key === 'Escape') { escaped = true; return; }
    if (e.key === 'Tab' && !mod && !e.altKey) {
      if (escaped) { escaped = false; return; }               // let focus move on
      e.preventDefault();
      const s = input.selectionStart, t = input.selectionEnd, value = input.value;
      if (e.shiftKey || value.slice(s, t).indexOf('\n') >= 0) {
        const r = shiftLines(value, s, t, e.shiftKey);
        if (r.block !== value.slice(r.from, r.to)) replace(r.from, r.to, r.block, r.start, r.end);
      } else {
        replace(s, t, INDENT);
      }
    } else if (e.key === 'Enter' && !mod && !e.shiftKey && !e.altKey) {
      e.preventDefault();
      const s = input.selectionStart;
      replace(s, input.selectionEnd, '\n' + newlineIndent(input.value.slice(input.value.lastIndexOf('\n', s - 1) + 1, s)));
    } else if (mod && (e.key === 's' || e.key === 'S')) {
      e.preventDefault();
      if (onSave) onSave();
    } else if (mod && e.key === 'Enter') {
      e.preventDefault();
      if (onRun) onRun();
    }
    escaped = false;
  });
  schedule();

  return {
    el,
    getValue: () => input.value,
    /** New text (a script or an example): the undo history starts again. */
    setValue(text) {
      input.value = text;
      marked = null;
      input.setSelectionRange(0, 0);
      el.scrollTop = 0;
      schedule();
    },
    /** Insert at the caret (an API call from the drawer). */
    insert(text) { replace(input.selectionStart, input.selectionEnd, text); },
    /** Mark line n (1-based; null clears it) and bring it into view. */
    mark(n) {
      marked = n || null;
      schedule();
      if (!marked) return;
      const lines = input.value.split('\n');
      let at = 0;
      for (let k = 0; k < Math.min(marked - 1, lines.length); k += 1) at += lines[k].length + 1;
      input.focus();
      input.setSelectionRange(at, at);
      const lh = parseFloat(getComputedStyle(input).lineHeight) || 20;
      el.scrollTop = Math.max(0, (marked - 3) * lh);
    },
    refresh: schedule,
    focus: () => input.focus(),
  };
}
