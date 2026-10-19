/* Monochrome stroke icons (1.5 px, currentColor, sized to the text) in place of
   symbol glyphs. System emoji fonts draw the media symbols (play, stop, record,
   pause) and friends in colour;
   no glyph in this UI is left to chance. Our own markup calls FIC.icon('play');
   text that arrives from elsewhere (the Studio's Qt labels through the extension
   mirror, status strings) is cleaned by a MutationObserver that swaps a leading
   glyph for the same icon. */
(() => {
'use strict';
const NS = 'http://www.w3.org/2000/svg';
const P = {
  play: 'M8 5.5v13l10.5-6.5z', stop: 'M6.5 6.5h11v11h-11z', record: 'M12 5.5a6.5 6.5 0 1 0 0 13a6.5 6.5 0 0 0 0-13zM12 9.7a2.3 2.3 0 1 0 0 4.6a2.3 2.3 0 0 0 0-4.6z',
  dot: 'M12 8a4 4 0 1 0 0 8a4 4 0 0 0 0-8z', ring: 'M12 8a4 4 0 1 0 0 8a4 4 0 0 0 0-8z',
  pause: 'M8.5 6v12M15.5 6v12', refresh: 'M19 12a7 7 0 1 1-2.2-5.1M19 4.5v4h-4', home: 'M4 11.5L12 4.5l8 7M6.5 10v9.5h11V10',
  gripper: 'M8 4.5v7M16 4.5v7M6 11.5h12l-1.8 7.5H7.8z', plot: 'M4.5 4.5v15h15M8 15l3.5-4.5 3 3L19 7.5',
  eye: 'M2.5 12S6 5.5 12 5.5S21.5 12 21.5 12S18 18.5 12 18.5S2.5 12 2.5 12zM12 9.5a2.5 2.5 0 1 0 0 5a2.5 2.5 0 0 0 0-5z',
  arrow: 'M5 12h14M13.5 6.5L19 12l-5.5 5.5', back: 'M19 12H5M10.5 6.5L5 12l5.5 5.5', up: 'M12 19V5M6.5 10.5L12 5l5.5 5.5',
  'chevron-up': 'M6 14.5l6-6l6 6', 'chevron-down': 'M6 9.5l6 6l6-6', 'chevron-right': 'M9.5 6l6 6l-6 6', diamond: 'M12 4.5l7.5 7.5L12 19.5L4.5 12z',
  check: 'M5 12.5l4.5 4.5L19 7.5', x: 'M6.5 6.5l11 11M17.5 6.5l-11 11',
  warn: 'M12 4.5l9 15.5H3zM12 10v4.2M12 16.8v.2', camera: 'M4 8h3.5l1.5-2.5h6L16.5 8H20v11H4zM12 10.5a3 3 0 1 0 0 6a3 3 0 0 0 0-6z',
  cursor: 'M6.5 4.5l11 6.5l-5 1.4l-2.6 4.8z', cube: 'M12 3.5l7.5 4v9l-7.5 4l-7.5-4v-9zM4.5 7.5l7.5 4l7.5-4M12 11.5v9',
  sphere: 'M12 4.5a7.5 7.5 0 1 0 0 15a7.5 7.5 0 0 0 0-15zM4.5 12c0 1.8 3.4 3.2 7.5 3.2s7.5-1.4 7.5-3.2',
  cylinder: 'M5.5 6.5c0-1.4 2.9-2.5 6.5-2.5s6.5 1.1 6.5 2.5s-2.9 2.5-6.5 2.5s-6.5-1.1-6.5-2.5zM5.5 6.5v11c0 1.4 2.9 2.5 6.5 2.5s6.5-1.1 6.5-2.5v-11',
  pen: 'M4.5 19.5l1-4l10-10l3 3l-10 10zM13.5 7.5l3 3', table: 'M3.5 15.5h17M6 15.5v4M18 15.5v4M12 4.5v7.5M9.5 9.5l2.5 2.5l2.5-2.5',
  wall: 'M18.5 3.5v17M4.5 12h10M11.5 9l3 3l-3 3', target: 'M12 3.5v4M12 16.5v4M3.5 12h4M16.5 12h4M12 9a3 3 0 1 0 0 6a3 3 0 0 0 0-6z',
  trash: 'M5 7h14M9.5 7V5h5v2M7 7l1 12.5h8L17 7', plus: 'M12 5v14M5 12h14',
};
const FILLED = new Set(['dot']);
function icon(name, cls) {
  const s = document.createElementNS(NS, 'svg');
  s.setAttribute('viewBox', '0 0 24 24'); s.setAttribute('aria-hidden', 'true'); s.setAttribute('focusable', 'false');
  s.setAttribute('class', 'ico ico-' + name + (FILLED.has(name) ? ' fill' : '') + (cls ? ' ' + cls : ''));
  const p = document.createElementNS(NS, 'path'); p.setAttribute('d', P[name] || P.dot); s.append(p);
  return s;
}
const MAP = {
  '▶': 'play', '►': 'play', '⏵': 'play', '■': 'stop', '\u23F9': 'stop', '◼': 'stop', '▪': 'stop', '\u23FA': 'record', '◉': 'record',
  '\u23F8': 'pause', '↻': 'refresh', '⟳': 'refresh', '⌂': 'home', '▲': 'chevron-up', '▼': 'chevron-down', '▾': 'chevron-down', '▸': 'chevron-right', '▴': 'chevron-up',
  '◀': 'back', '←': 'back', '⬆': 'up', '↑': 'up', '◇': 'diamond', '◆': 'diamond', '●': 'dot', '○': 'ring', '✓': 'check', '\u2714': 'check',
  '✕': 'x', '\u2716': 'x', '✗': 'x', '\u26A0': 'warn', '\u2699': 'dot',
};
const LEAD = new RegExp('^(\\s*)([' + Object.keys(MAP).join('') + '])[\\uFE0E\\uFE0F]?\\s*', 'u');
const SKIP = new Set(['SCRIPT', 'STYLE', 'TEXTAREA', 'OPTION', 'TITLE', 'PRE', 'CODE', 'svg']);
const unVS = s => String(s == null ? '' : s).replace(/[\uFE0E\uFE0F]/g, '');

/* "▶ START FOLLOWING" -> { glyph: '▶', text: 'Start following' } : strip a leading icon glyph and
   turn shouty extension labels into sentence case. Words with digits or short acronyms stay. */
function tidyLabel(label) {
  let t = unVS(label), glyph = '';
  const m = LEAD.exec(t);
  if (m) { glyph = m[2]; t = t.slice(m[0].length); }
  t = t.replace(/\s*[\u2192\u279c\u2794]\s*$/, '');
  t = t.replace(/\p{Extended_Pictographic}\uFE0F?\s*/gu, '').trim();     // any other emoji: no coloured glyphs or empty boxes
  const letters = t.replace(/[^A-Za-z]/g, '');
  if (letters.length > 3 && letters === letters.toUpperCase())
    t = t.toLowerCase().replace(/^\s*([a-z])/, c => c.toUpperCase());
  return { glyph, name: MAP[glyph] || null, text: t };
}

function convert(text) {
  const m = LEAD.exec(text.data);
  if (!m) return;
  const parent = text.parentNode;
  if (!parent || SKIP.has(parent.nodeName)) { if (parent && parent.nodeName === 'OPTION') text.data = text.data.replace(LEAD, '$1'); return; }
  const raw = text.data, rest = text.data.slice(m[0].length);
  const svg = icon(MAP[m[2]]);
  if (m[1]) parent.insertBefore(document.createTextNode(m[1]), text);
  parent.insertBefore(svg, text);
  if (rest) text.data = rest; else text.remove();
  if (parent.childNodes.length === (rest ? 2 : 1)) parent.__raw = raw;     // so setText() does not rewrite an unchanged label
}
function walk(root) {
  if (root.nodeType === 3) { convert(root); return; }
  if (root.nodeType !== 1 || SKIP.has(root.nodeName)) return;
  const w = document.createTreeWalker(root, NodeFilter.SHOW_TEXT), found = [];
  for (let n = w.nextNode(); n; n = w.nextNode()) if (LEAD.test(n.data)) found.push(n);
  found.forEach(convert);
}
function start() {
  walk(document.body);
  new MutationObserver(recs => {
    for (const r of recs) {
      if (r.type === 'characterData') convert(r.target);
      else for (const n of r.addedNodes) walk(n);
    }
  }).observe(document.body, { childList: true, subtree: true, characterData: true });
}
if (window.FIC) Object.assign(window.FIC, { icon, tidyLabel });
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start); else start();
})();
