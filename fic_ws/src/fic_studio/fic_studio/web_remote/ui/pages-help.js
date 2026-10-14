/* Help: the classic Help page's sections (Getting Started, Window Layout,
   Pages, Control Modes, Extensions, Gamepad, Keyboard Shortcuts, Scripting,
   Processes, Safety, Troubleshooting, Glossary, About) with search. The Studio
   supplies each section as HTML; it is shown in a sandboxed frame, painted with
   this page's colours. The documentation website opens from the button. */
(() => {
'use strict';
const { $, h, setText, pageTitle, cmd, toast } = window.FIC;

window.FIC.pages.help = function buildHelp(root) {
  const st = { sections: [], key: null };
  const nav = h('div', { class: 'help-nav' }), frame = h('iframe', { class: 'help-frame', sandbox: '', title: 'Help section' });
  const search = h('input', { class: 'input', type: 'search', placeholder: 'Search help', 'aria-label': 'Search help' });
  root.append(...pageTitle('Help', 'Pages, control modes, shortcuts and troubleshooting.'),
    h('div', { class: 'toolbar' }, h('div', { class: 'field' }, h('label', { text: 'Search' }), search), h('div', { class: 'spacer' }),
      h('button', { class: 'btn btn-primary', type: 'button', text: 'Open the documentation', onclick: async () => { const r = await cmd('help', { op: 'docs', page: 'home' }); toast(r.opened ? 'Opened in the browser on the Studio machine' : 'The documentation was not found', !r.opened); } })),
    h('div', { class: 'help-grid' }, nav, frame));
  const wanted = () => new URLSearchParams((location.hash.split('?')[1] || '')).get('s');
  search.addEventListener('input', async () => { const r = await cmd('help', { op: 'search', text: search.value }, true); if (r.ok) draw(r.keys); });
  function draw(keys) {
    nav.textContent = '';
    for (const s of st.sections.filter(x => !keys || keys.includes(x.key))) nav.append(h('button', { class: 'help-link', type: 'button', 'aria-current': String(s.key === st.key), text: s.title, onclick: () => open(s.key) }));
  }
  const css = () => {
    const c = getComputedStyle(document.documentElement), v = n => c.getPropertyValue(n).trim();
    return 'body{margin:16px 4px;font:13px/1.6 ' + v('--sans') + ';color:' + v('--ink') + ';background:' + v('--canvas') + '}h2{font:600 22px/1.2 ' + v('--sans') + ';color:' + v('--ink-strong') + ';margin:0 0 8px}h3{font-size:15px;color:' + v('--ink-strong') + ';margin:22px 0 6px}p{margin:6px 0}ul,ol{margin:6px 0 8px 22px}li{margin:3px 0}table{border-collapse:collapse;margin:8px 0}td,th{padding:5px 18px 5px 0;vertical-align:top;border-bottom:1px solid ' + v('--rule') + ';text-align:left}code{font:12px ' + v('--mono') + ';color:' + v('--accent') + '}b,strong{color:' + v('--ink-strong') + '}em{color:' + v('--ink-muted') + '}a{color:' + v('--accent') + '}';
  };
  async function open(key) {
    const r = await cmd('help', { op: 'section', key }, true);
    if (!r.ok) return;
    st.key = key; frame.srcdoc = '<!doctype html><meta charset="utf-8"><style>' + css() + '</style>' + r.html; draw(null);
  }
  (async () => {
    const r = await cmd('help', { op: 'list' }, true);
    if (!r.ok) return;
    st.sections = r.sections; draw(null); open(wanted() && r.sections.find(s => s.key === wanted()) ? wanted() : r.sections[0].key);
  })();
  const onHash = () => { if (wanted() && st.sections.find(s => s.key === wanted())) open(wanted()); };
  window.addEventListener('hashchange', onHash);
  return { update() {}, dispose() { window.removeEventListener('hashchange', onHash); } };
};
})();
