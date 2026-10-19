/* Section flow, opt-in: a container with the class flow-auto (Settings' body) lets its sections flow into
   columns as wide as the window allows (fic.css .sec-flow: columns of at least 34rem, so one column on a
   narrow window, two or three on a wide one). A child that is not a section, or holds wide content, spans
   every column. Other pages keep their own layout. Re-applied whenever a page adds or removes content. */
(function () {
  'use strict';
  const WIDE = 'table, canvas, iframe, .live-3d, .plots, .stage, .xtable, .logview, .two, .tm-grid, .rc-body, .help-grid';

  function apply(root) {
    for (const p of root.querySelectorAll('.flow-auto')) {
      const flow = [...p.children].filter(c => c.classList.contains('section')).length >= 2;
      p.classList.toggle('sec-flow', flow);
      if (!flow) continue;
      for (const c of p.children) c.classList.toggle('flow-span', !c.classList.contains('section') || !!c.querySelector(WIDE));
    }
  }

  let pending = false;
  function schedule(root) {
    if (pending) return;
    pending = true;
    requestAnimationFrame(() => { pending = false; apply(root); });
  }

  function start() {
    const main = document.getElementById('main');
    if (!main) return;
    new MutationObserver(() => schedule(main)).observe(main, { childList: true, subtree: true });
    schedule(main);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start); else start();
})();
