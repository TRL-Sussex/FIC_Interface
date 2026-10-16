/* Themed dropdown lists. Clicking a <select> (or Space, Enter, Alt+Down on it) opens a list drawn in the
   Studio's theme instead of the browser's native one; the <select> stays the value holder, and picking an
   option sets it and fires its 'input' and 'change' events, so every page works as before. Multi-line
   selects (multiple, size > 1) and disabled ones are left alone. */
(function () {
  'use strict';
  let pop = null, owner = null, items = [], hl = -1;

  const usable = el => el && el.tagName === 'SELECT' && !el.multiple && !(el.size > 1) && !el.disabled;

  function close(refocus) {
    if (!pop) return;
    pop.remove();
    pop = null;
    if (refocus && owner) owner.focus();
    owner = null; items = []; hl = -1;
    window.removeEventListener('resize', onAway, true);
    window.removeEventListener('blur', onAway, true);
    document.removeEventListener('scroll', onScroll, true);
  }
  const onAway = () => close(false);
  const onScroll = e => { if (pop && !pop.contains(e.target)) close(false); };

  function highlight(i) {
    if (!items.length) return;
    hl = Math.max(0, Math.min(items.length - 1, i));
    items.forEach((it, k) => it.el.classList.toggle('hl', k === hl));
    items[hl].el.scrollIntoView({ block: 'nearest' });
  }

  function pick(i) {
    const it = items[i];
    if (!it || it.opt.disabled) return;
    const sel = owner;
    close(true);
    if (sel.selectedIndex !== it.index) {
      sel.selectedIndex = it.index;
      sel.dispatchEvent(new Event('input', { bubbles: true }));
      sel.dispatchEvent(new Event('change', { bubbles: true }));
    }
  }

  function addItem(opt, index) {
    const el = document.createElement('div');
    el.className = 'dd-item' + (opt.disabled ? ' dis' : '') + (index === owner.selectedIndex ? ' sel' : '');
    el.textContent = opt.textContent;
    if (opt.title) el.title = opt.title;
    const k = items.length;
    el.addEventListener('mousemove', () => { if (hl !== k && !opt.disabled) highlight(k); });
    el.addEventListener('mousedown', e => e.preventDefault());
    el.addEventListener('click', () => pick(k));
    items.push({ el, opt, index });
    return el;
  }

  function open(sel) {
    close(false);
    owner = sel;
    pop = document.createElement('div');
    pop.className = 'dd-pop';
    pop.setAttribute('role', 'listbox');
    let index = 0;
    for (const child of sel.children) {
      if (child.tagName === 'OPTGROUP') {
        const g = document.createElement('div');
        g.className = 'dd-group';
        g.textContent = child.label;
        pop.append(g);
        for (const opt of child.children) pop.append(addItem(opt, index++));
      } else if (child.tagName === 'OPTION') {
        if (!child.hidden) pop.append(addItem(child, index));
        index++;
      }
    }
    document.body.append(pop);
    const r = sel.getBoundingClientRect(), vh = window.innerHeight, vw = window.innerWidth;
    pop.style.minWidth = Math.max(r.width, 120) + 'px';
    const below = vh - r.bottom - 8, above = r.top - 8;
    const h = Math.min(pop.scrollHeight, 320);
    const up = below < Math.min(h, 160) && above > below;
    pop.style.maxHeight = Math.max(120, Math.min(320, up ? above : below)) + 'px';
    pop.style.top = (up ? Math.max(8, r.top - Math.min(h, above) - 4) : r.bottom + 4) + 'px';
    pop.style.left = Math.max(8, Math.min(r.left, vw - pop.offsetWidth - 8)) + 'px';
    const cur = items.findIndex(it => it.index === sel.selectedIndex);
    highlight(cur >= 0 ? cur : 0);
    window.addEventListener('resize', onAway, true);
    window.addEventListener('blur', onAway, true);
    document.addEventListener('scroll', onScroll, true);
  }

  document.addEventListener('mousedown', e => {
    if (pop && !pop.contains(e.target) && e.target !== owner) close(false);
    const sel = e.target.closest ? e.target.closest('select') : null;
    if (e.button !== 0 || !usable(sel)) return;
    e.preventDefault();
    sel.focus();
    if (pop && owner === sel) close(true); else open(sel);
  }, true);

  document.addEventListener('keydown', e => {
    if (pop) {
      const step = { ArrowDown: 1, ArrowUp: -1, PageDown: 8, PageUp: -8 }[e.key];
      if (step) { e.preventDefault(); let k = hl + step;
        while (items[k] && items[k].opt.disabled) k += Math.sign(step);
        if (items[k]) highlight(k); return; }
      if (e.key === 'Home' || e.key === 'End') { e.preventDefault(); highlight(e.key === 'Home' ? 0 : items.length - 1); return; }
      if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); pick(hl); return; }
      if (e.key === 'Escape' || e.key === 'Tab') { if (e.key === 'Escape') e.preventDefault(); close(e.key === 'Escape'); return; }
      return;
    }
    const sel = document.activeElement;
    if (usable(sel) && (e.key === ' ' || e.key === 'Enter' || (e.altKey && e.key === 'ArrowDown'))) {
      e.preventDefault();
      open(sel);
    }
  }, true);

  window.FIC = window.FIC || {};
  window.FIC.dropdown = { open, close: () => close(false) };
})();
