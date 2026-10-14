/* Locomotion page: gait commands, a hold-to-move velocity command and live gait
   tuning for legged robots. The velocity is a hold: the page renews it every
   200 ms while a control is away from zero, and the Studio zeroes it after
   0.6 s without a renewal, so a closed tab or a dropped link cannot leave a
   robot walking. Everything is a `locomotion` command; nothing is computed here. */
(() => {
'use strict';
const { $, h, fmt, setText, section, pageTitle, cmd, toast } = window.FIC;

window.FIC.pages.locomotion = function buildLoco(root) {
  const vel = {}; let timer = null, built = '';
  const gait = h('div', { class: 'btn-row' }), pad = h('div', { class: 'rows' }), tune = h('div', { class: 'fgrid' });
  const note = h('p', { class: 'faint', style: 'font-size:var(--fs-m);margin-top:16px' });
  root.append(...pageTitle('Locomotion', 'Gait and velocity for legged robots.'),
    section('Gait', { id: 'lo-aside' }, gait, note),
    section('Velocity command', { id: 'lo-v' }, pad,
      h('div', { class: 'btn-row', style: 'margin-top:24px' }, h('button', { class: 'btn btn-danger', type: 'button', text: 'Stop walking', onclick: zero }),
        h('span', { class: 'faint', style: 'font-size:var(--fs-m)', text: 'Release a control and it returns to zero. Hold to keep moving.' }))),
    section('Gait tuning', null, tune));

  const send = () => cmd('locomotion', Object.assign({ op: 'velocity' }, vel), true);
  function tick() {
    if (!Object.values(vel).some(v => v)) { clearInterval(timer); timer = null; return; }
    send();
  }
  function zero() {
    for (const k of Object.keys(vel)) { vel[k] = 0; const r = pad.querySelector('[data-ax="' + k + '"]'); if (r) { r.value = 0; setText(pad.querySelector('[data-v="' + k + '"]'), '0.00'); } }
    clearInterval(timer); timer = null;
    cmd('locomotion', { op: 'stop' }, true);
  }
  const onHide = () => { if (document.hidden) zero(); };
  document.addEventListener('visibilitychange', onHide);
  window.addEventListener('blur', zero);

  function build(lo) {
    gait.textContent = ''; pad.textContent = ''; tune.textContent = '';
    for (const g of lo.gaits) gait.append(h('button', { class: 'btn btn-primary', type: 'button', text: g[0].toUpperCase() + g.slice(1), onclick: async () => { const r = await cmd('locomotion', { op: 'gait', command: g }); if (r.ok) toast('Gait: ' + g); } }));
    for (const a of lo.axes) {
      vel[a.key] = 0;
      const r = h('input', { type: 'range', min: -100, max: 100, step: 1, value: 0, 'data-ax': a.key, 'aria-label': a.label });
      r.addEventListener('input', () => { vel[a.key] = parseFloat(r.value) / 100 * a.limit; setText(pad.querySelector('[data-v="' + a.key + '"]'), fmt(vel[a.key], 2)); if (!timer) timer = setInterval(tick, 200); send(); });
      for (const ev of ['pointerup', 'pointercancel', 'keyup']) r.addEventListener(ev, () => { r.value = 0; vel[a.key] = 0; setText(pad.querySelector('[data-v="' + a.key + '"]'), '0.00'); send(); });
      pad.append(h('div', { class: 'jrow3' }, h('span', { text: a.label }), r, h('span', { class: 'num', 'data-v': a.key, text: '0.00' })));
    }
    for (const t of lo.tuning) {
      const el = h('input', { class: 'input num', type: 'number', min: t.min, max: t.max, step: t.step, value: t.default, 'aria-label': t.label });
      el.addEventListener('change', async () => { const r = await cmd('locomotion', { op: 'set', key: t.key, value: parseFloat(el.value) }); if (r.ok) toast(t.label + ' set to ' + el.value); });
      tune.append(h('div', { class: 'frow' }, h('label', { text: t.label }), el, h('span', { class: 'unit faint', text: t.min + ' to ' + t.max })));
    }
    built = 'yes';
  }
  return {
    update(s) {
      const lo = s.locomotion || {};
      setText($('#lede'), lo.available ? 'Legged robot · ' + (lo.running ? 'locomotion node running' : 'locomotion node stopped') : 'The selected robot is not a legged robot.');
      if (!lo.available) { if (built !== 'no') { gait.textContent = ''; pad.textContent = ''; tune.textContent = ''; note.textContent = 'Pick a legged robot (for example Unitree Go2) in the top bar to use these controls.'; built = 'no'; } return; }
      if (built !== 'yes') { build(lo); note.textContent = lo.running ? '' : 'Start the session to run the locomotion node; commands are sent either way.'; }
      setText($('#lo-aside'), lo.running ? 'running' : 'stopped');
      setText($('#lo-v'), 'hold ' + fmt(lo.hold_s, 1) + ' s');
    },
    dispose() { zero(); document.removeEventListener('visibilitychange', onHide); window.removeEventListener('blur', zero); },
  };
};
})();
