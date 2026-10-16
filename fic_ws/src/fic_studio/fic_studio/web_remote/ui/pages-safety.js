/* Safety page: the Cartesian zones, joint limits and velocity limits the
   controllers enforce, and where the robot is against them right now.
   Editing is two steps on purpose: change values, press Review, read the
   before / after list, press Apply. The Studio checks the numbers again and
   refuses anything unsafe or malformed (`safety_set` needs confirm: true). */
(() => {
'use strict';
const { $, h, fmt, setText, csection, csectionTools, pageTitle, cmd, toast, inlineConfirm } = window.FIC;
const STATE_LABEL = { ok: 'Inside', warn: 'Near the edge', violation: 'Outside', off: 'Not checked' };
const OVERALL = { ok: 'All inside', warn: 'Near a limit', violation: 'Limit exceeded', off: 'Monitor off' };
const DOT = { ok: 'ok', warn: 'warn', violation: 'bad', off: '' };

const section = (title, aside, ...b) => csection('safety/' + title, title, aside, ...b);
window.FIC.pages.safety = function buildSafety(root) {
  const draft = new Map();          // path -> new value; only what the operator changed
  let cur = null;                   // latest safety section
  const askHost = h('div', { id: 'saf-ask' });
  const stat = (id, k) => h('div', { class: 'stat' }, h('div', { class: 'k', text: k }), h('div', { class: 'v', id }), h('div', { class: 'n', id: id + '-n' }));
  const cartBody = h('tbody'), jointBody = h('tbody'), velBox = h('div', { class: 'kv' }), switches = h('div', { class: 'rows' });
  const review = h('button', { class: 'btn btn-primary', type: 'button', text: 'Review changes', onclick: askApply });
  const discard = h('button', { class: 'btn', type: 'button', text: 'Discard', onclick: () => { draft.clear(); dismissAsk(); paint(true); } });
  const bar = h('div', { class: 'btn-row sticky-actions', id: 'saf-bar', hidden: true }, review, discard, h('span', { class: 'faint', id: 'saf-n', style: 'font-size:var(--fs-m)' }));
  const err = h('div', { class: 'bad-line', id: 'saf-err' });
  root.append(
    ...pageTitle('Safety', 'Zones and limits the controllers enforce. Changes need a second, explicit confirmation.'),
    h('div', { class: 'stats' }, stat('sf-over', 'State'), stat('sf-mon', 'Monitor'), stat('sf-enf', 'On violation'), stat('sf-thr', 'Warning band')),
    bar, err, askHost, csectionTools(root),
    section('Cartesian zones', { id: 'sf-c-aside' }, h('table', { class: 'table safety' },
      h('thead', null, h('tr', null, ...['Axis', 'Min m', 'Max m', 'Checked', 'Position m', 'Where'].map(t => h('th', { text: t })))), cartBody)),
    section('Joint limits', { id: 'sf-j-aside' }, h('table', { class: 'table safety' },
      h('thead', null, h('tr', null, ...['Joint', 'Lower °', 'Upper °', 'Mechanical °', 'Position °', 'Where'].map(t => h('th', { text: t })))), jointBody),
      h('div', { style: 'margin-top:20px' }, toggle('joints.enabled', 'Check joint limits'))),
    section('Velocity limits', { id: 'sf-v-aside' }, velBox, h('div', { style: 'margin-top:20px' }, toggle('velocity.enabled', 'Limit velocity'))),
    section('Monitoring', null, switches));

  function dismissAsk() { askHost.textContent = ''; }
  function get(path, base) { return draft.has(path) ? draft.get(path) : base; }
  function mark() {
    $('#saf-bar').hidden = draft.size === 0;
    setText($('#saf-n'), draft.size + (draft.size === 1 ? ' value changed, not applied' : ' values changed, not applied'));
    for (const el of root.querySelectorAll('[data-k]')) el.closest('tr, .kvcell, label') && el.closest('tr, .kvcell, label').classList.toggle('edited', draft.has(el.dataset.k));
  }
  function num(path, step, lo, hi) {
    const el = h('input', { class: 'input num', type: 'number', step, 'data-k': path, 'aria-label': path.replace(/\./g, ' ') });
    if (lo != null) el.min = lo; if (hi != null) el.max = hi;
    el.addEventListener('input', () => { if (el.value === '') return; draft.set(path, parseFloat(el.value)); mark(); });
    return el;
  }
  function toggle(path, label) {
    const box = h('input', { type: 'checkbox', 'data-k': path });
    box.addEventListener('change', () => { draft.set(path, box.checked); mark(); });
    return h('label', { class: 'toggle' }, box, h('span', { class: 'track' }), h('span', { text: label }));
  }

  // ── structure, built once per shape ──
  let shape = '';
  function build(s) {
    cartBody.textContent = ''; jointBody.textContent = ''; velBox.textContent = ''; switches.textContent = '';
    for (const a of ['x', 'y', 'z']) {
      cartBody.append(h('tr', { 'data-row': 'c.' + a },
        h('td', { text: a.toUpperCase() }), h('td', null, num('c.' + a + '.min', '0.01', -s.bounds.cart_abs, s.bounds.cart_abs)),
        h('td', null, num('c.' + a + '.max', '0.01', -s.bounds.cart_abs, s.bounds.cart_abs)), h('td', null, toggle('c.' + a + '.enabled', '')),
        h('td', { class: 'live' }), h('td', { class: 'where' })));
    }
    s.joints.rows.forEach((r, i) => jointBody.append(h('tr', { 'data-row': 'j.' + i },
      h('td', { text: r.label }), h('td', null, num('j.' + i + '.lower', '0.5', r.mech_lower, r.mech_upper)), h('td', null, num('j.' + i + '.upper', '0.5', r.mech_lower, r.mech_upper)),
      h('td', { class: 'faint', text: r.mech_lower != null ? fmt(r.mech_lower, 0) + ' to ' + fmt(r.mech_upper, 0) : '—' }), h('td', { class: 'live' }), h('td', { class: 'where' }))));
    for (const [k, label, unit, max, min] of [['max_joint_vel', 'Maximum joint velocity', 'rad/s', s.bounds.joint_vel, 0.01], ['max_cart_vel', 'Maximum Cartesian velocity', 'm/s', s.bounds.cart_vel, 0.01],
        ['joint_target_speed', 'Joint target speed (joint impedance; 0 = off)', 'rad/s', s.bounds.joint_vel, 0]]) {
      velBox.append(h('div', { class: 'kvcell' }, h('div', { class: 'k', text: label + ' · ' + unit }), num('v.' + k, '0.05', min, max)));
    }
    switches.append(toggle('monitor', 'Monitor positions against the limits (warn and log)'), toggle('enforce', 'Stop the robot on a violation'));
    shape = s.joints.rows.length + ':' + s.bounds.cart_abs;
  }
  const where = (state, extra) => [h('span', { class: 'dot ' + DOT[state] }), STATE_LABEL[state] + (extra || '')];
  const setIfIdle = (el, v) => { if (document.activeElement !== el && el.value !== String(v)) el.value = v; };
  function paint(force) {
    const s = cur; if (!s) return;
    const at = (path, base) => get(path, base);
    for (const a of ['x', 'y', 'z']) {
      const c = s.cartesian[a], row = cartBody.querySelector('[data-row="c.' + a + '"]');
      setIfIdle(row.querySelector('[data-k="c.' + a + '.min"]'), at('c.' + a + '.min', c.min));
      setIfIdle(row.querySelector('[data-k="c.' + a + '.max"]'), at('c.' + a + '.max', c.max));
      row.querySelector('[data-k$=".enabled"]').checked = !!at('c.' + a + '.enabled', c.enabled);
      setText(row.querySelector('.live'), c.value == null ? '—' : fmt(c.value, 3));
      row.querySelector('.where').replaceChildren(...where(c.state));
    }
    s.joints.rows.forEach((r, i) => {
      const row = jointBody.querySelector('[data-row="j.' + i + '"]');
      setIfIdle(row.querySelector('[data-k="j.' + i + '.lower"]'), at('j.' + i + '.lower', r.lower == null ? '' : r.lower));
      setIfIdle(row.querySelector('[data-k="j.' + i + '.upper"]'), at('j.' + i + '.upper', r.upper == null ? '' : r.upper));
      setText(row.querySelector('.live'), r.value == null ? '—' : fmt(r.value, 1));
      row.querySelector('.where').replaceChildren(...where(r.state));
    });
    for (const k of ['max_joint_vel', 'max_cart_vel', 'joint_target_speed']) setIfIdle(velBox.querySelector('[data-k="v.' + k + '"]'), at('v.' + k, s.velocity[k]));
    const set = (path, v) => { const b = root.querySelector('[data-k="' + path + '"]'); if (b) b.checked = !!at(path, v); };
    set('joints.enabled', s.joints.enabled); set('velocity.enabled', s.velocity.enabled); set('monitor', s.enabled); set('enforce', s.enforce);
    mark();
  }

  // ── review, then apply ──
  function payload() {
    const s = cur, out = { confirm: true }, lines = [];
    const cart = {};
    for (const a of ['x', 'y', 'z']) {
      const c = s.cartesian[a];
      for (const f of ['min', 'max', 'enabled']) {
        const p = 'c.' + a + '.' + f;
        if (!draft.has(p) || draft.get(p) === c[f]) continue;
        (cart[a] = cart[a] || {})[f] = draft.get(p);
        const widen = f === 'min' ? draft.get(p) < c.min : f === 'max' ? draft.get(p) > c.max : draft.get(p) === false;
        lines.push((widen ? 'WIDER  ' : '       ') + a.toUpperCase() + ' ' + f + ': ' + (f === 'enabled' ? (c[f] ? 'checked' : 'off') + ' -> ' + (draft.get(p) ? 'checked' : 'off') : fmt(c[f], 3) + ' -> ' + fmt(draft.get(p), 3)));
      }
    }
    if (Object.keys(cart).length) out.cartesian = cart;
    const lo = s.joints.rows.map(r => r.lower), hi = s.joints.rows.map(r => r.upper);
    let jointEdit = false;
    s.joints.rows.forEach((r, i) => {
      for (const [f, arr] of [['lower', lo], ['upper', hi]]) {
        const p = 'j.' + i + '.' + f;
        if (draft.has(p) && draft.get(p) !== arr[i]) {
          const widen = f === 'lower' ? draft.get(p) < arr[i] : draft.get(p) > arr[i];
          lines.push((widen ? 'WIDER  ' : '       ') + r.label + ' ' + f + ': ' + fmt(arr[i], 1) + ' -> ' + fmt(draft.get(p), 1));
          arr[i] = draft.get(p); jointEdit = true;
        }
      }
    });
    const joints = {};
    if (jointEdit) { joints.lower = lo; joints.upper = hi; }
    if (draft.has('joints.enabled') && draft.get('joints.enabled') !== s.joints.enabled) {
      joints.enabled = draft.get('joints.enabled'); lines.push((joints.enabled ? '       ' : 'WIDER  ') + 'Joint limits: ' + (joints.enabled ? 'checked' : 'NOT checked'));
    }
    if (Object.keys(joints).length) out.joints = joints;
    const vel = {};
    for (const f of ['max_joint_vel', 'max_cart_vel', 'joint_target_speed']) {
      const p = 'v.' + f;
      if (draft.has(p) && draft.get(p) !== s.velocity[f]) { vel[f] = draft.get(p); lines.push((draft.get(p) > s.velocity[f] ? 'WIDER  ' : '       ') + f.replace(/_/g, ' ') + ': ' + fmt(s.velocity[f], 2) + ' -> ' + fmt(draft.get(p), 2)); }
    }
    if (draft.has('velocity.enabled') && draft.get('velocity.enabled') !== s.velocity.enabled) { vel.enabled = draft.get('velocity.enabled'); lines.push((vel.enabled ? '       ' : 'WIDER  ') + 'Velocity limit: ' + (vel.enabled ? 'on' : 'OFF')); }
    if (Object.keys(vel).length) out.velocity = vel;
    for (const [p, key, label] of [['monitor', 'monitor', 'Monitor'], ['enforce', 'enforce', 'Stop on violation']]) {
      const base = p === 'monitor' ? s.enabled : s.enforce;
      if (draft.has(p) && draft.get(p) !== base) { out[key] = draft.get(p); lines.push((draft.get(p) ? '       ' : 'WIDER  ') + label + ': ' + (base ? 'on' : 'off') + ' -> ' + (draft.get(p) ? 'on' : 'off')); }
    }
    return { out, lines };
  }
  function askApply() {
    setText($('#saf-err'), '');
    const { out, lines } = payload();
    if (!lines.length) { draft.clear(); mark(); toast('Nothing differs from the current limits'); return; }
    const wider = lines.some(l => l.startsWith('WIDER'));
    inlineConfirm(askHost, {
      title: wider ? 'Apply these changes? Some of them make the limits wider.' : 'Apply these changes to the safety limits?',
      lines: [...lines, 'The controllers start using them as soon as they are applied.'],
      label: 'Apply to the controllers', danger: wider,
      onConfirm: async () => {
        const r = await cmd('safety_set', out, true);
        if (r.ok) { draft.clear(); toast('Safety limits applied'); paint(true); }
        else setText($('#saf-err'), (r.message || 'The Studio refused the change') + '. Nothing was changed.');
      } });
  }

  return s => {
    const sf = s.safety; if (!sf) { setText($('#lede'), 'This Studio reports no safety state.'); return; }
    cur = sf;
    if (shape !== sf.joints.rows.length + ':' + sf.bounds.cart_abs) build(sf);
    setText($('#lede'), 'Limits are published to every robot’s controller; the Studio also checks positions at 10 Hz.');
    setText($('#sf-over'), OVERALL[sf.overall]); setText($('#sf-over-n'), sf.enabled ? 'monitor on' : 'positions are not checked');
    $('#sf-over').style.color = sf.overall === 'violation' ? 'var(--bad)' : sf.overall === 'warn' ? 'var(--warn)' : '';
    setText($('#sf-mon'), sf.enabled ? 'On' : 'Off'); setText($('#sf-mon-n'), 'warn and log');
    setText($('#sf-enf'), sf.enforce ? 'Stop' : 'Warn only'); setText($('#sf-enf-n'), sf.enforce ? 'graceful controller stop' : 'the robot is not stopped');
    setText($('#sf-thr'), fmt(sf.warning_threshold * 100, 0) + ' %'); setText($('#sf-thr-n'), 'of each range');
    setText($('#sf-c-aside'), Object.values(sf.cartesian).filter(c => c.enabled).length + ' of 3 checked');
    setText($('#sf-j-aside'), sf.joints.enabled ? sf.joints.rows.length + ' joints' : 'not checked');
    setText($('#sf-v-aside'), sf.velocity.enabled ? 'limited' : 'not limited');
    paint(false);
  };
};
})();
