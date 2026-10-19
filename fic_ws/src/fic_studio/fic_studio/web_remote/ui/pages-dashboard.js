/* Dashboard: the classic monitoring page, Overview and Detailed tabs.
   Base (mode, target, config, telemetry), Joints, End effector
   with the tool wrench, Safety; and, in Detailed, limits and headroom, the
   whole pose, the robot and the telemetry channels. Read-only (the base processes start and stop one at a
   time in Settings → Processes). Above the tabs, the live view
   (pages-dashlive.js): the robot in 3D with path overlays beside live plots. A channel with no live source
   shows a dash; nothing is made up. */
(() => {
'use strict';
const { $, h, fmt, setText, setSeg, section, pageTitle, cmd, toast } = window.FIC;
const TARGET = { simulation: 'simulation', hardware: 'hardware', vr: 'vr' };
const EE_ROWS = ['commanded', 'pose', 'error', 'velocity'];
const EE_COLS = ['x', 'y', 'z', 'ox', 'oy', 'oz'];
const SAFE_DOT = { ok: 'ok', warn: 'warn', violation: 'bad', unknown: '' };
const SAFE_TEXT = { ok: 'ok', warn: 'near limit', violation: 'past limit', unknown: 'no data' };
const JOINT_COLS = ['commanded', 'position', 'error', 'velocity', 'torque'];
const JOINT_DETAIL = JOINT_COLS.concat(['τ max', '% τ', 'limit lo', 'limit hi', '% range']);

window.FIC.pages.dashboard = function buildDashboard(root) {
  let tab = window.FIC.store.get('fic-dash-tab') === 'detailed' ? 'detailed' : 'overview';
  const tabs = h('div', { class: 'seg', role: 'tablist', 'aria-label': 'Dashboard view' });
  const head = h('div', { class: 'dash-head' },
    h('span', { class: 'pill', id: 'dh-pill' }, h('span', { class: 'dot', id: 'dh-dot' }), h('span', { id: 'dh-state', text: 'idle' })),
    h('span', { class: 'faint', id: 'dh-info', text: '—' }), h('span', { class: 'spacer' }),
    h('span', { class: 'mods', id: 'dh-mods' }), h('span', { class: 'num', id: 'dh-health' }));
  const overview = h('div', { id: 'dash-over' }), detailed = h('div', { id: 'dash-det' });
  const liveBox = h('div', { id: 'dash-live' });
  const live = window.FIC.dashLive(liveBox);
  root.append(...pageTitle('Dashboard'), head, liveBox, h('div', { style: 'margin-top:var(--s6)' }, tabs), overview, detailed);

  // ── Overview ──
  const baseRows = {};
  const base = h('div', { class: 'kvl' });
  for (const k of ['control mode', 'target', 'profile', 'telemetry']) { const v = h('span', { class: 'num' }); baseRows[k] = v; base.append(h('div', null, h('span', { class: 'k', text: k }), v)); }
  const jbody = h('tbody'), ebody = h('tbody');
  const wrenchHead = h('div', { class: 'wrench-head' }, h('span', { class: 'eyebrow', text: 'tool wrench' }), h('span', { class: 'spacer' }),
    h('span', { class: 'num', id: 'w-f', text: '|F| — N' }), h('span', { class: 'num', id: 'w-t', text: '|τ| — Nm' }), h('span', { class: 'num', id: 'w-c', text: '○ free' }));
  const wbody = h('tbody');
  const safety = h('div', { class: 'safe-row' });
  const safeEls = {};
  for (const [k, label] of [['joints', 'Joint limits'], ['velocity', 'Velocity limits'], ['torque', 'Force / torque']]) { const d = h('span', { class: 'dot' }), t = h('span', { text: label }), st = h('span', { class: 'faint sr-state', text: 'no data' }); safeEls[k] = { d, st }; safety.append(h('span', { class: 'sr-item' }, d, t, st)); }
  overview.append(
    section('Base', null, base),
    section('Joints', { id: 'jt-aside' }, tableOf(JOINT_COLS, jbody)),
    section('End effector', { id: 'ee-aside' }, tableOf(EE_COLS, ebody), wrenchHead, tableOf(['x', 'y', 'z'], wbody)),
    section('Safety', null, safety));

  // ── Detailed ──
  const jdbody = h('tbody'), edbody = h('tbody'), wdbody = h('tbody');
  const robotKv = kv(['model', 'joints', 'gripper', 'ee frame', 'elbow frame', 'torque sign', 'gravity comp', 'wrench source', 'urdf']);
  const chanKv = kv(['joint states', 'joint velocity', 'joint effort', 'end-effector pose', 'tool wrench', 'deadbands']);
  detailed.append(
    section('Joints · limits and headroom', null, h('div', { class: 'hscroll' }, tableOf(JOINT_DETAIL, jdbody))),
    section('End effector · pose, twist and wrench', null, tableOf(EE_COLS, edbody), h('div', { style: 'height:var(--s5)' }), tableOf(['x', 'y', 'z', 'magnitude'], wdbody)),
    h('div', { class: 'two' }, section('Robot', null, robotKv.box), section('Telemetry channels', null, chanKv.box)));

  setSeg(tabs, [{ value: 'overview', label: 'Overview' }, { value: 'detailed', label: 'Detailed' }], tab, v => { tab = v; window.FIC.store.set('fic-dash-tab', v); showTab(); update(window.FIC.state()); });
  function showTab() { overview.hidden = tab !== 'overview'; detailed.hidden = tab !== 'detailed'; setSeg(tabs, [{ value: 'overview', label: 'Overview' }, { value: 'detailed', label: 'Detailed' }], tab, () => {}); }
  showTab();

  function tableOf(cols, body) { return h('table', { class: 'table' }, h('thead', null, h('tr', null, h('th', { text: '' }), ...cols.map(c => h('th', { text: c })))), body); }
  function kv(keys) { const vals = {}; const box = h('div', { class: 'kvl' }); for (const k of keys) { const v = h('span', { class: 'num' }); vals[k] = v; box.append(h('div', null, h('span', { class: 'k', text: k }), v)); } return { box, vals }; }
  function fillRows(body, rows, cols, decimals) {
    const sig = rows.map(r => r.label).join('|');
    if (body.__sig !== sig) { body.__sig = sig; body.textContent = ''; for (const r of rows) body.append(h('tr', null, h('td', { text: r.label }), ...cols.map(() => h('td')))); }
    rows.forEach((r, i) => cols.forEach((_, k) => {
      const v = r.values[k], td = body.children[i].children[k + 1]; if (!td) return;
      setText(td, v == null ? '—' : fmt(v, r.d || decimals)); td.classList.toggle('dim', v == null);
    }));
  }

  function update(s) {
    if (!s) return;
    live.update(s);
    const d = s.dashboard || {}, sess = s.session || {}, tel = s.telemetry || {}, hl = tel.health || {};
    const robot = (s.robots && (s.robots.items.find(r => r.name === s.robots.current) || {}).label) || (s.studio && s.studio.robot_display) || '';
    const running = !!(sess.processes && (sess.processes.simulation || sess.processes.fic || sess.processes.controller));
    setText($('#dh-state'), running ? 'running' : 'idle'); $('#dh-dot').className = 'dot ' + (running ? 'ok live' : '');
    setText($('#dh-info'), [robot, 'Robot ' + (s.studio && s.studio.station), TARGET[sess.target], s.mode && s.mode.name].filter(Boolean).join(' · '));
    setText($('#dh-mods'), (d.modifiers || []).join('  '));
    const health = hl.state === 'live' ? '● live · ' + fmt(hl.rate, 0) + ' Hz · ' + fmt(hl.age_ms, 0) + ' ms' : hl.state === 'stale' ? '● stale · ' + fmt(hl.rate, 0) + ' Hz' : '— Hz · offline';
    setText($('#dh-health'), health); $('#dh-health').className = 'num ' + (hl.state === 'live' ? 'ok-t' : hl.state === 'stale' ? 'warn-t' : 'faint');
    setText($('#lede'), '');

    const joints = d.joints || [];
    if (tab === 'overview') {
      setText(baseRows['control mode'], (s.mode && s.mode.name) || '—');
      setText(baseRows.target, sess.target || '—');
      setText(baseRows.profile, (s.profiles && s.profiles.current) || '—');
      setText(baseRows.telemetry, health);
      fillRows(jbody, joints.map(j => ({ label: j.label, values: [j.commanded, j.position, j.error, j.velocity, j.torque] })), JOINT_COLS, 2);
      setText($('#jt-aside'), joints.length ? joints.length + ' joints' : '');
      const ee = d.ee || {};
      fillRows(ebody, EE_ROWS.map(k => ({ label: k, values: ee[k] || [], d: 3 })), EE_COLS, 3);
      const w = d.wrench || {};
      setText($('#w-f'), '|F| ' + (w.force_mag == null ? '—' : fmt(w.force_mag, 1)) + ' N'); setText($('#w-t'), '|τ| ' + (w.torque_mag == null ? '—' : fmt(w.torque_mag, 2)) + ' Nm');
      setText($('#w-c'), w.contact ? '● contact' : '○ free');
      fillRows(wbody, [{ label: 'force', values: w.force || [] }, { label: 'torque', values: w.torque || [] }], ['x', 'y', 'z'], 2);
      const sf = d.safety || {};
      for (const k of Object.keys(safeEls)) {      // never green without data: grey 'no data' until the robot reports
        const v = sf[k] === true ? 'ok' : sf[k] === false ? 'warn' : (sf[k] || 'unknown');
        safeEls[k].d.className = 'dot ' + (SAFE_DOT[v] || '');
        setText(safeEls[k].st, SAFE_TEXT[v] || 'no data');
      }
    } else {
      fillRows(jdbody, joints.map(j => ({ label: j.label, values: [j.commanded, j.position, j.error, j.velocity, j.torque, j.tau_max, j.pct_tau, j.lo, j.hi, j.pct_range] })), JOINT_DETAIL, 2);
      const ee = d.ee || {}, w = d.wrench || {};
      fillRows(edbody, EE_ROWS.concat(['quaternion']).map(k => ({ label: k, values: ee[k] || [], d: 4 })), EE_COLS, 4);
      const f = w.force, t = w.torque;
      fillRows(wdbody, [{ label: 'force', values: f ? f.concat([w.force_mag]) : [] }, { label: 'torque', values: t ? t.concat([w.torque_mag]) : [] }], ['x', 'y', 'z', 'magnitude'], 3);
      for (const [k, v] of Object.entries(d.robot || {})) if (robotKv.vals[k]) setText(robotKv.vals[k], v == null ? '—' : String(v));
      for (const [k, v] of Object.entries(d.channels || {})) if (chanKv.vals[k]) setText(chanKv.vals[k], v == null ? '—' : String(v));
    }
  }
  return { update, dispose: live.dispose };
};
})();
