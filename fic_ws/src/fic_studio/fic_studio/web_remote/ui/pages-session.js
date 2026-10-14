/* Session page: the processes behind Start / Stop, and teleoperation pairing.
   Every button is one command (`session`, `process`, `teleop`) answered by the
   Python Studio; this file only draws the snapshot and asks. */
(() => {
'use strict';
const { $, h, fmt, setText, section, pageTitle, cmd, toast, inlineConfirm } = window.FIC;
const STATE_TEXT = { idle: 'Idle', partial: 'Partial', running: 'Running' };

window.FIC.pages.session = function buildSession(root) {
  const procBody = h('tbody', { id: 'proc-body' });
  const pairBox = h('div', { id: 'pairs' });
  const askHost = h('div', { id: 'sess-ask' });
  const procAsk = h('div', { id: 'proc-ask' });
  const stat = (id, k) => h('div', { class: 'stat' }, h('div', { class: 'k', text: k }), h('div', { class: 'v', id }), h('div', { class: 'n', id: id + '-n' }));
  const bigBtn = h('button', { class: 'btn btn-primary', id: 'sess-big', type: 'button', onclick: askSession }, 'Start session');
  root.append(
    ...pageTitle('Session', 'Start and stop what the robot runs, and answer teleoperation pairings.'),
    h('div', { class: 'stats' }, stat('ss-state', 'State'), stat('ss-target', 'Target'), stat('ss-parts', 'Processes up'), stat('ss-loop', 'Inner loop')),
    h('div', { class: 'btn-row', style: 'margin-top:32px' }, bigBtn, h('span', { class: 'faint', id: 'sess-hint', style: 'font-size:var(--fs-m)' })),
    askHost,
    section('Processes', { id: 'proc-aside' },
      h('table', { class: 'table left' }, h('thead', null, h('tr', null, h('th', { text: 'Process' }), h('th', { text: 'State' }), h('th', { text: '' }))), procBody), procAsk),
    section('Teleoperation', { id: 'tp-aside' }, pairBox,
      h('p', { class: 'faint', style: 'margin-top:16px;font-size:var(--fs-m)' }, 'Start a pairing from ', h('a', { href: '#/multi_robot', text: 'Multi-robot' }), '.')),
    section('Recent log', { id: 'sl-aside' }, h('div', { class: 'log', id: 'sess-log' })));

  function askSession() {
    const s = window.FIC.state(); if (!s) return;
    const running = s.session.state !== 'idle';
    const op = running ? 'stop' : 'start', target = s.session.target;
    inlineConfirm(askHost, {
      title: (running ? 'Stop the ' : 'Start a ') + target + ' session?',
      lines: target === 'hardware' && !running ? ['Starts the driver and the controller; the arm is held by the controller.', 'Make sure the area around it is clear.'] : [],
      label: running ? 'Stop session' : 'Start session', danger: running || target === 'hardware',
      onConfirm: async () => { const r = await cmd('session', { op, confirm: true }); if (r.ok) toast('Session ' + (op === 'start' ? 'starting' : 'stopping')); },
    });
  }
  function askProcess(p, op) {
    inlineConfirm(procAsk, {
      title: (op === 'start' ? 'Start ' : 'Stop ') + p.label + '?',
      lines: p.hardware && op === 'start' ? ['This talks to the real robot.'] : [],
      label: (op === 'start' ? 'Start ' : 'Stop ') + p.label, danger: op === 'stop' || p.hardware,
      onConfirm: async () => { const r = await cmd('process', { id: p.id, op, confirm: true }); if (r.ok && !r.already) toast(p.label + (op === 'start' ? ' starting' : ' stopping')); },
    });
  }
  let psig = '';
  return s => {
    const sess = s.session || {}, running = sess.state && sess.state !== 'idle';
    setText($('#lede'), 'Target ' + (sess.target || '') + ' · ' + (STATE_TEXT[sess.state] || '').toLowerCase());
    setText($('#ss-state'), STATE_TEXT[sess.state] || '—');
    setText($('#ss-state-n'), sess.state === 'partial' ? 'some processes are down' : (running ? 'all processes up' : 'nothing running'));
    setText($('#ss-target'), ({ simulation: 'Simulation', hardware: 'Hardware', vr: 'VR' })[sess.target] || '—');
    setText($('#ss-target-n'), sess.driver_configured ? 'driver configured' : 'no driver command');
    setText($('#ss-parts'), (sess.parts_up || 0) + ' / ' + (sess.parts_total || 0));
    setText($('#ss-parts-n'), 'for this target');
    const il = s.inner_loop || {};
    setText($('#ss-loop'), il.path === 'mcu' ? 'MCU' : 'Standard');
    setText($('#ss-loop-n'), il.path === 'mcu' ? (il.state || '') + (il.where ? ' · ' + il.where : '') : 'controller on this PC');
    bigBtn.firstChild.textContent = running ? 'Stop session' : 'Start session';
    bigBtn.className = 'btn ' + (running ? 'btn-danger' : 'btn-primary');
    setText($('#sess-hint'), 'Same as Start / Stop in the top bar [S]');

    // processes
    const list = sess.process_list || [];
    const sig = list.map(p => p.id + p.running).join('|');
    if (sig !== psig) {
      psig = sig; procBody.textContent = '';
      for (const p of list) {
        procBody.append(h('tr', null,
          h('td', null, p.label, p.hardware ? h('span', { class: 'tag faint', text: ' hardware', style: 'font-size:var(--fs-xs)' }) : null),
          h('td', null, h('span', { class: 'dot ' + (p.running ? 'ok' : '') }), p.running ? 'Running' : 'Stopped'),
          h('td', null, h('button', { class: 'btn' + (p.running ? ' btn-danger' : ' btn-primary'), type: 'button',
            text: p.running ? 'Stop' : 'Start', onclick: () => askProcess(p, p.running ? 'stop' : 'start') }))));
      }
    }
    setText($('#proc-aside'), list.filter(p => p.running).length + ' of ' + list.length + ' running');

    // teleoperation: requests waiting here first, then everything in progress
    const tp = s.teleop || {};
    const sigT = JSON.stringify([tp.pending, tp.sessions]);
    if (pairBox.__sig !== sigT) {
      pairBox.__sig = sigT; pairBox.textContent = '';
      if (!tp.available) pairBox.append(h('div', { class: 'empty', text: 'This Studio has no teleoperation.' }));
      else {
        for (const p of tp.pending || []) {
          pairBox.append(h('div', { class: 'pair' },
            h('span', null, h('b', { text: 'Station ' + p.leader }), ' (' + (p.from_host || '?') + ') asks to lead this robot · scale ' + p.scale),
            h('span', { class: 'btn-row' },
              h('button', { class: 'btn btn-primary', type: 'button', text: 'Accept', onclick: () => cmd('teleop', { op: 'accept', sid: p.sid, confirm: true }) }),
              h('button', { class: 'btn', type: 'button', text: 'Refuse', onclick: () => cmd('teleop', { op: 'reject', sid: p.sid, confirm: true }) }))));
        }
        for (const x of (tp.sessions || []).filter(x => x.state !== 'pending' || x.follower !== tp.station)) {
          pairBox.append(h('div', { class: 'pair' },
            h('span', null, 'Station ' + x.leader + ' → station ' + x.follower, h('span', { class: 'faint', text: ' · ' + x.state + (x.holding ? ' (holding)' : '') + (x.reason ? ' · ' + x.reason : '') })),
            ['active', 'requested', 'pending'].includes(x.state) ? h('button', { class: 'btn btn-danger', type: 'button', text: 'Stop', onclick: () => cmd('teleop', { op: 'stop', sid: x.sid, confirm: true }) }) : null));
        }
        if (!pairBox.children.length) pairBox.append(h('div', { class: 'empty', text: 'No pairings.' }));
      }
    }
    setText($('#tp-aside'), (tp.pending || []).length ? (tp.pending.length + ' waiting for you') : '');

    const lines = (s.log || []).slice(-8);
    $('#sess-log').replaceChildren(...lines.map(e => h('div', { class: e.level }, e.time + '  ' + (e.level || '').padEnd(5) + '  ' + e.message)));
  };
};
})();
