/* Multi-robot page: every Studio on the network, lab-wide stops, per-station
   start / stop / drive, and teleoperation pairing. The fleet itself (presence,
   commands, pairing) is the Studio's; this page lists it and sends `fleet` and
   `teleop` commands. The Robots section is the classic page's robot selector
   and its Add robot entry, as cards: each station this Studio can command, the
   one the control bar drives, Pause and Stop for it, and a card that adds the
   next. The controls themselves are in the control bar at the bottom. */
(() => {
'use strict';
const { $, h, setText, section, pageTitle, cmd, toast, inlineConfirm } = window.FIC;
const TARGET = { simulation: 'Simulation', hardware: 'Hardware', vr: 'VR' };

window.FIC.pages.multi_robot = function buildFleet(root) {
  const body = h('tbody'), ask = h('div'), pairBox = h('div'), warn = h('div', { class: 'note-warn' }), tAsk = h('div'), err = h('div', { class: 'bad-line' });
  const leader = h('select', { class: 'select', 'aria-label': 'Leader' }), follower = h('select', { class: 'select', 'aria-label': 'Follower' });
  const scale = h('input', { class: 'input num', type: 'number', min: '0.05', max: '2', step: '0.05', value: '1', 'aria-label': 'Scale' });
  const speed = h('input', { class: 'input num', type: 'number', min: '5', max: '90', step: '5', value: '35', 'aria-label': 'Max speed' });
  const cards = h('div', { class: 'cards' }), addHost = h('div');
  const addNum = h('input', { class: 'input num', type: 'number', min: '1', max: '99', step: '1', value: '2', 'aria-label': 'Station number' });
  const addCard = h('div', { class: 'card add' }, h('div', { class: 'card-t', text: '＋ Add robot' }), h('div', { class: 'card-m faint', text: 'Bring another station online so the control bar can drive it.' }),
    h('div', { class: 'btn-row' }, h('div', { class: 'field' }, h('label', { text: 'Station' }), addNum),
      h('button', { class: 'btn btn-primary', type: 'button', text: 'Add robot', onclick: () => {
        const n = Number(addNum.value);
        inlineConfirm(addHost, { title: 'Add station ' + n + '?', lines: ['This Studio starts publishing to robot' + n + '_* topics.'], label: 'Add robot',
          onConfirm: async () => { const r = await cmd('station', { op: 'add', n }); if (r.ok) toast('Robot ' + n + ' added'); } });
      } })), addHost);
  root.append(
    ...pageTitle('Multi-Robot', 'The robots this Studio commands, the Studios on this network, and teleoperation between their arms.'),
    section('Robots', { id: 'st-aside' }, cards),
    h('div', { class: 'btn-row', style: 'margin-top:32px' },
      h('button', { class: 'btn btn-danger strong', type: 'button', text: 'Stop every robot', title: 'Lab-wide E-STOP: every station stops, no confirmation',
        onclick: async () => { const r = await cmd('fleet', { op: 'estop_all' }); if (r.ok) toast('E-STOP sent to every station'); } }),
      h('button', { class: 'btn', type: 'button', text: 'Pause all', title: 'Every robot ramps down and holds until Resume all', onclick: async () => { const r = await cmd('fleet', { op: 'pause_all' }); if (r.ok) toast('Pause sent'); } }),
      h('button', { class: 'btn', type: 'button', text: 'Resume all', title: 'Every paused robot gets its mode back', onclick: async () => { const r = await cmd('fleet', { op: 'resume_all' }); if (r.ok) toast('Resume sent'); } }),
      h('span', { class: 'faint', style: 'font-size:var(--fs-m)', text: 'Stop every robot acts at once, like E-STOP.' })),
    warn,
    section('Studios', { id: 'fl-aside' },
      h('table', { class: 'table left' }, h('thead', null, h('tr', null, ...['Station', 'Robot', 'Target', 'State', 'Mode', 'Teleop', ''].map(t => h('th', { text: t })))), body), ask),
    section('Teleoperation', { id: 'tp-aside' },
      h('div', { class: 'toolbar' },
        h('div', { class: 'field' }, h('label', { text: 'Leader' }), leader), h('div', { class: 'field' }, h('label', { text: 'Follower' }), follower),
        h('div', { class: 'field' }, h('label', { text: 'Scale' }), scale), h('div', { class: 'field' }, h('label', { text: 'Max speed °/s' }), speed),
        h('div', { class: 'spacer' }), h('button', { class: 'btn btn-primary', type: 'button', text: 'Start teleop', onclick: askTeleop })),
      h('p', { class: 'faint', style: 'font-size:var(--fs-m);margin-top:12px;max-width:46rem' }, 'Joint space, same robot type. The follower starts where it is and copies the leader’s motion. An E-stop on either side stops both.'),
      err, tAsk, pairBox));

  function stateOf(c) { return c.viewer ? ['ok', 'Watching'] : c.running ? ['ok', 'Running'] : c.has_joints ? ['ok', 'Live'] : ['', 'Idle']; }
  function askStation(c, session_op, target) {
    const hw = target === 'hardware';
    inlineConfirm(ask, {
      title: (session_op === 'start' ? 'Start ' + c.label + ' on ' + (TARGET[target] || target).toUpperCase() : 'Stop ' + c.label) + '?',
      lines: hw && session_op === 'start' ? ['Its controller and driver start, and the arm is held by the controller.', 'Make sure the area around it is clear.'] : [],
      label: session_op === 'start' ? 'Start ' + c.label : 'Stop ' + c.label, danger: hw || session_op === 'stop',
      onConfirm: async () => { const r = await cmd('fleet', { op: 'session', station: c.station, session_op, target, confirm: true, hardware: hw }); if (r.ok) toast('Asked ' + c.label + ' to ' + session_op); } });
  }
  function askDrive(c) {
    const hw = c.target === 'hardware';
    inlineConfirm(ask, { title: 'Drive ' + c.label + ' from here?', lines: [hw ? 'Its arm is on HARDWARE, run by ' + c.host + '. Commands from the control bar will move it.' : 'The control bar will command that station.'],
      label: 'Drive ' + c.label, danger: hw,
      onConfirm: async () => { const r = await cmd('fleet', { op: 'drive', station: c.station, confirm: true }); if (r.ok) toast('Control now points at ' + c.label); } });
  }
  function askTeleop() {
    setText(err, '');
    const l = Number(leader.value), f = Number(follower.value);
    if (!l || !f || l === f) { setText(err, 'Pick two different stations.'); return; }
    const lab = n => (leader.querySelector('option[value="' + n + '"]') || {}).textContent || 'Station ' + n;
    inlineConfirm(tAsk, { title: lab(l) + ' leads ' + lab(f) + '?', lines: ['Scale ' + scale.value + ', max ' + speed.value + ' °/s.', 'The follower’s Studio is asked to agree (at once in simulation, its operator on hardware).'],
      label: 'Start teleop',
      onConfirm: async () => { const r = await cmd('teleop', { op: 'start', leader: l, follower: f, scale: parseFloat(scale.value), max_speed_deg: parseFloat(speed.value), confirm: true }, true); if (r.ok) toast('Teleop requested'); else setText(err, r.message || 'Refused'); } });
  }
  let sig = '', psig = '';
  let csig = '';
  function drawStations(s) {
    const st = s.stations || { items: [], active: '' }, studios = (s.fleet && s.fleet.studios) || [];
    const sig = JSON.stringify([st, studios.map(c => [c.station, c.mode, c.target, c.running])]);
    if (sig === csig) return; csig = sig;
    cards.textContent = '';
    for (const it of st.items) {
      const active = it.rid === st.active, info = studios.find(c => c.station === it.n) || {}, own = it.n === (s.studio && s.studio.station);
      cards.append(h('div', { class: 'card' + (active ? ' on' : '') },
        h('div', { class: 'card-t' }, it.label, active ? h('span', { class: 'tag', text: '  driven by the control bar' }) : null),
        h('div', { class: 'card-m faint', text: [info.robot_display || info.robot || '', info.target || '', info.mode || ''].filter(Boolean).join(' · ') || 'Station ' + it.n }),
        h('div', { class: 'btn-row' },
          active ? null : h('button', { class: 'btn btn-primary', type: 'button', text: 'Control', onclick: () => cmd('station', { op: 'select', n: it.n }) }),
          h('button', { class: 'btn', type: 'button', text: 'Pause', onclick: () => cmd('robot_pause', { n: it.n }) }),
          h('button', { class: 'btn btn-danger', type: 'button', text: 'Stop', onclick: () => cmd('robot_stop', { n: it.n }) }),
          own ? null : h('button', { class: 'btn', type: 'button', text: 'Remove', onclick: () => inlineConfirm(addHost, { title: 'Remove ' + it.label + '?', lines: ['Its publishers and subscriptions on this Studio are torn down.'], label: 'Remove', danger: true,
            onConfirm: async () => { const r = await cmd('station', { op: 'remove', n: it.n, confirm: true }); if (r.ok) toast(it.label + ' removed'); } }) }))));
    }
    cards.append(addCard);
    addNum.value = String(st.next || 2);
    setText($('#st-aside'), st.items.length + ' active');
  }
  return s => {
    drawStations(s);
    const fl = s.fleet || {}, studios = fl.studios || [];
    if (!fl.available) { setText($('#lede'), 'This Studio is not on a fleet.'); body.textContent = ''; return; }
    const running = studios.filter(c => c.running).length;
    const pairs = new Set(studios.flatMap(c => (c.teleop || []).filter(p => p.state === 'active').map(p => p.sid)));
    setText($('#lede'), studios.length + ' Studio' + (studios.length === 1 ? '' : 's') + ' · ' + running + ' running · ' + pairs.size + ' teleop pair' + (pairs.size === 1 ? '' : 's'));
    warn.textContent = (fl.conflicts || []).map(c => c.label + ' is claimed by ' + c.hosts.join(' and ') + ': both command the same topics.').join('  ');
    const next = JSON.stringify(studios);
    if (next !== sig) {
      sig = next; body.textContent = '';
      for (const c of studios) {
        const [dot, text] = stateOf(c), remote = !c.self && !c.master && !c.vr;
        const lab = n => (studios.find(x => x.station === n) || {}).label || n;
        const roles = (c.teleop || []).map(p => p.leader === c.station ? 'leads ' + lab(p.follower) : p.follower === c.station ? 'follows ' + lab(p.leader) : '').filter(Boolean).join(', ');
        body.append(h('tr', null,
          h('td', null, c.label, c.self ? h('span', { class: 'faint', text: '  this Studio' }) : null, h('div', { class: 'faint', style: 'font-size:var(--fs-s)', text: c.host })),
          h('td', { text: c.robot_display || c.robot }), h('td', { text: (TARGET[c.target] || c.target) }),
          h('td', null, h('span', { class: 'dot ' + dot }), text), h('td', { text: c.mode || '—' }), h('td', { text: roles || '—' }),
          h('td', null, h('span', { class: 'btn-row', style: 'justify-content:flex-end;gap:16px' },
            remote ? h('button', { class: 'btn', type: 'button', text: 'Drive', onclick: () => askDrive(c) }) : null,
            remote && !c.running ? h('button', { class: 'btn', type: 'button', text: 'Start sim', onclick: () => askStation(c, 'start', 'simulation') }) : null,
            remote && !c.running ? h('button', { class: 'btn', type: 'button', text: 'Start hardware', onclick: () => askStation(c, 'start', 'hardware') }) : null,
            remote && c.running ? h('button', { class: 'btn btn-danger', type: 'button', text: 'Stop', onclick: () => askStation(c, 'stop') }) : null,
            c.urls && c.urls[0] ? h('a', { class: 'btn', href: c.urls[0], target: '_blank', rel: 'noopener', text: 'Web' }) : null))));
      }
      for (const sel of [leader, follower]) {
        const keep = sel.value;
        sel.replaceChildren(...studios.filter(c => !c.viewer).map(c => h('option', { value: String(c.station), text: c.label + ' — ' + c.host })));
        if (keep) sel.value = keep;
      }
      const robots = studios.filter(c => !c.viewer);
      if (!follower.__set && robots.length > 1) { follower.value = String(robots[1].station); follower.__set = true; }
    }
    setText($('#fl-aside'), studios.length + ' on the network');
    // pairings seen anywhere in the fleet
    const seen = new Map(), labels = new Map(studios.map(c => [c.station, c.label]));
    const name = n => labels.get(n) || 'Station ' + n;
    for (const c of studios) for (const p of c.teleop || []) seen.set(p.sid, p);
    const psigNew = JSON.stringify([...seen.values()]);
    if (psigNew !== psig) {
      psig = psigNew; pairBox.textContent = '';
      for (const p of seen.values()) pairBox.append(h('div', { class: 'pair' },
        h('span', null, name(p.leader) + ' → ' + name(p.follower), h('span', { class: 'faint', text: ' · ' + p.state })),
        h('button', { class: 'btn btn-danger', type: 'button', text: 'Stop', onclick: () => cmd('teleop', { op: 'stop', sid: p.sid, confirm: true }) })));
      if (!seen.size) pairBox.append(h('div', { class: 'empty', text: 'No pairings.' }));
    }
    setText($('#tp-aside'), seen.size ? seen.size + ' pairing' + (seen.size === 1 ? '' : 's') : '');
  };
};
})();
