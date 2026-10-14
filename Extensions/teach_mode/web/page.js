// Teach Mode — web page (manifest "web"). The Studio side is web_state() /
// web_action() in ../extension.py. Guide the robot by hand in admittance,
// record waypoints, and save them as a sequence. The page asks before anything
// moves the robot, and again, in red, on the real robot.
export default function build(root, api) {
  const { ui, h } = api;
  let s = null;                                    // the last web_state()
  let wpSig = '';

  const mode = ui.select({ options: ['Joint', 'Cartesian'], onChange: v => api.call('mode', { value: v }) });
  const robot = ui.select({ options: [], onChange: v => api.call('robot', { value: v }) });
  const startBtn = ui.button({ label: 'Start teaching', icon: 'play', kind: 'solid', onClick: () => start() });
  const stopBtn = ui.button({ label: 'Stop teaching', icon: 'stop', kind: 'danger', onClick: () => api.call('stop') });
  const status = h('span', { class: 'tm-status' });
  const runsOn = h('span', { class: 'tm-runs' });
  const ask = h('div', { class: 'tm-ask' });

  // ── Live position ──
  const jointBox = h('div', { class: 'tm-live' });
  const cartBox = h('div', { class: 'tm-live' });

  // ── Record ──
  const recordBtn = ui.button({ label: 'Record position', icon: 'record', kind: 'primary', title: 'Space',
    onClick: () => api.call('record') });
  const auto = ui.toggle({ label: '', title: 'Record by itself while teaching', onChange: v => api.call('auto', { enabled: v }) });
  const every = ui.number({ min: 0.5, max: 10, step: 0.5, unit: 's', digits: 1, onChange: v => api.call('auto', { interval: v }) });
  const moved = ui.number({ min: 0.1, max: 50, step: 1, unit: 'mm', digits: 1, onChange: v => api.call('auto', { threshold: v }) });

  // ── Gripper (width in mm on the page, metres in the Studio) ──
  const width = ui.number({ min: 0, max: 80, step: 5, unit: 'mm', digits: 0, onChange: v => api.call('gripper', { width: v / 1000 }) });
  const sendGripper = w => moving('Move the gripper to ' + Math.round(w * 1000) + ' mm?',
    ok => api.call('gripper', { width: w, send: true, hardware_ok: ok }));
  const gripperRecord = ui.button({ label: 'Record gripper', icon: 'record', onClick: () => api.call('record_gripper') });

  // ── Waypoints and saving ──
  const list = h('div', { class: 'tm-list' });
  const count = h('span', { class: 'tm-count' });
  const seqName = ui.text({ placeholder: 'Sequence name', onChange: v => api.call('name', { value: v }) });
  const saveBtn = ui.button({ label: 'Save as sequence', kind: 'primary', onClick: () => saveSequence() });
  const seqNote = ui.note('');

  root.append(h('div', { class: 'tm-page' },
    h('div', { class: 'tm-bar' }, ui.field('Mode', mode), ui.field('Robot', robot), ui.buttons(startBtn, stopBtn), status,
      h('span', { class: 'tm-grow' }), runsOn),
    ask,
    h('div', { class: 'tm-work' },
      h('div', { class: 'tm-col' },
        ui.section('Joints', jointBox),
        ui.section('End effector', cartBox),
        ui.section('Record', ui.buttons(recordBtn), ui.field('Auto-record', auto), ui.field('Every', every), ui.field('Only if moved', moved)),
        ui.section('Gripper', ui.field('Width', width),
          ui.buttons(ui.button({ label: 'Open', onClick: () => sendGripper(0.08) }),
            ui.button({ label: 'Close', onClick: () => sendGripper(0) }),
            ui.button({ label: 'Send', kind: 'primary', onClick: () => sendGripper(s ? s.gripper_width : 0.08) })),
          ui.buttons(gripperRecord))),
      h('div', { class: 'tm-col' },
        ui.section('Waypoints', { aside: '' }, list,
          ui.buttons(ui.button({ label: 'Undo last', onClick: () => api.call('undo') }),
            ui.button({ label: 'Clear all', kind: 'danger', onClick: () => clearAll() })), count),
        ui.section('Save', ui.field('Name', seqName), ui.buttons(saveBtn), seqNote)))));

  root.addEventListener('keydown', e => {
    if (e.code === 'Space' && s && s.teaching && !/INPUT|SELECT|TEXTAREA|BUTTON/.test(e.target.tagName)) { e.preventDefault(); api.call('record'); }
  });

  /** Ask before anything that moves the robot; on hardware say so, in red. */
  function moving(title, go) {
    if (!s) return;
    const hw = s.target === 'hardware';
    api.confirm(ask, { title: hw ? title.replace(/\?$/, ' on the real robot?') : title, lines: [s.robot],
      label: hw ? 'Move the real robot' : 'Move', danger: hw, onConfirm: () => go(hw) });
  }
  function start() {
    if (!s) return;
    const adm = s.mode === 'Cartesian' ? 'Cartesian Admittance' : 'Joint Admittance';
    const hw = s.target === 'hardware';
    api.confirm(ask, { title: 'Switch ' + s.robot + ' to ' + adm + '?',
      lines: ['You can then guide it by hand and record ' + s.mode.toLowerCase() + ' waypoints.', 'Make sure the area is clear.'],
      label: hw ? 'Start on the real robot' : 'Start teaching', danger: hw,
      onConfirm: () => api.call('start', { hardware_ok: hw }) });
  }
  function clearAll() {
    if (!s || !s.waypoints.length) return;
    api.confirm(ask, { title: 'Delete all ' + s.waypoints.length + ' recorded waypoints?', label: 'Delete all', danger: true,
      onConfirm: () => api.call('clear') });
  }
  async function saveSequence() {
    const r = await api.call('save_sequence', { name: seqName.value.trim() });
    if (r) api.toast('Saved ' + r.count + ' waypoints as “' + r.name + '”. Play it on the Sequences page.');
  }

  function live(box, labels, values, units) {
    box.replaceChildren(...labels.map((l, i) => h('div', { class: 'tm-cell' }, h('span', { class: 'tm-k', text: l }),
      h('span', { class: 'tm-v num', text: values[i] == null ? '—' : values[i].toFixed(units[i] === 'mm' ? 1 : 2) }),
      h('span', { class: 'tm-u', text: units[i] }))));
  }
  function drawList() {
    const rows = s.waypoints.map(w => h('div', { class: 'tm-row' },
      h('span', { class: 'tm-n num', text: String(w.index + 1) }),
      h('span', { class: 'tm-t chip', text: w.mode }),
      h('span', { class: 'tm-wv num', text: w.text }),
      h('span', { class: 'tm-time num', text: w.time.toFixed(1) + ' s' }),
      ui.buttons(
        ui.button({ label: 'Go', title: 'Move to this waypoint', onClick: () => moving('Move to waypoint ' + (w.index + 1) + '?',
          ok => api.call('go', { index: w.index, hardware_ok: ok })) }),
        ui.button({ label: 'Delete', kind: 'danger', onClick: () => api.call('delete', { index: w.index }) }))));
    list.replaceChildren(...(rows.length ? rows : [ui.note(s.teaching ? 'Guide the robot, then Record position (or press Space).' : 'Start teaching to record waypoints.')]));
    if (rows.length) list.scrollTop = list.scrollHeight;
  }

  return {
    update(state) {
      s = state;
      mode.set(s.mode); mode.disabled = s.teaching;
      robot.setOptions(s.robots, s.robot_id); robot.disabled = s.teaching;
      startBtn.disabled = s.teaching; stopBtn.disabled = !s.teaching;
      status.textContent = s.teaching ? 'Teaching (' + s.mode + ') · ' + Math.round(s.elapsed_s) + ' s' : 'Not teaching';
      status.className = 'tm-status' + (s.teaching ? ' on' : '');
      const target = s.target || 'simulation';
      runsOn.textContent = target.charAt(0).toUpperCase() + target.slice(1) + ' · ' + s.robot;
      runsOn.classList.toggle('hw', target === 'hardware');
      live(jointBox, s.joints_deg.length ? s.joints_deg.map((_, i) => 'J' + (i + 1)) : ['J1'], s.joints_deg.length ? s.joints_deg : [null],
        s.joints_deg.length ? s.joints_deg.map(() => '°') : ['°']);
      live(cartBox, ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'], s.cart.length ? s.cart : [null, null, null, null, null, null], ['mm', 'mm', 'mm', '°', '°', '°']);
      recordBtn.disabled = !s.teaching; gripperRecord.disabled = !s.teaching;
      auto.set(s.auto_record); auto.input.disabled = !s.teaching;
      every.set(s.auto_interval_s); moved.set(s.threshold_mm);
      width.set(s.gripper_width * 1000);
      seqName.set(s.sequence_name);
      saveBtn.disabled = !s.waypoints.length || !s.sequences;
      seqNote.set(s.sequences ? '' : 'Turn on the Sequences extension (Settings › Extensions) to save sequences.', 'warn');
      count.textContent = s.waypoints.length ? s.waypoints.length + ' waypoint' + (s.waypoints.length === 1 ? '' : 's') : '';
      const sig = JSON.stringify([s.waypoints, s.teaching]);
      if (sig !== wpSig) { wpSig = sig; drawList(); }
    },
    dispose() {},
  };
}
