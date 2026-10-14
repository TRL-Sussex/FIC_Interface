// Sequences — web page (manifest "web"). The worked example for extension web
// pages (fic_docs/pages/extensions.html): the Studio side is web_state() /
// web_action() in ../extension.py; this file only draws, with the UI kit
// (api.ui). It asks before anything moves the robot, and again, in red, when
// the sequence goes to a real robot.
const KINDS = [{ value: 'joint', label: 'Joint' }, { value: 'cartesian', label: 'Cartesian' }, { value: 'gripper', label: 'Gripper' }];

export default function build(root, api) {
  const { ui, h } = api;
  let s = null;                                    // the last web_state()
  let listSig = '';
  let editing = null;                              // index of the waypoint in the form, or null for a new one

  // ── Sequence ──
  const pick = ui.select({ options: [], onChange: v => api.call('select', { name: v }) });
  const ask = h('div', { class: 'sq-ask' });

  // ── Playback ──
  const playBtn = ui.button({ label: 'Play', icon: 'play', kind: 'solid', onClick: () => play() });
  const stepBtn = ui.button({ label: 'Step', title: 'Move to the next waypoint', onClick: () => step() });
  const stopBtn = ui.button({ label: 'Stop', icon: 'stop', kind: 'danger', onClick: () => api.call('stop') });
  const loop = ui.toggle({ label: 'Loop', onChange: v => api.call('loop', { value: v }) });
  const status = h('span', { class: 'sq-status' });
  const target = ui.select({ options: [], onChange: v => api.call('target', { value: v }) });
  const sendsTo = h('span', { class: 'sq-runs' });

  // ── Waypoints ──
  const list = h('div', { class: 'sq-list' });

  // ── The waypoint form: capture from the robot, or type the values ──
  const kind = ui.select({ options: KINDS, value: 'joint', onChange: () => drawForm() });
  const hold = ui.number({ value: 1.0, min: 0, max: 600, step: 0.5, unit: 's', digits: 1 });
  const fields = h('div', { class: 'sq-fields' });
  const formTitle = h('h3', { class: 'sq-form-title' });
  let inputs = [];
  const captureBtn = ui.button({ label: 'Capture from robot', icon: 'record', kind: 'primary',
    onClick: () => api.call('capture', { kind: kind.value, hold: num(hold) }) });
  const addBtn = ui.button({ label: 'Add', onClick: () => submit() });
  const cancelBtn = ui.button({ label: 'Cancel', onClick: () => edit(null) });

  // ── Playback settings ──
  const speed = setting('speed', 0.1, 3, 0.1, '×', 1);
  const jointTol = setting('joint_tol_deg', 0.5, 20, 0.5, '°', 1);
  const posTol = setting('pos_tol_mm', 1, 50, 1, 'mm', 0);
  const timeout = setting('timeout_s', 5, 120, 5, 's', 0);

  root.append(h('div', { class: 'sq-page' },
    h('div', { class: 'sq-bar' }, pick,
      ui.buttons(
        ui.button({ label: 'New', onClick: () => named('New sequence', '', n => api.call('new', { name: n })) }),
        ui.button({ label: 'Rename', onClick: () => s && named('Rename “' + s.current + '”', s.current, n => api.call('rename', { name: n })) }),
        ui.button({ label: 'Delete', kind: 'danger', onClick: () => s && api.confirm(ask, {
          title: 'Delete the sequence “' + s.current + '”?', label: 'Delete', danger: true, onConfirm: () => api.call('delete_sequence') }) })),
      h('span', { class: 'sq-grow' }),
      ui.buttons(
        ui.button({ label: 'Save file', title: 'Write every sequence to the sequences file', onClick: () => api.call('save').then(r => { if (r) api.toast('Saved to ' + s.file); }) }),
        ui.button({ label: 'Reload file', onClick: () => api.confirm(ask, { title: 'Reload the sequences file?',
          lines: ['Changes not saved to the file are lost.'], label: 'Reload', onConfirm: () => api.call('load') }) }))),
    h('div', { class: 'sq-bar' }, ui.buttons(playBtn, stepBtn, stopBtn), loop, status, h('span', { class: 'sq-grow' }),
      h('label', { class: 'sq-to', text: 'Send to' }), target, sendsTo),
    ask,
    h('div', { class: 'sq-work' },
      ui.section('Waypoints', list),
      h('div', { class: 'sq-col' },
        h('section', { class: 'section sq-form' }, formTitle, ui.field('Type', kind), fields, ui.field('Hold', hold),
          ui.buttons(addBtn, cancelBtn), h('div', { class: 'sq-or', text: 'or' }), ui.buttons(captureBtn),
          ui.note('Capture adds the robot’s live position as a new waypoint.')),
        ui.section('Playback', ui.field('Speed', speed), ui.field('Joint tolerance', jointTol),
          ui.field('Position tolerance', posTol), ui.field('Timeout', timeout),
          ui.note('Speed scales the hold times; the planner sets the motion speed.'))))));

  function num(el) { const v = parseFloat(el.input.value); return isNaN(v) ? 0 : v; }
  function setting(key, min, max, step, unit, digits) {
    return ui.number({ min, max, step, unit, digits, onChange: v => api.call('settings', { key, value: v }) });
  }
  /** Ask for a name in place (New, Rename). */
  function named(title, value, go) {
    const box = ui.text({ value, placeholder: 'Name' });
    const close = () => { ask.textContent = ''; };
    const ok = async () => { const r = await go(box.value.trim()); if (r) close(); };
    box.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); ok(); } else if (e.key === 'Escape') close(); });
    ask.replaceChildren(h('div', { class: 'sq-name' }, h('label', { text: title }), box,
      ui.buttons(ui.button({ label: 'OK', kind: 'primary', onClick: ok }), ui.button({ label: 'Cancel', onClick: close }))));
    box.focus(); box.select();
  }
  /** Ask before anything that moves the robot; on hardware say so, in red. */
  function moving(title, go) {
    if (!s) return;
    if (!s.hardware) return go(false);
    api.confirm(ask, { title: title.replace(/\?$/, ' on the real robot?'), lines: ['Sends to ' + s.target + '.'],
      label: 'Move the real robot', danger: true, onConfirm: () => go(true) });
  }
  function play() {
    if (!s) return;
    moving('Play “' + s.current + '” (' + s.waypoints.length + ' waypoints)?',
      ok => api.call('play', { loop: loop.input.checked, hardware_ok: ok }));
  }
  function step() { moving('Move to the next waypoint?', ok => api.call('step', { hardware_ok: ok })); }

  // ── The form ──
  function drawForm(wp) {
    const k = kind.value;
    const v = wp || {};
    if (k === 'joint') {
      const n = s ? s.joints : 7;
      inputs = Array.from({ length: n }, (_, i) => ui.number({ value: (v.values || [])[i] != null ? v.values[i] : 0, min: -360, max: 360, step: 1, unit: '°', digits: 2 }));
      fields.replaceChildren(h('div', { class: 'sq-grid' }, ...inputs.map((el, i) => ui.field('J' + (i + 1), el))));
    } else if (k === 'cartesian') {
      const p = v.position || [0.4, 0, 0.4], a = v.axis_angle || [1, 0, 0, 180];
      inputs = ['X', 'Y', 'Z'].map((l, i) => ui.number({ value: p[i], min: -3, max: 3, step: 0.01, unit: 'm', digits: 3 }))
        .concat(['Axis x', 'Axis y', 'Axis z'].map((l, i) => ui.number({ value: a[i], min: -1, max: 1, step: 0.1, digits: 3 })),
          [ui.number({ value: a[3], min: -360, max: 360, step: 5, unit: '°', digits: 1 })]);
      const labels = ['X', 'Y', 'Z', 'Axis x', 'Axis y', 'Axis z', 'Angle'];
      fields.replaceChildren(h('div', { class: 'sq-grid' }, ...inputs.map((el, i) => ui.field(labels[i], el))));
    } else {
      inputs = [ui.number({ value: v.width != null ? v.width * 1000 : 80, min: 0, max: 80, step: 5, unit: 'mm', digits: 0 })];
      fields.replaceChildren(ui.field('Width', inputs[0]));
    }
    formTitle.textContent = editing == null ? 'Add a waypoint' : 'Edit waypoint ' + (editing + 1);
    addBtn.set({ label: editing == null ? 'Add' : 'Update' });
    cancelBtn.hidden = editing == null;
    captureBtn.hidden = editing != null;
  }
  function edit(index) {
    editing = index;
    const wp = index == null ? null : s.waypoints[index];
    if (wp) { kind.value = wp.type; hold.set(wp.hold); }
    drawForm(wp || undefined);
    drawList();
  }
  async function submit() {
    const k = kind.value;
    const vals = inputs.map(num);
    const waypoint = { type: k, hold: num(hold) };
    if (k === 'joint') waypoint.values = vals;
    else if (k === 'cartesian') { waypoint.position = vals.slice(0, 3); waypoint.axis_angle = vals.slice(3, 7); }
    else waypoint.width = vals[0] / 1000;
    const r = editing == null ? await api.call('add', { waypoint }) : await api.call('update', { index: editing, waypoint });
    if (r) { api.toast(editing == null ? 'Added waypoint ' + (r.index + 1) : 'Updated waypoint ' + (r.index + 1)); edit(null); }
  }

  function drawList() {
    if (!s) return;
    const rows = s.waypoints.map(w => {
      const at = s.playing && s.play_index === w.index;
      const holdBox = ui.number({ value: w.hold, min: 0, max: 600, step: 0.5, unit: 's', digits: 1,
        onChange: v => api.call('hold', { index: w.index, value: v }) });
      return h('div', { class: 'sq-row' + (at ? ' at' : '') + (editing === w.index ? ' editing' : '') + (!s.playing && s.selected === w.index ? ' sel' : '') },
        h('span', { class: 'sq-n num', text: String(w.index + 1) }),
        h('span', { class: 'sq-t chip', text: w.type }),
        h('span', { class: 'sq-v num', text: w.text }),
        holdBox,
        ui.buttons(
          ui.button({ label: 'Go', title: 'Move to this waypoint', disabled: s.playing,
            onClick: () => moving('Move to waypoint ' + (w.index + 1) + '?', ok => api.call('goto', { index: w.index, hardware_ok: ok })) }),
          ui.button({ label: '↑', title: 'Move up', disabled: s.playing || w.index === 0, onClick: () => api.call('move', { index: w.index, dir: 'up' }) }),
          ui.button({ label: '↓', title: 'Move down', disabled: s.playing || w.index === s.waypoints.length - 1, onClick: () => api.call('move', { index: w.index, dir: 'down' }) }),
          ui.button({ label: 'Edit', disabled: s.playing, onClick: () => edit(w.index) }),
          ui.button({ label: 'Delete', kind: 'danger', disabled: s.playing, onClick: () => api.confirm(ask, {
            title: 'Delete waypoint ' + (w.index + 1) + '?', label: 'Delete', danger: true,
            onConfirm: () => { if (editing === w.index) edit(null); api.call('delete', { index: w.index }); } }) })));
    });
    list.replaceChildren(...(rows.length ? rows : [ui.note('No waypoints yet: capture the robot’s position or type one on the right.')]));
  }

  drawForm();
  return {
    update(state) {
      const first = !s;
      s = state;
      pick.setOptions(s.sequences.map(q => ({ value: q.name, label: q.name + '  (' + q.count + ')' })), s.current);
      target.setOptions(s.targets, s.target); target.disabled = s.playing;
      sendsTo.textContent = s.hardware ? 'real robot' : '';
      sendsTo.classList.toggle('hw', s.hardware);
      loop.set(s.loop);
      status.textContent = s.status + (s.playing && s.play_index != null ? ' · ' + (s.play_index + 1) + ' / ' + s.waypoints.length : '');
      status.className = 'sq-status' + (s.playing ? ' on' : '');
      playBtn.disabled = s.playing || !s.waypoints.length;
      stepBtn.disabled = s.playing || !s.waypoints.length;
      stopBtn.disabled = !s.playing;
      speed.set(s.speed); jointTol.set(s.joint_tol_deg); posTol.set(s.pos_tol_mm); timeout.set(s.timeout_s);
      if (editing != null && editing >= s.waypoints.length) edit(null);
      if (first) drawForm();
      const sig = JSON.stringify([s.current, s.waypoints, s.playing, s.play_index, s.selected, editing]);
      if (sig !== listSig && !list.contains(document.activeElement)) { listSig = sig; drawList(); }
    },
    dispose() {},
  };
}
