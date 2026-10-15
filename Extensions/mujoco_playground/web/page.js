// MuJoCo Playground — web page (manifest "web"). The Studio side is web_state() /
// web_action() in ../extension.py; this file only draws, with the UI kit (api.ui).
const MM = v => (v * 1000).toFixed(0);

export default function build(root, api) {
  const { ui, h } = api;
  let s = null;
  let objSig = '', poseSig = '';
  let editing = null;                                 // index of the object in the form, or null

  // ── Robot, scene, start ──
  const robot = ui.select({ options: [], onChange: v => api.call('robot', { name: v }) });
  const preset = ui.select({ options: [], onChange: v => api.call('preset', { key: v }) });
  const startBtn = ui.button({ label: 'Start', icon: 'play', kind: 'solid', onClick: () => api.call('start') });
  const stopBtn = ui.button({ label: 'Stop', icon: 'stop', kind: 'danger', onClick: () => api.call('stop') });
  const status = ui.note('');
  const robotLike = ui.toggle({ label: 'Simulate like the real robot', title: 'The arm\'s joint friction, damping and sensor noise (robots_config.yaml hardware_model): more realistic, slower to reach targets. Off: the ideal simulator.',
    onChange: v => api.call('robot_like', { value: v }) });
  const robotNote = ui.note('');

  // ── Objects ──
  const table = ui.toggle({ label: 'Table', title: 'A table in front of the robot; objects on it rest on its top',
    onChange: v => api.call('table', { value: v }) });
  const objList = h('div', { class: 'pg-list' });
  const name = ui.text({ placeholder: 'name' });
  const shape = ui.select({ options: ['box', 'sphere', 'cylinder'], value: 'box', onChange: () => drawSize() });
  const color = ui.select({ options: ['red'], value: 'red' });
  const x = ui.number({ value: 0.5, min: -1.2, max: 1.2, step: 0.01, unit: 'm', digits: 2 });
  const y = ui.number({ value: 0.0, min: -1.2, max: 1.2, step: 0.01, unit: 'm', digits: 2 });
  const sizeBox = h('span', { class: 'pg-size' });
  let sizeInputs = [];
  const saveBtn = ui.button({ label: 'Add object', kind: 'primary', onClick: () => submit() });
  const cancelBtn = ui.button({ label: 'Cancel', onClick: () => edit(null) });

  // ── Live poses ──
  const poses = h('div', { class: 'pg-poses' });
  const log = ui.log({ lines: [], height: 140 });

  const learn = h('ol', { class: 'pg-learn' },
    h('li', null, h('b', { text: 'Move the arm. ' }), 'Open Joint or Cart in the bar at the bottom and drag a slider; Gripper opens and closes the hand.'),
    h('li', null, h('b', { text: 'Record waypoints. ' }), 'In Quick Actions make a new sequence, move the arm, and press Snapshot for each point you want.'),
    h('li', null, h('b', { text: 'Play it. ' }), 'Quick Actions plays, steps through or loops the sequence; change the speed there too. The Sequences page edits it.'),
    h('li', null, h('b', { text: 'Trajectories. ' }), 'Quick Actions → Trajectory runs a line, a circle or a figure-eight from where the arm is, and can save it as a sequence to study.'),
    h('li', null, h('b', { text: 'Pick an object. ' }), 'Quick Actions → Pick builds the approach, grasp and lift for an object as a sequence you can play and edit.'),
    h('li', null, h('b', { text: 'Script it. ' }), 'The Script Editor drives the same arm from Python: its examples move joints and trace a circle.'));

  root.append(h('div', { class: 'pg-page' },
    ui.section('Playground',
      ui.grid(3, ui.field('Robot', robot), ui.field('Scene', preset), ui.buttons(startBtn, stopBtn)),
      robotLike,
      robotNote, status),
    ui.grid(2,
      ui.section('Objects', { aside: '' }, table, objList,
        h('div', { class: 'pg-form' },
          ui.field('Name', name), ui.field('Shape', shape), ui.field('Size', sizeBox),
          ui.field('Position', h('span', { class: 'pg-xy' }, h('span', { class: 'faint', text: 'x' }), x, h('span', { class: 'faint', text: 'y' }), y)),
          ui.field('Colour', color), ui.buttons(saveBtn, cancelBtn))),
      ui.section('Where things are', poses, h('h3', { class: 'pg-h', text: 'Try this' }), learn)),
    ui.section('Log', log)));

  function drawSize(values) {
    const fields = (s && s.shapes[shape.value]) || ['size'];
    const defaults = { box: [0.02, 0.02, 0.02], sphere: [0.025], cylinder: [0.02, 0.04] }[shape.value] || [];
    sizeInputs = fields.map((f, i) => ui.number({ value: (values || defaults)[i], min: 0.005, max: 0.15, step: 0.005, digits: 3, unit: f.replace('_', ' ') }));
    sizeBox.replaceChildren(...sizeInputs);
  }
  const num = el => parseFloat(el.input.value);

  function edit(i) {
    editing = i;
    const o = i == null ? null : s.objects[i];
    name.value = o ? o.name : '';
    shape.value = o ? o.shape : 'box';
    color.value = o ? o.color : 'red';
    x.set(o ? o.x : 0.5); y.set(o ? o.y : 0.0);
    drawSize(o ? o.size : null);
    saveBtn.set({ label: i == null ? 'Add object' : 'Save object' });
    cancelBtn.hidden = i == null;
  }

  function submit() {
    const object = { name: name.value, shape: shape.value, color: color.value, x: num(x), y: num(y), size: sizeInputs.map(num) };
    const p = editing == null ? api.call('add_object', { object }) : api.call('update_object', { index: editing, object });
    p.then(r => { if (r) edit(null); });
  }

  function drawObjects() {
    const busy = s.running || s.starting;
    const rows = s.objects.map((o, i) => h('div', { class: 'pg-row' },
      h('span', { class: 'pg-dot', style: 'background:' + o.color }),
      h('span', { class: 'pg-name', text: o.name }),
      h('span', { class: 'faint', text: o.shape + ' · ' + o.size.map(MM).join('×') + ' mm · at ' + o.x.toFixed(2) + ', ' + o.y.toFixed(2) + ' m' }),
      h('span', { class: 'pg-grow' }),
      ui.buttons(ui.button({ label: 'Edit', disabled: busy, onClick: () => edit(i) }),
        ui.button({ label: 'Remove', kind: 'danger', disabled: busy, onClick: () => api.call('remove_object', { index: i }) }))));
    objList.replaceChildren(...(rows.length ? rows : [ui.note('No objects: add one below, or pick a scene above.')]));
  }

  drawSize();
  cancelBtn.hidden = true;
  return {
    update(state) {
      s = state;
      robot.setOptions(s.robots.map(r => ({ value: r.name, label: r.name + (r.ok ? (r.gripper ? '' : '  (no gripper)') : '  — ' + r.reason) })), s.robot);
      preset.setOptions(s.presets.map(p => ({ value: p.key, label: p.title })), s.preset);
      const busy = s.running || s.starting;
      robot.disabled = busy; preset.disabled = busy; table.input.disabled = busy;
      table.set(s.table);
      robotLike.set(s.robot_like); robotLike.input.disabled = busy;
      startBtn.disabled = busy || !s.robot; stopBtn.disabled = !busy;
      color.setOptions(s.colors, color.value);
      saveBtn.disabled = busy;
      robotNote.set(s.robot && !s.gripper ? s.robot + ' has no gripper in simulation: picking is not available.' : '', 'warn');
      status.set(s.starting ? 'Starting the simulator and the FIC controller…'
        : s.running ? 'Running: move the arm with the Joint, Cart and Gripper panels in the bar at the bottom.'
        : s.sim_running ? 'Another simulation is running: Start replaces it with the Playground.' : 'Stopped', s.running ? '' : (s.sim_running ? 'warn' : ''));
      const sig = JSON.stringify([s.objects, busy]);
      if (sig !== objSig && !objList.contains(document.activeElement)) { objSig = sig; drawObjects(); }
      const p = JSON.stringify([s.poses, s.can_pick]);
      if (p !== poseSig) {
        poseSig = p;
        const rows = Object.entries(s.poses).filter(([n]) => n !== 'table').map(([n, v]) => h('div', { class: 'pg-row' },
          h('span', { class: 'pg-name', text: n }),
          h('span', { class: 'faint num', text: 'x ' + MM(v[0]) + '  y ' + MM(v[1]) + '  z ' + MM(v[2]) + ' mm' }),
          h('span', { class: 'pg-grow' }),
          s.can_pick ? ui.button({ label: 'Pick', icon: 'play', title: 'Build the pick as a sequence and play it',
            onClick: () => api.call('pick', { name: n }).then(r => { if (r) api.toast(r.message); }) }) : null));
        poses.replaceChildren(...(rows.length ? rows : [ui.note(s.running ? 'Waiting for the simulator…' : 'Start the Playground to see where the objects are.')]));
      }
      log.set(s.log);
    },
    dispose() {},
  };
}
