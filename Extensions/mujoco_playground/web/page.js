// MuJoCo Playground — web page (manifest "web"). The Studio side is web_state() /
// web_action() in ../extension.py; this file only draws, with the UI kit (api.ui).
const MM = v => (v * 1000).toFixed(0);
const CHUNK = 768 * 1024;                             // raw bytes per upload message (~1 MB as base64)

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
  const meshSel = ui.select({ options: [] });
  const units = ui.select({ options: ['m', 'cm', 'mm', 'in'], value: 'mm' });
  const mass = ui.number({ value: 0.05, min: 0.005, max: 5, step: 0.01, unit: 'kg', digits: 3 });
  const color = ui.select({ options: ['red'], value: 'red' });
  const x = ui.number({ value: 0.5, min: -1.2, max: 1.2, step: 0.01, unit: 'm', digits: 2 });
  const y = ui.number({ value: 0.0, min: -1.2, max: 1.2, step: 0.01, unit: 'm', digits: 2 });
  const sizeBox = h('span', { class: 'pg-size' });
  let sizeInputs = [];
  const saveBtn = ui.button({ label: 'Add object', kind: 'primary', onClick: () => submit() });
  const cancelBtn = ui.button({ label: 'Cancel', onClick: () => edit(null) });

  // ── Mesh files: dropped here or chosen, sent to the Studio in chunks, added to the scene ──
  const dzNote = h('div', { class: 'dz-note faint', 'aria-live': 'polite' });
  const pick = h('input', { type: 'file', accept: '.stl,.obj', hidden: true });
  const dz = h('div', { class: 'dz pg-dz', role: 'group', 'aria-label': 'Add an object from a mesh file' },
    h('div', { class: 'dz-title', text: 'Drop an STL or OBJ file here' }),
    h('div', { class: 'dz-hint', text: 'It is added to the scene at a free spot; its units are guessed (CAD files are usually in mm), change them with Edit. In the Studio window.' }),
    h('div', { class: 'btn-row dz-btns' }, ui.button({ label: 'Choose a file…', onClick: () => pick.click() })), dzNote, pick);
  const b64 = blob => new Promise((ok, no) => { const r = new FileReader(); r.onload = () => ok(String(r.result).split(',')[1] || ''); r.onerror = no; r.readAsDataURL(blob); });
  async function upload(file) {
    if (!file) return;
    if (s && (s.running || s.starting)) { dzNote.textContent = 'Stop the Playground first, then add objects.'; return; }
    if (s && file.size > s.max_mesh_mb * 1048576) { dzNote.textContent = file.name + ' is over ' + s.max_mesh_mb + ' MB.'; return; }
    const begin = await api.call('mesh_upload', { op: 'begin', filename: file.name, size: file.size });
    if (!begin) { dzNote.textContent = ''; return; }
    try {
      for (let off = 0; off < file.size; off += CHUNK) {
        const part = file.slice(off, off + CHUNK);
        if (!await api.call('mesh_upload', { op: 'chunk', upload: begin.upload, offset: off, data: await b64(part) })) throw new Error('refused');
        dzNote.textContent = 'Sending ' + file.name + '… ' + Math.round(100 * Math.min(file.size, off + CHUNK) / file.size) + '%';
      }
    } catch (e) { dzNote.textContent = 'The upload stopped: drop the file again.'; api.call('mesh_upload', { op: 'cancel', upload: begin.upload }); return; }
    dzNote.textContent = 'Checking it in MuJoCo…';
    const r = await api.call('mesh_upload', { op: 'finish', upload: begin.upload });
    dzNote.textContent = r ? r.message : '';
  }
  const hasFiles = e => [...((e.dataTransfer && e.dataTransfer.types) || [])].includes('Files');
  let depth = 0;
  dz.addEventListener('dragenter', e => { if (hasFiles(e)) { e.preventDefault(); depth++; dz.classList.add('over'); } });
  dz.addEventListener('dragover', e => { if (hasFiles(e)) { e.preventDefault(); e.dataTransfer.dropEffect = 'copy'; } });
  dz.addEventListener('dragleave', () => { depth = Math.max(0, depth - 1); if (!depth) dz.classList.remove('over'); });
  dz.addEventListener('drop', e => { e.preventDefault(); depth = 0; dz.classList.remove('over'); upload((e.dataTransfer.files || [])[0]); });
  pick.addEventListener('change', () => { const f = pick.files[0]; pick.value = ''; upload(f); });
  // a file dropped beside the zone must not make the window open it
  const stray = e => { if (hasFiles(e) && !dz.contains(e.target)) e.preventDefault(); };
  window.addEventListener('dragover', stray); window.addEventListener('drop', stray);

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
      ui.section('Objects', { aside: '' }, table, objList, dz,
        h('div', { class: 'pg-form' },
          ui.field('Name', name), ui.field('Shape', shape), ui.field('Size', sizeBox), ui.field('Mass', mass),
          ui.field('Position', h('span', { class: 'pg-xy' }, h('span', { class: 'faint', text: 'x' }), x, h('span', { class: 'faint', text: 'y' }), y)),
          ui.field('Colour', color), ui.buttons(saveBtn, cancelBtn))),
      ui.section('Where things are', poses, h('h3', { class: 'pg-h', text: 'Try this' }), learn)),
    ui.section('Log', log)));

  function drawSize(values) {
    if (shape.value === 'mesh') { sizeInputs = []; sizeBox.replaceChildren(meshSel, h('span', { class: 'faint', text: 'in' }), units); return; }
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
    mass.set(o && o.mass ? o.mass : 0.05);
    if (o && o.shape === 'mesh') { meshSel.value = o.mesh; units.value = o.units; }
    drawSize(o ? o.size : null);
    saveBtn.set({ label: i == null ? 'Add object' : 'Save object' });
    cancelBtn.hidden = i == null;
  }

  function submit() {
    const object = { name: name.value, shape: shape.value, color: color.value, x: num(x), y: num(y), mass: num(mass), size: sizeInputs.map(num) };
    if (shape.value === 'mesh') Object.assign(object, { mesh: meshSel.value, units: units.value });
    const p = editing == null ? api.call('add_object', { object }) : api.call('update_object', { index: editing, object });
    p.then(r => { if (r) edit(null); });
  }

  function drawObjects() {
    const busy = s.running || s.starting;
    const rows = s.objects.map((o, i) => h('div', { class: 'pg-row' },
      h('span', { class: 'pg-dot', style: 'background:' + o.color }),
      h('span', { class: 'pg-name', text: o.name }),
      h('span', { class: 'faint', text: (o.shape === 'mesh' ? o.mesh : o.shape) + ' · ' + o.size.map(MM).join('×') + ' mm · ' + (o.mass || 0.05) + ' kg · at ' + o.x.toFixed(2) + ', ' + o.y.toFixed(2) + ' m' }),
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
      shape.setOptions(['box', 'sphere', 'cylinder'].concat(s.meshes.length ? ['mesh'] : []), shape.value);
      meshSel.setOptions(s.meshes, meshSel.value || s.meshes[0]);
      units.setOptions(s.units, units.value);
      saveBtn.disabled = busy;
      dz.classList.toggle('pg-off', busy);
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
    dispose() { window.removeEventListener('dragover', stray); window.removeEventListener('drop', stray); },
  };
}
