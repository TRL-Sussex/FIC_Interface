/* Robot Configuration: every robot the Studio knows, and the selected one over
   five tabs (Overview, Joints, Frames, Hardware, Safety), laid out like the
   classic page. A robot is added blank or from its URDF (a path on the Studio
   machine); one Save writes the Studio's robot list and the controllers' copy.
   The Studio validates and merges; a save changes limits and safety zones, so
   it is reviewed and confirmed first. */
(() => {
'use strict';
const { $, h, fmt, setText, section: plainSection, csection, csectionTools, pageTitle, cmd, toast, inlineConfirm } = window.FIC;
const TABS = [['overview', 'Overview'], ['joints', 'Joints'], ['frames', 'Frames'], ['hardware', 'Hardware'], ['safety', 'Safety']];
const QN = ['qx', 'qy', 'qz', 'qw'];

const section = (title, aside, ...b) => title === 'Ready to simulate?' ? plainSection(title, aside, ...b) : csection('robots/' + title, title, aside, ...b);
window.FIC.pages.robot_config = function buildRobots(root) {
  const st = { items: [], current: null, cfg: null, orig: null, status: null, tab: 'overview', sync: false, hasReport: false, modelDir: '' };
  const list = h('div', { class: 'rl' }), askHost = h('div'), err = h('div', { class: 'bad-line' });
  const detail = h('div', { class: 'rd' }), head = h('div', { class: 'rd-head' }), tabsEl = h('div', { class: 'seg', role: 'tablist' }), pane = h('div', { class: 'rd-pane' });
  const dzNote = h('div', { class: 'dz-note faint', 'aria-live': 'polite' });
  const pickFiles = h('input', { type: 'file', multiple: true, hidden: true, accept: '.urdf,.xacro,.xml,.zip,.dae,.stl,.obj,.srdf,.yaml,.yml,.png,.jpg' });
  const pickDir = h('input', { type: 'file', hidden: true, webkitdirectory: true });
  const dropZone = h('div', { class: 'dz', id: 'rc-drop', role: 'group', 'aria-label': 'Add a robot by dropping files' },
    h('div', { class: 'dz-title', text: 'Add a robot' }),
    h('div', { class: 'dz-hint', text: 'Drop a URDF, a .xacro, a description-package folder or a .zip anywhere on this page (up to 200 MB in all).' }),
    h('div', { class: 'btn-row dz-btns' }, h('button', { class: 'btn', type: 'button', text: 'Choose files', onclick: () => pickFiles.click() }),
      h('button', { class: 'btn', type: 'button', text: 'Choose folder', onclick: () => pickDir.click() })), dzNote, pickFiles, pickDir);
  const importBox = h('div'), upState = { id: null }, pathIn = h('input', { class: 'input', type: 'text', placeholder: '/path/to/robot.urdf, its folder or zip', 'aria-label': 'URDF path', style: 'min-width:16rem' });
  const MAX_UPLOAD = 200 * 1024 * 1024, CHUNK = 768 * 1024;
  root.append(...pageTitle('Robot Configuration', 'Every robot the Studio can run, and what it needs.'), err, askHost,
    h('div', { class: 'rc-body' },
      h('div', { class: 'rc-left' }, h('div', { class: 'eyebrow', text: 'Robots' }), list,
        dropZone,
        h('div', { class: 'field', style: 'margin-top:var(--s4)' }, h('label', { text: 'Or type a path on the Studio machine' }), pathIn),
        h('div', { class: 'btn-row', style: 'margin-top:var(--s2)' }, h('button', { class: 'btn btn-primary', type: 'button', text: 'Read URDF', onclick: () => analyse() }),
          h('button', { class: 'btn', type: 'button', text: '+ Blank robot', title: 'Add a robot by typing its settings in, without a URDF', onclick: addBlank })),
        importBox),
      detail));
  detail.append(head, tabsEl, pane);
  window.FIC.onSave = () => review();

  const dirtyKeys = () => st.cfg ? Object.keys(st.cfg).filter(k => JSON.stringify(st.cfg[k]) !== JSON.stringify(st.orig[k])) : [];

  async function loadList(keep) {
    const r = await cmd('robots_list', {}, true);
    if (!r.ok) { setText($('#lede'), r.message || 'Not available'); return; }
    st.items = r.items; st.sync = r.sync.stale;
    if (!keep || !st.items.find(x => x.id === st.current)) st.current = (st.items.find(x => x.id === r.current) || st.items[0] || {}).id;
    drawList();
    if (st.current) await loadRobot(st.current);
  }
  function drawList() {
    list.textContent = '';
    for (const it of st.items) list.append(h('button', { class: 'rl-row', type: 'button', 'aria-pressed': String(it.id === st.current), onclick: () => select(it.id) },
      h('span', { class: 'dot ' + (it.sim_ready ? 'ok' : 'warn') }), h('span', { class: 'rl-n' }, h('b', { text: it.name }), h('span', { class: 'faint', text: it.id + ' · ' + it.joints + ' joints' + (it.supported ? ' · hardware' : '') }))));
  }
  function select(id) {
    if (id === st.current) return;
    if (dirtyKeys().length) { inlineConfirm(askHost, { title: 'Discard unsaved changes to ' + st.current + '?', lines: dirtyKeys(), label: 'Discard', danger: true, onConfirm: () => { st.current = id; drawList(); loadRobot(id); } }); return; }
    st.current = id; drawList(); loadRobot(id);
  }
  async function loadRobot(id) {
    const r = await cmd('robots_get', { id }, true);
    if (!r.ok) { setText(err, r.message || ''); return; }
    st.cfg = JSON.parse(JSON.stringify(r.config)); st.orig = JSON.parse(JSON.stringify(r.config)); st.status = r.status; st.hasReport = r.has_report; st.modelDir = r.model_dir; st.viewer = r.viewer;
    setText(err, ''); drawHead(); drawTabs(); drawPane();
  }

  // ── header ──
  function drawHead() {
    head.textContent = '';
    const dirty = dirtyKeys().length > 0, ok = st.status && st.status.sim_ready;
    head.append(h('div', { class: 'rd-names' }, h('h2', { class: 'rd-title', text: st.cfg.display_name || st.current }),
        h('div', { class: 'faint' }, st.current, '  ', h('span', { class: ok ? 'ok-t' : 'warn-t', text: ok ? 'Simulation ready' : 'Needs attention' }), '  ', h('span', { class: 'faint', text: st.cfg.is_supported ? 'Hardware supported' : 'Simulation only' }))),
      h('span', { class: 'spacer' }),
      h('span', { class: dirty ? 'warn-t' : st.sync ? 'warn-t' : 'faint', style: 'font-size:var(--fs-m)', text: dirty ? '● Unsaved changes' : st.sync ? '● Controllers’ copy is older: Save to update' : '✓ Saved' }),
      h('button', { class: 'btn btn-primary', type: 'button', text: 'Save', title: 'Save this robot: the Studio’s list and the controllers’ robots_config.yaml together', onclick: review }),
      moreMenu());
  }
  function moreMenu() {
    const pop = h('div', { class: 'menu-pop', hidden: true, style: 'right:0;left:auto' });
    const item = (label, fn) => h('button', { class: 'mi', type: 'button', onclick: () => { pop.hidden = true; fn(); } }, h('span', { class: 'mi-check' }), h('span', { text: label }));
    pop.append(item('Duplicate', duplicate), item('Delete…', del), h('div', { class: 'mi-sep' }),
      item('Import robots from YAML…', importYaml), item('Export all robots to YAML…', async () => { const r = await cmd('export', { what: 'robots' }); if (r.ok) download(r.filename, r.mime, r.text); }),
      item('View the controllers’ YAML', async () => { const r = await cmd('robots_yaml', { id: st.current }); if (r.ok) showText('Controllers’ YAML · ' + st.current, r.path, r.text); }));
    const btn = h('button', { class: 'btn', type: 'button', text: '⋯', 'aria-label': 'More', onclick: () => { pop.hidden = !pop.hidden; } });
    return h('span', { class: 'menu', style: 'position:relative' }, btn, pop);
  }
  function download(name, mime, text) { const url = URL.createObjectURL(new Blob([text], { type: mime })); const a = h('a', { href: url, download: name }); document.body.append(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(url), 2000); }
  function showText(title, sub, text) {
    inlineConfirm(askHost, { title, lines: [sub].concat(text.split('\n').slice(0, 60)), label: 'Close', onConfirm: () => {} });
  }
  function drawTabs() { window.FIC.setSeg(tabsEl, TABS.map(([v, l]) => ({ value: v, label: l })), st.tab, v => { st.tab = v; drawTabs(); drawPane(); }); }

  // ── small form builders; each edit goes straight into st.cfg ──
  const get = (path) => path.split('.').reduce((o, k) => (o == null ? o : o[k]), st.cfg);
  function put(path, v) {
    const ks = path.split('.'); let o = st.cfg;
    for (const k of ks.slice(0, -1)) { if (o[k] == null || typeof o[k] !== 'object') o[k] = {}; o = o[k]; }
    o[ks[ks.length - 1]] = v; drawHead();
  }
  const text = (path, label, extra) => row(label, h('input', { class: 'input', type: 'text', value: get(path) == null ? '' : String(get(path)), oninput: e => put(path, e.target.value), 'aria-label': label }), extra);
  const number = (path, label, step, unit) => row(label, h('input', { class: 'input num', type: 'number', step: step || 'any', value: get(path) == null ? '' : String(get(path)), oninput: e => { if (e.target.value !== '') put(path, parseFloat(e.target.value)); }, 'aria-label': label }), unit);
  const toggle = (path, label) => h('div', { class: 'frow' }, h('label', { class: 'toggle' }, h('input', { type: 'checkbox', checked: !!get(path), onchange: e => put(path, e.target.checked) }), h('span', { class: 'track' }), h('span', { text: label })));
  const row = (label, ctl, unit) => h('div', { class: 'frow' }, h('label', { text: label }), ctl, h('span', { class: 'unit faint', text: unit || '' }));
  const listField = (path, label, unit) => row(label, h('input', { class: 'input', type: 'text', value: (get(path) || []).join(', '), oninput: e => put(path, e.target.value.split(/[,\s]+/).filter(Boolean).map(Number)), 'aria-label': label }), unit);

  function drawPane() {
    pane.textContent = ''; if (!st.cfg) return;
    pane.append(csectionTools(pane));
    const c = st.cfg, n = c.num_joints || 0;
    if (st.tab === 'overview') {
      pane.append(section('Ready to simulate?', null,
        h('div', { class: 'rc-status' }, ...(st.status.messages.length ? st.status.messages.map(m => h('div', { class: m.level === 'ok' ? 'ok-t' : m.level === 'warn' ? 'warn-t' : 'bad-t', text: { ok: '✓', warn: '!', error: '✕' }[m.level] + '  ' + m.text })) : [h('div', { class: 'faint', text: 'No checks to show.' })])),
        h('div', { class: 'btn-row', style: 'margin-top:var(--s4)' },
          h('button', { class: 'btn btn-primary', type: 'button', icon: 'play', text: 'Try in simulation', disabled: !st.status.sim_ready, onclick: trySim }),
          h('button', { class: 'btn', type: 'button', text: 'Open in MuJoCo viewer', disabled: !st.viewer, onclick: async () => { const r = await cmd('robots_viewer', { id: st.current }); if (r.ok) toast('The viewer opens on the Studio machine'); } }),
          st.hasReport ? h('button', { class: 'btn', type: 'button', text: 'Import report', onclick: async () => { const r = await cmd('robots_report', { id: st.current }); if (r.ok) showText('Import report · ' + st.current, '', r.text); } }) : null)),
        section('Identity', null, h('div', { class: 'fgrid' }, text('display_name', 'Display name'), toggle('is_supported', 'Real robot control available'))),
        section('Files', null, h('div', { class: 'fgrid' }, text('urdf_path', 'URDF'), text('mujoco_xml_path', 'MuJoCo model'))));
    } else if (st.tab === 'joints') {
      const tbody = h('tbody');
      for (let i = 0; i < n; i++) tbody.append(h('tr', null,
        h('td', { text: 'J' + (i + 1) }),
        h('td', null, h('input', { class: 'input', type: 'text', value: (c.joint_names || [])[i] || '', oninput: e => { c.joint_names = c.joint_names.slice(); c.joint_names[i] = e.target.value; drawHead(); } })),
        ...[['joint_limits_deg.lower', 'lower'], ['joint_limits_deg.upper', 'upper'], ['joint_torque_limits.max', 'torque']].map(([p]) => h('td', null, h('input', { class: 'input num', type: 'number', step: 'any', value: String(((get(p) || [])[i]) ?? ''), oninput: e => { const a = (get(p) || []).slice(); a[i] = parseFloat(e.target.value); put(p, a); } }))),
        h('td', null, h('input', { class: 'input num', type: 'number', step: 'any', value: String(((c.q_home_deg || [])[i]) ?? 0), oninput: e => { const a = (c.q_home_deg || []).slice(); a[i] = parseFloat(e.target.value); put('q_home_deg', a); } }))));
      pane.append(section('Joints', null, h('div', { class: 'fgrid' }, row('Number of joints', h('input', { class: 'input num', type: 'number', min: '1', max: '40', value: String(n), onchange: e => resizeJoints(parseInt(e.target.value, 10)), 'aria-label': 'Number of joints' }))),
        h('table', { class: 'table left', style: 'margin-top:var(--s4)' }, h('thead', null, h('tr', null, ...['', 'name', 'lower °', 'upper °', 'torque max N·m', 'home °'].map(t => h('th', { text: t })))), tbody)));
    } else if (st.tab === 'frames') {
      pane.append(section('Frames', null, h('div', { class: 'fgrid' }, text('ee_frame_name', 'End-effector frame'), text('elbow_frame_name', 'Elbow frame'), text('kinematics_solver', 'Kinematics solver'))),
        section('Default Cartesian target', null, h('div', { class: 'fgrid' },
          ...['x', 'y', 'z'].map((a, i) => number('default_ee_target.position.' + i, 'Position ' + a, '0.001', 'm')),
          ...QN.map((a, i) => number('default_ee_target.orientation.' + i, 'Orientation ' + a, '0.0001'))),
          h('div', { class: 'btn-row', style: 'margin-top:var(--s4)' }, h('button', { class: 'btn', type: 'button', text: 'Set from home pose', title: 'The end effector’s pose at the home pose, from the URDF', onclick: fromHome }))));
    } else if (st.tab === 'hardware') {
      pane.append(section('Real robot', null, h('div', { class: 'fgrid' }, text('driver_command', 'Driver command'), text('build_directory', 'Build directory'), text('build_command', 'Build command')),
          h('div', { class: 'btn-row', style: 'margin-top:var(--s4)' }, h('button', { class: 'btn', type: 'button', text: 'Compile driver', onclick: () => inlineConfirm(askHost, { title: 'Build the ' + st.current + ' driver?', lines: [c.build_command || ''], label: 'Compile',
            onConfirm: async () => { const r = await cmd('robots_compile_driver', { id: st.current, confirm: true }); if (r.ok) toast('Build started; follow it under Settings, Developer'); } }) }))),
        section('Torque and wrench', null, h('div', { class: 'fgrid' },
          row('Torque sign', (() => { const s = h('select', { class: 'select' }, h('option', { value: '1', text: 'Positive (+1)' }), h('option', { value: '-1', text: 'Negative (-1)' })); s.value = String(c.torque_sign >= 0 ? 1 : -1); s.addEventListener('change', () => put('torque_sign', Number(s.value))); return s; })()),
          toggle('gravity_compensated', 'Joint torques are gravity-compensated'), toggle('wrench_gravity_compensated', 'Wrench is gravity-compensated'), toggle('wrench_from_api', 'Wrench from the driver API'))),
        section('Sensor deadbands', null, h('div', { class: 'fgrid' }, number('admittance_deadbands.joint_tau', 'Joint torque', 'any', 'N·m'), number('admittance_deadbands.cart_force', 'Cartesian force', 'any', 'N'), number('admittance_deadbands.cart_torque', 'Cartesian torque', 'any', 'N·m'))),
        section('Simulate like the real robot', null,
          h('p', { class: 'faint', style: 'font-size:var(--fs-m)' }, 'When on, the simulator gives the arm these joint friction, damping and rotor-inertia values and this torque-sensor noise and command delay, and measures torque as on the robot: filtered at the robot’s cutoff, H-PMP’s push estimated from the joint torques, the deadbands above. Tunes made in simulation then carry over. Off, or left empty, the simulator is ideal. Measure the values on the arm with fic_studio.hardware_model (fic_docs: Simulation vs. Hardware).'),
          h('div', { class: 'fgrid' }, toggle('hardware_model.enabled', 'Simulate like the real robot'),
            listField('hardware_model.joint_friction', 'Joint friction', 'N·m, per joint'), listField('hardware_model.joint_damping', 'Joint damping', 'N·m·s/rad, per joint'),
            listField('hardware_model.armature', 'Rotor inertia', 'kg·m², per joint'), listField('hardware_model.torque_noise', 'Torque noise', 'N·m, per joint'),
            number('hardware_model.torque_filter_hz', 'Measured-torque filter', 'any', 'Hz'), number('hardware_model.command_delay_ms', 'Command delay', 'any', 'ms'),
            text('hardware_model.source', 'Where the values come from'))),
        section('Gripper', null, h('div', { class: 'fgrid' }, toggle('has_gripper', 'Has a gripper'), number('gripper_range.0', 'Closed width', '0.001', 'm'), number('gripper_range.1', 'Open width', '0.001', 'm'))),
        h('p', { class: 'faint', style: 'font-size:var(--fs-m);margin-top:var(--s5)' }, 'The inner-loop MCU settings and the driver-library drop zone are set in the classic window.'));
    } else {
      const z = (c.safety_zones = c.safety_zones || {});
      const axes = ['x', 'y', 'z'].map(a => h('tr', null, h('td', { text: a.toUpperCase() }),
        ...['min', 'max'].map(k => h('td', null, h('input', { class: 'input num', type: 'number', step: '0.01', value: String((((z.cartesian || {})[a]) || {})[k] ?? ''), oninput: e => put('safety_zones.cartesian.' + a + '.' + k, parseFloat(e.target.value)) }))),
        h('td', null, h('label', { class: 'toggle' }, h('input', { type: 'checkbox', checked: !!(((z.cartesian || {})[a]) || {}).enabled, onchange: e => put('safety_zones.cartesian.' + a + '.enabled', e.target.checked) }), h('span', { class: 'track' })))));
      const js = z.joint_safety_deg || { lower: [], upper: [], enabled: false };
      const jl = h('textarea', { class: 'xmlarea', rows: String(Math.max(3, n)), spellcheck: 'false', 'aria-label': 'Joint safety limits' }); jl.value = (js.lower || []).map((lo, i) => lo + ', ' + (js.upper || [])[i]).join('\n');
      jl.addEventListener('input', () => { const rows = jl.value.split('\n').map(l => l.split(',').map(x => parseFloat(x))).filter(r => r.length >= 2 && !isNaN(r[0]) && !isNaN(r[1])); put('safety_zones.joint_safety_deg.lower', rows.map(r => r[0])); put('safety_zones.joint_safety_deg.upper', rows.map(r => r[1])); });
      pane.append(section('Workspace box', null, h('table', { class: 'table left' }, h('thead', null, h('tr', null, ...['axis', 'min m', 'max m', 'on'].map(t => h('th', { text: t })))), h('tbody', null, ...axes))),
        section('Joint safety limits', null, h('p', { class: 'faint', style: 'font-size:var(--fs-m)' }, 'One line per joint: lower, upper (degrees).'), jl, h('div', { class: 'frow', style: 'margin-top:var(--s3)' }, toggle('safety_zones.joint_safety_deg.enabled', 'Enforce joint safety limits'))),
        section('Speed limits', null, h('div', { class: 'fgrid' }, number('safety_zones.velocity.max_joint_vel', 'Max joint speed', 'any', 'rad/s'), number('safety_zones.velocity.max_cart_vel', 'Max Cartesian speed', 'any', 'm/s'), toggle('safety_zones.velocity.enabled', 'Enforce speed limits'))));
    }
  }
  function resizeJoints(n) {
    if (!(n >= 1 && n <= 40)) return;
    const c = st.cfg, fit = (a, v) => { a = (a || []).slice(0, n); while (a.length < n) a.push(v); return a; };
    c.num_joints = n; c.joint_names = fit(c.joint_names, '').map((x, i) => x || 'joint' + (i + 1)); c.q_home_deg = fit(c.q_home_deg, 0);
    c.joint_limits_deg = { lower: fit((c.joint_limits_deg || {}).lower, -180), upper: fit((c.joint_limits_deg || {}).upper, 180) };
    c.joint_torque_limits = { max: fit((c.joint_torque_limits || {}).max, 50) };
    drawHead(); drawPane();
  }
  async function fromHome() {
    const r = await cmd('robots_target_from_home', { id: st.current, joint_names: st.cfg.joint_names, q_home_deg: st.cfg.q_home_deg, ee_frame_name: st.cfg.ee_frame_name, urdf_path: st.cfg.urdf_path, mujoco_xml_path: st.cfg.mujoco_xml_path });
    if (r.ok) { put('default_ee_target', { position: r.position, orientation: r.orientation }); drawPane(); toast('Target set from the home pose'); }
  }

  // ── actions ──
  function review() {
    const keys = dirtyKeys();
    if (!keys.length) { toast('Nothing changed'); return; }
    const lines = keys.map(k => k + ': ' + short(st.orig[k]) + '  ->  ' + short(st.cfg[k]));
    inlineConfirm(askHost, { title: 'Save ' + st.current + '?', lines: lines.concat(['Writes the Studio’s robot list and the controllers’ copy. Limits and safety zones take effect the next time a controller starts.']), label: 'Save robot',
      onConfirm: async () => {
        const patch = {}; for (const k of keys) patch[k] = st.cfg[k];
        const r = await cmd('robots_save', { id: st.current, config: patch, confirm: true }, true);
        if (r.ok) { toast('Saved ' + st.current); setText(err, ''); loadList(true); } else setText(err, r.message || 'The Studio refused the save. Nothing was written.');
      } });
  }
  const short = v => { const t = JSON.stringify(v); return t && t.length > 70 ? t.slice(0, 67) + '...' : t; };
  async function trySim() {
    inlineConfirm(askHost, { title: 'Start the simulator and controller for ' + st.current + '?', lines: ['The Studio switches to this robot and starts the simulation, then the controller.'], label: 'Try in simulation',
      onConfirm: async () => { const a = await cmd('set_robot', { name: st.current }); if (a.ok === false) return; await cmd('session', { op: 'start', confirm: true }); toast('Starting the simulation for ' + st.current); } });
  }
  function addBlank() {
    const id = h('input', { class: 'input', type: 'text', placeholder: 'MyRobot_7DOF', 'aria-label': 'Robot ID' });
    const box = h('div', { class: 'btn-row', style: 'margin-top:var(--s3)' }, h('div', { class: 'field' }, h('label', { text: 'Robot ID (letters, digits, _)' }), id),
      h('button', { class: 'btn btn-primary', type: 'button', text: 'Add', onclick: async () => { const r = await cmd('robots_add', { id: id.value.trim() }); if (r.ok) { importBox.textContent = ''; st.current = r.id; loadList(true); toast('Added ' + r.id); } } }));
    importBox.replaceChildren(box); id.focus();
  }
  function duplicate() { cmd('robots_duplicate', { id: st.current }).then(r => { if (r.ok) { st.current = r.id; loadList(true); toast('Duplicated as ' + r.id); } }); }
  function del() {
    inlineConfirm(askHost, { title: 'Delete ‘' + st.current + '’ from the robot list?', lines: ['This cannot be undone. Its model folder is left in place.'], label: 'Delete', danger: true,
      onConfirm: async () => { const r = await cmd('robots_delete', { id: st.current, confirm: true }); if (r.ok) { st.current = null; loadList(false); toast('Deleted'); } } });
  }
  function importYaml() {
    const input = h('input', { type: 'file', accept: '.yaml,.yml,text/yaml' });
    input.addEventListener('change', async () => {
      const f = input.files[0]; if (!f) return; const text = await f.text();
      inlineConfirm(askHost, { title: 'Import robots from ' + f.name + '?', lines: ['Robots with an ID that already exists are skipped.'], label: 'Import',
        onConfirm: async () => { const r = await cmd('robots_import_yaml', { text, confirm: true }); if (r.ok) { toast('Imported ' + r.added.length + ', skipped ' + r.skipped.length); loadList(true); } } });
    });
    input.click();
  }
  async function analyse(src) {
    if (!src || !src.upload) { const typed = pathIn.value.trim(); if (!typed) return; src = { path: typed }; await dropCancel(); }
    const r = await cmd('robots_import', Object.assign({ op: 'analyse' }, src));
    if (!r.ok) { await dropCancel(); return; }
    const p = r.plan, idIn = h('input', { class: 'input', type: 'text', value: p.id, 'aria-label': 'Robot ID' }), nameIn = h('input', { class: 'input', type: 'text', value: p.name, 'aria-label': 'Display name' });
    importBox.replaceChildren(h('div', { class: 'imp' },
      h('div', { class: 'eyebrow', text: 'Found in ' + (src.upload ? 'Dropped: ' : 'Found in ') + p.urdf.split('/').pop() }),
      ...p.found.map(f => h('div', { class: 'imp-row' }, h('b', { text: f.what }), h('span', { text: ' ' + f.value }), h('span', { class: 'faint', text: ' · ' + f.source }))),
      ...p.messages.map(m => h('div', { class: m.level === 'error' ? 'bad-t' : m.level === 'warn' ? 'warn-t' : 'faint', text: m.text })),
      h('div', { class: 'fgrid', style: 'margin-top:var(--s3)' }, row('Robot ID', idIn), row('Display name', nameIn)),
      h('div', { class: 'btn-row', style: 'margin-top:var(--s3)' },
        h('button', { class: 'btn btn-primary', type: 'button', text: 'Create robot', disabled: p.errors.length > 0, onclick: () => inlineConfirm(askHost, { title: 'Create ' + idIn.value + ' from the URDF?', lines: ['Writes menagerie/model/' + idIn.value + ' (URDF, MuJoCo model, report) and adds the robot to the list.', 'The simulator runs briefly to check the model, so this takes a little while.'], label: 'Create robot',
          onConfirm: async () => { const c = await cmd('robots_import', Object.assign({ op: 'create', robot_id: idIn.value.trim(), name: nameIn.value.trim(), confirm: true }, src)); if (c.ok) { upState.id = null; pollImport(); } } }) }),
        h('button', { class: 'btn', type: 'button', text: 'Cancel', onclick: () => { importBox.replaceChildren(); dropCancel(); } }))));
  }
  // ── drag and drop: the tree goes to the Studio in chunks, then the same review as a typed path ──
  async function dropCancel() { const id = upState.id; upState.id = null; if (id) await cmd('robots_upload', { op: 'cancel', upload: id }, true); }
  const rel = (p) => String(p).replace(/^\/+/, '');
  function readEntries(dirEntry) {
    const reader = dirEntry.createReader(), all = [];
    return new Promise((resolve, reject) => { const next = () => reader.readEntries(batch => { if (!batch.length) resolve(all); else { all.push(...batch); next(); } }, reject); next(); });
  }
  async function walkEntry(entry, out) {
    if (entry.isFile) { const file = await new Promise((ok, no) => entry.file(ok, no)); out.push({ path: rel(entry.fullPath), file }); }
    else if (entry.isDirectory) for (const child of await readEntries(entry)) await walkEntry(child, out);
  }
  /** Everything in a drop, folders walked with webkitGetAsEntry (relative paths kept); plain files as a fallback. */
  async function collectDrop(dt) {
    const entries = [...(dt.items || [])].map(i => (i.kind === 'file' && i.webkitGetAsEntry) ? i.webkitGetAsEntry() : null).filter(Boolean);   // before any await: the items die with the event
    const out = [];
    if (entries.length) for (const e of entries) await walkEntry(e, out);
    else for (const f of dt.files || []) out.push({ path: f.name, file: f });
    return out;
  }
  const collectPicked = (input) => [...input.files].map(f => ({ path: rel(f.webkitRelativePath || f.name), file: f }));
  const b64 = (blob) => new Promise((ok, no) => { const r = new FileReader(); r.onload = () => ok(String(r.result).split(',')[1] || ''); r.onerror = no; r.readAsDataURL(blob); });
  async function uploadAndReview(items) {
    if (!items.length) { setText(dzNote, 'Nothing to add: the drop had no files.'); return; }
    const total = items.reduce((a, x) => a + x.file.size, 0);
    if (total > MAX_UPLOAD) { setText(dzNote, 'That is ' + Math.round(total / 1048576) + ' MB; the limit is ' + (MAX_UPLOAD / 1048576) + ' MB. Drop the description package without large extras.'); return; }
    await dropCancel(); importBox.replaceChildren();
    const begin = await cmd('robots_upload', { op: 'begin' });
    if (!begin.ok) { setText(dzNote, begin.message || 'The Studio did not accept the upload.'); return; }
    upState.id = begin.upload;
    let sent = 0;
    try {
      for (const { path, file } of items) {
        let off = 0;
        do {
          const part = file.slice(off, off + CHUNK);
          const r = await cmd('robots_upload', { op: 'chunk', upload: upState.id, path, offset: off, data: await b64(part) }, true);
          if (!r.ok) throw new Error(r.message || 'upload refused');
          off += part.size; sent += part.size;
          setText(dzNote, 'Sending ' + items.length + (items.length === 1 ? ' file' : ' files') + '… ' + Math.round(100 * sent / Math.max(total, 1)) + '%');
        } while (off < file.size);
      }
    } catch (e) { setText(dzNote, 'Upload failed: ' + e.message); await dropCancel(); return; }
    setText(dzNote, items.length + (items.length === 1 ? ' file' : ' files') + ' received.');
    await analyse({ upload: upState.id });
  }
  const dropOk = (e) => [...((e.dataTransfer && e.dataTransfer.types) || [])].includes('Files');
  let depth = 0;
  dropZone.addEventListener('dragenter', e => { if (dropOk(e)) { e.preventDefault(); depth++; dropZone.classList.add('over'); } });
  dropZone.addEventListener('dragover', e => { if (dropOk(e)) { e.preventDefault(); e.dataTransfer.dropEffect = 'copy'; dropZone.classList.add('over'); } });
  dropZone.addEventListener('dragleave', () => { depth = Math.max(0, depth - 1); if (!depth) dropZone.classList.remove('over'); });
  async function onDrop(e) {
    e.preventDefault(); depth = 0; dropZone.classList.remove('over');
    setText(dzNote, 'Reading what was dropped…');
    try { await uploadAndReview(await collectDrop(e.dataTransfer)); } catch (err) { setText(dzNote, 'Could not read the drop: ' + err.message); }
  }
  dropZone.addEventListener('drop', onDrop);
  // files dropped anywhere else on the page are taken too (the window must not open them); the zone lights up
  let glow = 0;
  const stray = e => {
    if (!dropOk(e) || dropZone.contains(e.target)) return;
    e.preventDefault();
    if (e.type === 'dragover') { e.dataTransfer.dropEffect = 'copy'; dropZone.classList.add('over'); clearTimeout(glow); glow = setTimeout(() => dropZone.classList.remove('over'), 300); return; }
    onDrop(e);
  };
  window.addEventListener('dragover', stray); window.addEventListener('drop', stray);
  for (const input of [pickFiles, pickDir]) input.addEventListener('change', () => { const items = collectPicked(input); input.value = ''; uploadAndReview(items); });
  let timer = null;
  function pollImport() {
    clearInterval(timer);
    const note = h('div', { class: 'faint', text: 'Importing…' }); importBox.replaceChildren(note);
    timer = setInterval(async () => {
      const r = await cmd('robots_import', { op: 'status' }, true);
      if (!r.ok) return;
      setText(note, r.error ? 'Failed: ' + r.error : r.running ? (r.progress || 'Working') + '…' : 'Done');
      if (!r.running) { clearInterval(timer); if (r.done) { st.current = r.done.id; importBox.replaceChildren(); toast('Added ' + r.done.id); loadList(true); } }
    }, 1000);
  }
  loadList(false);
  return { update() {}, dispose() { clearInterval(timer); window.removeEventListener('dragover', stray); window.removeEventListener('drop', stray); dropCancel(); } };
};
})();
