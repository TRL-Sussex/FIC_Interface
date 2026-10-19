/* Safety page: the zones, joint limits and velocity limits the controllers enforce, and where the
   robot is against them right now.
   Editing is two steps on purpose: change values, press Review, read the before / after list, press
   Apply. The Studio checks the numbers again and refuses anything unsafe or malformed (`safety_set`
   needs confirm: true), then keeps what was applied in the robot's configuration.
   The 3D view is where zones are made: its toolbar places a box, a sphere or a cylinder where you click
   the floor, or draws an outline corner by corner; a selected zone shows handles to move, lift, resize
   and turn it. The same toolbar sets a workspace limit where the tool is (Table here, Wall here). Every
   zone is kept in (the arm stays inside) or kept out (the arm stays out), and the robot's controller
   checks it along the whole arm. Everything changes only the draft until Review and Apply. */
(() => {
'use strict';
const { $, h, fmt, setText, csection, csectionTools, pageTitle, cmd, toast, inlineConfirm } = window.FIC;
const STATE_LABEL = { ok: 'Inside', warn: 'Near the edge', violation: 'Outside', off: 'Not checked' };
const OVERALL = { ok: 'All inside', warn: 'Near a limit', violation: 'Limit exceeded', off: 'Monitor off' };
const DOT = { ok: 'ok', warn: 'warn', violation: 'bad', off: '' };

const section = (title, aside, ...b) => csection('safety/' + title, title, aside, ...b);
const AXES = ['x', 'y', 'z'];
const SPAN = { x: [-1, 1], y: [-1, 1], z: [0, 1.5] };   // where an unchecked axis is drawn
const MIN_GAP = 0.05;                                     // m kept between a workspace min and max
const FACES = [['x.max', 'X max'], ['x.min', 'X min'], ['y.max', 'Y max'], ['y.min', 'Y min'], ['z.max', 'Ceiling · Z max']];
const SHAPE_NAME = { box: 'Box', sphere: 'Sphere', cylinder: 'Cylinder', prism: 'Outline' };
const TOOLS = [['orbit', 'cursor', 'Select and turn: drag a zone’s handles, drag elsewhere to turn the view'],
  ['box', 'cube', 'Box: click the floor where it goes'], ['sphere', 'sphere', 'Sphere: click the floor where it goes'],
  ['cylinder', 'cylinder', 'Cylinder: click the floor where it goes'], ['draw', 'pen', 'Draw an outline: click the floor at each corner']];
const clone = o => JSON.parse(JSON.stringify(o));
const r3 = v => Math.round(v * 1000) / 1000;

window.FIC.pages.safety = function buildSafety(root) {
  const draft = new Map();          // path -> new value; only what the operator changed
  let zd = null, bd = null;         // the zones and the arm's size as drafted (null: as applied)
  let cur = null;                   // latest safety section
  let sel = -1, tool = 'orbit', newKeep = 'out', drawn = 0;
  const askHost = h('div', { id: 'saf-ask' });
  const stat = (id, k) => h('div', { class: 'stat' }, h('div', { class: 'k', text: k }), h('div', { class: 'v', id }), h('div', { class: 'n', id: id + '-n' }));
  const cartBody = h('tbody'), jointBody = h('tbody'), velBox = h('div', { class: 'saf-vel' }), switches = h('div', { class: 'rows' });
  const review = h('button', { class: 'btn btn-primary', type: 'button', text: 'Review changes', onclick: askApply });
  const discard = h('button', { class: 'btn', type: 'button', text: 'Discard', onclick: () => { draft.clear(); zd = bd = null; dismissAsk(); paint(true); setText(stageNote, 'Changes discarded.'); } });
  const bar = h('div', { class: 'btn-row sticky-actions', id: 'saf-bar', hidden: true }, review, discard, h('span', { class: 'faint', id: 'saf-n', style: 'font-size:var(--fs-m)' }));
  const err = h('div', { class: 'bad-line', id: 'saf-err' });
  const stage = h('div', { class: 'stage', id: 'saf-stage', 'aria-label': '3D view of the robot and its safety zones' });
  const stageNote = h('div', { class: 'faint', id: 'saf-stage-note', style: 'font-size:var(--fs-m);margin-top:8px', text: 'Loading the robot…' });

  // ── the toolbar on the 3D view ──
  const toolBtns = new Map();
  const iconBtn = (icon, title, onclick, extra) => h('button', Object.assign({ class: 'btn btn-icon', type: 'button', title, 'aria-label': title, onclick }, extra || {}), window.FIC.icon(icon));
  for (const [t, icon, title] of TOOLS) toolBtns.set(t, iconBtn(icon, title, () => setTool(t), { 'aria-pressed': String(t === tool) }));
  const keepSeg = h('div', { class: 'saf-keep', role: 'group', 'aria-label': 'New zones are' },
    ...[['out', 'Keep out'], ['in', 'Keep in']].map(([k, t]) => h('button', { type: 'button', 'data-keep': k, 'aria-pressed': String(k === newKeep), text: t,
      title: k === 'out' ? 'New zones keep the arm out of them' : 'New zones keep the arm inside them',
      onclick: () => { newKeep = k; for (const b of keepSeg.children) b.setAttribute('aria-pressed', String(b.dataset.keep === k)); } })));
  const binBtn = iconBtn('trash', 'Delete the selected zone (Review and Apply to remove it from the controllers)', () => deleteZone(sel), { disabled: true });
  const margin = h('input', { class: 'input num', type: 'number', step: '5', min: '0', max: '200', value: '20', id: 'saf-margin', 'aria-label': 'Margin inside a limit set where the tool is, mm', title: 'Margin inside a limit set where the tool is (mm)' });
  const facePick = h('select', { class: 'select', id: 'saf-face', 'aria-label': 'Which workspace limit Set here moves', title: 'Which workspace limit Set here moves' }, ...FACES.map(([v, t]) => h('option', { value: v, text: t })));
  const tools = h('div', { class: 'saf-tools', role: 'toolbar', 'aria-label': 'Safety zone tools' },
    ...toolBtns.values(), keepSeg, binBtn, h('span', { class: 'sep', 'aria-hidden': 'true' }),
    iconBtn('table', 'Table here: the workspace floor (Z min) where the tool is, the margin above it', () => teach('z.min')),
    iconBtn('wall', 'Wall here: the workspace side nearest the tool, where the tool is, the margin inside', () => teach('wall')),
    iconBtn('target', 'Set here: the workspace limit chosen next to it, where the tool is', () => teach(facePick.value)), facePick,
    h('label', { class: 'saf-mm', title: 'Margin inside a limit set where the tool is' }, margin, h('span', { class: 'faint', text: 'mm' })));
  const toolAt = h('div', { class: 'saf-at num', id: 'saf-tool', 'aria-live': 'off' });
  const drawBottom = h('input', { class: 'input num', type: 'number', step: '0.01', value: '0', 'aria-label': 'Outline bottom, m' });
  const drawTop = h('input', { class: 'input num', type: 'number', step: '0.01', value: '0.8', 'aria-label': 'Outline top, m' });
  const drawCount = h('span', { class: 'num' });
  const strip = h('div', { class: 'saf-strip', hidden: true });
  const touchErr = h('div', { class: 'bad-line', id: 'saf-touch-err' });
  stage.append(tools, toolAt, strip);

  const zoneList = h('div', { class: 'zlist', id: 'saf-zlist' });
  const zoneEdit = h('div', { class: 'zedit', id: 'saf-zedit' });
  const bodyBox = h('div', { class: 'zbody' });
  const zoneNow = h('div', { class: 'znow', id: 'saf-znow' });

  root.append(
    ...pageTitle('Safety', 'Zones and limits the controllers enforce. Changes need a second, explicit confirmation.'),
    h('div', { class: 'stats' }, stat('sf-over', 'State'), stat('sf-mon', 'Monitor'), stat('sf-enf', 'On violation'), stat('sf-thr', 'Warning band')),
    bar, err, askHost, csectionTools(root),
    section('Zones in 3D', { id: 'sf-3d-aside' }, h('div', { class: 'stage-wrap', style: 'margin-top:0' }, stage, stageNote, touchErr)),
    section('Safety zones (whole arm)', { id: 'sf-z-aside' },
      h('p', { class: 'faint', style: 'margin:0 0 var(--s4)', text: 'Every zone is checked along the arm, from the joint chosen below to the tool. Keep-in zones: the arm stays inside one of them. Keep-out zones: the arm stays out of every one. The controller pushes the arm back from a boundary and refuses targets that cross one; touching a zone or crossing it puts a warning in Notices.' }),
      zoneNow, zoneList, zoneEdit, bodyBox),
    section('Workspace box (tool point)', { id: 'sf-c-aside' }, h('table', { class: 'table safety' },
      h('thead', null, h('tr', null, ...['Axis', 'Min m', 'Max m', 'Checked', 'Position m', 'Where'].map(t => h('th', { text: t })))), cartBody)),
    section('Joint limits', { id: 'sf-j-aside' }, h('table', { class: 'table safety jlim' },
      h('thead', null, h('tr', null, ...['Joint', 'Range (mechanical end to end)', 'Lower °', 'Upper °', 'Position °', 'Where'].map(t => h('th', { text: t })))), jointBody),
      h('div', { style: 'margin-top:20px' }, toggle('joints.enabled', 'Check joint limits'))),
    section('Velocity limits', { id: 'sf-v-aside' }, velBox, h('div', { style: 'margin-top:20px' }, toggle('velocity.enabled', 'Limit velocity'))),
    section('Monitoring', null, switches));

  function dismissAsk() { askHost.textContent = ''; }
  function get(path, base) { return draft.has(path) ? draft.get(path) : base; }
  const zones = () => zd || (cur ? cur.zones : []) || [];
  const body = () => bd || (cur ? cur.body : null) || { radius: 0.05, tool_radius: 0.03, from_joint: 2 };
  const zonesEdited = () => !!zd && JSON.stringify(zd) !== JSON.stringify(cur ? cur.zones : []);
  const bodyEdited = () => !!bd && JSON.stringify(bd) !== JSON.stringify(cur ? cur.body : null);
  function mark() {
    const n = draft.size + (zonesEdited() ? 1 : 0) + (bodyEdited() ? 1 : 0);
    $('#saf-bar').hidden = n === 0;
    setText($('#saf-n'), n + (n === 1 ? ' change, not applied' : ' changes, not applied'));
    for (const el of root.querySelectorAll('[data-k]')) el.closest('tr, .kvcell, label, .vrow') && el.closest('tr, .kvcell, label, .vrow').classList.toggle('edited', draft.has(el.dataset.k));
  }
  function num(path, step, lo, hi, onSet) {
    const el = h('input', { class: 'input num', type: 'number', step, 'data-k': path, 'aria-label': path.replace(/\./g, ' ') });
    if (lo != null) el.min = lo; if (hi != null) el.max = hi;
    el.addEventListener('input', () => { if (el.value === '') return; draft.set(path, parseFloat(el.value)); mark(); onSet && onSet(); });
    return el;
  }
  function toggle(path, label) {
    const box = h('input', { type: 'checkbox', 'data-k': path });
    box.addEventListener('change', () => { draft.set(path, box.checked); mark(); });
    return h('label', { class: 'toggle' }, box, h('span', { class: 'track' }), h('span', { text: label }));
  }
  // A slider and a number for the same value: either one sets it.
  function slideNum({ label, unit, lo, hi, step, digits, get: read, set: write, title }) {
    const d = digits == null ? 3 : digits;
    const rng = h('input', { type: 'range', min: lo, max: hi, step, 'aria-label': label + ' slider' });
    const box = h('input', { class: 'input num', type: 'number', min: lo, max: hi, step, 'aria-label': label });
    rng.addEventListener('input', () => { box.value = Number(rng.value).toFixed(d); write(parseFloat(rng.value), false); });
    rng.addEventListener('change', () => write(parseFloat(rng.value), true));
    box.addEventListener('change', () => { const v = parseFloat(box.value); if (!isNaN(v)) { rng.value = v; write(v, true); } });
    const el = h('div', { class: 'vrow', title: title || '' }, h('span', { class: 'vrow-l', text: label + (unit ? ' · ' + unit : '') }), h('div', { class: 'xnum' }, rng, box));
    el.paint = () => { const v = read(); if (v == null) return; if (document.activeElement !== box) box.value = Number(v).toFixed(d); if (document.activeElement !== rng) rng.value = v; };
    return el;
  }

  // ── structure, built once per shape ──
  let shape = '';
  const jrefs = [];
  function build(s) {
    cartBody.textContent = ''; jointBody.textContent = ''; velBox.textContent = ''; switches.textContent = ''; jrefs.length = 0;
    for (const a of ['x', 'y', 'z']) {
      cartBody.append(h('tr', { 'data-row': 'c.' + a },
        h('td', { text: a.toUpperCase() }), h('td', null, num('c.' + a + '.min', '0.01', -s.bounds.cart_abs, s.bounds.cart_abs)),
        h('td', null, num('c.' + a + '.max', '0.01', -s.bounds.cart_abs, s.bounds.cart_abs)), h('td', null, toggle('c.' + a + '.enabled', '')),
        h('td', { class: 'live' }), h('td', { class: 'where' })));
    }
    s.joints.rows.forEach((r, i) => {
      const lo = r.mech_lower != null ? r.mech_lower : -180, hi = r.mech_upper != null ? r.mech_upper : 180;
      const a = h('input', { type: 'range', min: lo, max: hi, step: '0.5', 'aria-label': r.label + ' lower limit slider' });
      const b = h('input', { type: 'range', min: lo, max: hi, step: '0.5', 'aria-label': r.label + ' upper limit slider' });
      const fill = h('i', { class: 'r2-fill', 'aria-hidden': 'true' }), now = h('i', { class: 'jr-now', 'aria-hidden': 'true', style: 'display:none' });
      const lower = num('j.' + i + '.lower', '0.5', r.mech_lower, r.mech_upper, () => paintJoint(i)), upper = num('j.' + i + '.upper', '0.5', r.mech_lower, r.mech_upper, () => paintJoint(i));
      const slide = (which, input) => () => {
        let v = parseFloat(input.value);
        const other = which === 'lower' ? get('j.' + i + '.upper', cur.joints.rows[i].upper) : get('j.' + i + '.lower', cur.joints.rows[i].lower);
        if (other != null) v = which === 'lower' ? Math.min(v, other - 0.5) : Math.max(v, other + 0.5);
        input.value = v;
        draft.set('j.' + i + '.' + which, v); mark(); paintJoint(i);
      };
      a.addEventListener('input', slide('lower', a)); b.addEventListener('input', slide('upper', b));
      jrefs[i] = { a, b, fill, now, lower, upper, lo, hi };
      jointBody.append(h('tr', { 'data-row': 'j.' + i },
        h('td', { text: r.label }), h('td', { class: 'r2-cell' }, h('div', { class: 'range2' }, fill, a, b, now),
          h('div', { class: 'r2-ends faint num' }, h('span', { text: fmt(lo, 0) + '°' }), h('span', { text: fmt(hi, 0) + '°' }))),
        h('td', null, lower), h('td', null, upper), h('td', { class: 'live' }), h('td', { class: 'where' })));
    });
    for (const [k, label, unit, max, min, step] of [['max_joint_vel', 'Maximum joint velocity', 'rad/s', s.bounds.joint_vel, 0.01, 0.01], ['max_cart_vel', 'Maximum Cartesian velocity', 'm/s', s.bounds.cart_vel, 0.01, 0.01],
        ['joint_target_speed', 'Joint target speed (joint impedance; 0 = off)', 'rad/s', s.bounds.joint_vel, 0, 0.01]]) {
      const row = slideNum({ label, unit, lo: min, hi: max, step, digits: 2, get: () => get('v.' + k, cur.velocity[k]),
        set: (v) => { draft.set('v.' + k, v); mark(); } });
      row.querySelector('input.num').dataset.k = 'v.' + k;
      row.dataset.v = k;
      velBox.append(row);
    }
    switches.append(toggle('monitor', 'Monitor positions against the limits (warn and log)'),
      toggle('enforce', 'Stop the robot when it is clearly past a limit (' + fmt(s.stop_past.m * 1000, 0) + ' mm, or ' + fmt(s.stop_past.deg, 0) + '° for a joint)'));
    shape = s.joints.rows.length + ':' + s.bounds.cart_abs;
    buildBody(s);
  }
  function paintJoint(i) {
    const s = cur, j = jrefs[i]; if (!s || !j) return;
    const r = s.joints.rows[i];
    const lo = get('j.' + i + '.lower', r.lower), hi = get('j.' + i + '.upper', r.upper);
    const span = (j.hi - j.lo) || 1, pa = lo == null ? 0 : (lo - j.lo) / span, pb = hi == null ? 1 : (hi - j.lo) / span;
    if (document.activeElement !== j.a && lo != null) j.a.value = lo;
    if (document.activeElement !== j.b && hi != null) j.b.value = hi;
    j.fill.style.setProperty('--a', Math.max(0, Math.min(1, pa))); j.fill.style.setProperty('--b', Math.max(0, Math.min(1, pb)));
    j.now.style.display = r.value == null ? 'none' : '';
    if (r.value != null) j.now.style.setProperty('--p', Math.max(0, Math.min(1, (r.value - j.lo) / span)));
  }
  function buildBody(s) {
    bodyBox.textContent = '';
    const n = s.joint_count || s.joints.rows.length || 7;
    const from = h('select', { class: 'select', 'aria-label': 'First joint checked' }, ...Array.from({ length: n }, (_, k) => h('option', { value: k + 1, text: 'Joint ' + (k + 1) })));
    from.addEventListener('change', () => { bd = Object.assign(clone(body()), { from_joint: parseInt(from.value, 10) }); mark(); paintBody(); });
    const rad = slideNum({ label: 'Link radius', unit: 'mm', lo: 0, hi: 150, step: 1, digits: 0, get: () => body().radius * 1000,
      set: v => { bd = Object.assign(clone(body()), { radius: v / 1000 }); mark(); }, title: 'Each link of the arm is checked as a capsule this thick' });
    const tr = slideNum({ label: 'Tool radius', unit: 'mm', lo: 0, hi: 150, step: 1, digits: 0, get: () => body().tool_radius * 1000,
      set: v => { bd = Object.assign(clone(body()), { tool_radius: v / 1000 }); mark(); }, title: 'The tool point is checked as a ball this big' });
    bodyBox.append(h('div', { class: 'zbody-head' }, h('b', { text: 'The arm checked' }),
      h('label', { class: 'zbody-from' }, h('span', { class: 'faint', text: 'from' }), from, h('span', { class: 'faint', text: 'to the tool' }))), rad, tr);
    bodyBox.paint = () => { if (document.activeElement !== from) from.value = String(body().from_joint); rad.paint(); tr.paint(); };
  }
  function paintBody() { if (bodyBox.paint) bodyBox.paint(); }

  // ── the zone list and the selected zone's values ──
  let listSig = '', editSig = '';
  function setZones(list, keepSel) {
    zd = list;
    if (!keepSel && sel >= list.length) sel = list.length - 1;
    mark(); paintZones(); draw();
  }
  function updateZone(i, z) { const list = clone(zones()); list[i] = z; setZones(list, true); }
  function selectZone(i) { sel = i; paintZones(true); draw(); }
  function deleteZone(i) {
    const list = zones(), z = list[i];
    if (!z) return;
    const l = clone(list); l.splice(i, 1);
    if (sel === i) sel = -1; else if (sel > i) sel -= 1;
    setZones(l);
    setText(stageNote, z.name + ' deleted · not applied yet: Review changes (Discard brings it back)');
  }
  function zoneColour(z) { return z.enabled === false ? 'var(--ink-faint)' : z.keep === 'in' ? 'var(--ok, #2f7d4f)' : 'var(--bad)'; }
  function paintZones(force) {
    const list = zones(), zn = cur && cur.zone_now;
    const sig = JSON.stringify(list.map(z => [z.name, z.shape, z.keep, z.enabled])) + sel;
    if (force || sig !== listSig) {
      listSig = sig;
      zoneList.textContent = '';
      if (!list.length) zoneList.append(h('p', { class: 'faint', text: 'No safety zones. Use the box, sphere, cylinder or outline tool on the 3D view to add one.' }));
      list.forEach((z, i) => {
        const name = h('input', { class: 'input', type: 'text', value: z.name || '', maxlength: 40, 'aria-label': 'Zone ' + (i + 1) + ' name' });
        name.addEventListener('change', () => { const c = clone(z); c.name = name.value.trim() || SHAPE_NAME[z.shape] + ' ' + (i + 1); updateZone(i, c); });
        const keep = h('div', { class: 'saf-keep', role: 'group', 'aria-label': 'Zone ' + (i + 1) + ' keeps the arm' },
          ...[['out', 'Keep out'], ['in', 'Keep in']].map(([k, t]) => h('button', { type: 'button', 'aria-pressed': String((z.keep || 'out') === k), text: t,
            onclick: (e) => { e.stopPropagation(); const c = clone(z); c.keep = k; updateZone(i, c); } })));
        const on = h('input', { type: 'checkbox', checked: z.enabled !== false, 'aria-label': 'Zone ' + (i + 1) + ' on' });
        on.addEventListener('change', () => { const c = clone(z); c.enabled = on.checked; updateZone(i, c); });
        const del = h('button', { class: 'btn btn-icon', type: 'button', title: 'Remove this zone', 'aria-label': 'Remove zone ' + (i + 1),
          onclick: (e) => { e.stopPropagation(); deleteZone(i); } }, window.FIC.icon('trash'));
        const row = h('div', { class: 'zrow' + (i === sel ? ' sel' : ''), 'data-z': i, onclick: (e) => { if (e.target.closest('input, button, label')) return; selectZone(i === sel ? -1 : i); } },
          h('span', { class: 'zchip', style: 'background:' + zoneColour(z) }), name, h('span', { class: 'faint zshape', text: SHAPE_NAME[z.shape] }), keep,
          h('label', { class: 'toggle' }, on, h('span', { class: 'track' }), h('span', { class: 'faint', text: 'On' })),
          h('button', { class: 'btn', type: 'button', text: i === sel ? 'Done' : 'Edit', onclick: (e) => { e.stopPropagation(); selectZone(i === sel ? -1 : i); } }), del);
        zoneList.append(row);
      });
    }
    const z = list[sel];
    const esig = z ? sel + ':' + z.shape + ':' + (z.points ? z.points.length : 0) : '';
    if (force || esig !== editSig) {
      editSig = esig;
      zoneEdit.textContent = '';
      zoneEdit.fields = [];
      if (z) buildZoneEdit(z);
    }
    for (const f of zoneEdit.fields || []) f.paint();
    if (zn && cur) {
      const d = zn.clearance * 1000;
      setText(zoneNow, zn.state === 'violation' ? 'The arm is ' + fmt(-d, 0) + ' mm past ' + zn.name + '.'
        : d < (cur.touch_m || 0.005) * 1000 ? 'The arm is touching ' + zn.name + '.'
        : 'The arm is ' + fmt(d, 0) + ' mm from ' + zn.name + ', the nearest zone boundary.');
      zoneNow.className = 'znow ' + DOT[zn.state];
    } else { setText(zoneNow, list.some(z => z.enabled !== false) ? (zonesEdited() ? 'Apply to check the arm against these zones.' : 'No reading from the robot’s controller yet.') : ''); zoneNow.className = 'znow'; }
    paintBody();
  }
  function buildZoneEdit(z) {
    const i = sel;
    const f = (label, unit, lo, hi, step, read, write) => {
      const el = slideNum({ label, unit, lo, hi, step, digits: unit === '°' ? 0 : 3, get: () => { const c = zones()[i]; return c ? read(c) : null; },
        set: (v) => { const c = clone(zones()[i]); write(c, v); updateZone(i, c); } });
      zoneEdit.fields.push(el); return el;
    };
    const vec = (key, k, label, lo, hi) => f(label, 'm', lo, hi, 0.005, c => c[key][k], (c, v) => { c[key][k] = r3(v); });
    const rows = [];
    if (z.shape === 'box') {
      rows.push(vec('centre', 0, 'Centre X', -2, 2), vec('centre', 1, 'Centre Y', -2, 2), vec('centre', 2, 'Centre Z', -0.5, 2.5),
        vec('size', 0, 'Size X', 0.02, 3), vec('size', 1, 'Size Y', 0.02, 3), vec('size', 2, 'Size Z', 0.02, 3),
        f('Turn', '°', -180, 180, 1, c => c.yaw_deg || 0, (c, v) => { c.yaw_deg = Math.round(v); }));
    } else if (z.shape === 'sphere') {
      rows.push(vec('centre', 0, 'Centre X', -2, 2), vec('centre', 1, 'Centre Y', -2, 2), vec('centre', 2, 'Centre Z', -0.5, 2.5),
        f('Radius', 'm', 0.01, 1.5, 0.005, c => c.radius, (c, v) => { c.radius = r3(v); }));
    } else {
      if (z.shape === 'cylinder') rows.push(vec('centre', 0, 'Centre X', -2, 2), vec('centre', 1, 'Centre Y', -2, 2), f('Radius', 'm', 0.01, 1.5, 0.005, c => c.radius, (c, v) => { c.radius = r3(v); }));
      rows.push(f('Bottom', 'm', -0.5, 2.5, 0.005, c => c.z_min, (c, v) => { c.z_min = r3(Math.min(v, c.z_max - 0.01)); }),
        f('Top', 'm', -0.5, 2.5, 0.005, c => c.z_max, (c, v) => { c.z_max = r3(Math.max(v, c.z_min + 0.01)); }));
    }
    zoneEdit.append(h('div', { class: 'zedit-head' }, h('b', { text: (z.name || SHAPE_NAME[z.shape]) + ' · ' + SHAPE_NAME[z.shape] }),
      h('span', { class: 'faint', text: z.shape === 'prism' ? 'Drag a corner in the 3D view to move it; its numbers are below.' : 'Drag its handles in the 3D view, or set the values here.' })),
      h('div', { class: 'zgrid' }, ...rows));
    if (z.shape === 'prism') {
      const corners = h('div', { class: 'zcorners' });
      z.points.forEach((p, k) => {
        const cx = h('input', { class: 'input num', type: 'number', step: '0.005', 'aria-label': 'Corner ' + (k + 1) + ' X' }), cy = h('input', { class: 'input num', type: 'number', step: '0.005', 'aria-label': 'Corner ' + (k + 1) + ' Y' });
        const write = () => { const c = clone(zones()[i]); const x = parseFloat(cx.value), y = parseFloat(cy.value); if (isNaN(x) || isNaN(y)) return; c.points[k] = [r3(x), r3(y)]; updateZone(i, c); };
        cx.addEventListener('change', write); cy.addEventListener('change', write);
        const rm = h('button', { class: 'btn btn-icon', type: 'button', title: 'Remove this corner', 'aria-label': 'Remove corner ' + (k + 1), disabled: z.points.length <= 3,
          onclick: () => { const c = clone(zones()[i]); c.points.splice(k, 1); updateZone(i, c); } }, window.FIC.icon('x'));
        const cell = h('div', { class: 'zcorner' }, h('span', { class: 'faint num', text: String(k + 1) }), cx, cy, rm);
        cell.paint = () => { const c = zones()[i]; if (!c || !c.points[k]) return; if (document.activeElement !== cx) cx.value = c.points[k][0].toFixed(3); if (document.activeElement !== cy) cy.value = c.points[k][1].toFixed(3); };
        zoneEdit.fields.push(cell);
        corners.append(cell);
      });
      zoneEdit.append(h('div', { class: 'faint', style: 'margin-top:var(--s3);font-size:var(--fs-s)', text: 'Corners (x, y m)' }), corners);
    }
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
      paintJoint(i);
    });
    for (const row of velBox.children) row.paint && row.paint();
    const set = (path, v) => { const b = root.querySelector('[data-k="' + path + '"]'); if (b) b.checked = !!at(path, v); };
    set('joints.enabled', s.joints.enabled); set('velocity.enabled', s.velocity.enabled); set('monitor', s.enabled); set('enforce', s.enforce);
    mark();
    paintZones(force);
    const v = AXES.map(a => s.cartesian[a].value);
    setText(toolAt, v.some(x => x == null) ? 'Tool: —' : 'Tool at ' + v.map(x => fmt(x, 3)).join(', ') + ' m');
    draw();
  }

  // ── the 3D view: the workspace box (faces to drag), the zones (handles to drag) ──
  let mod = null, dead = false, themeKey = '', colours = null;
  const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  function zone(a, draftToo) {
    const c = cur.cartesian[a], g = (k, b) => draftToo ? get('c.' + a + '.' + k, b) : b;
    return { min: g('min', c.min), max: g('max', c.max), on: !!g('enabled', c.enabled), value: c.value };
  }
  function faceState(a, f, z) {
    if (z.value == null || !z.on) return 'ok';
    const band = (z.max - z.min) * (1 - cur.warning_threshold), past = f === 'min' ? z.min - z.value : z.value - z.max;
    return past > 0 ? 'violation' : past > -band ? 'warn' : 'ok';
  }
  function draw() {
    if (!mod || !cur) return;
    const key = css('--canvas') + css('--accent') + css('--warn');
    if (key !== themeKey) {
      themeKey = key;
      mod.setPalette({ background: null, grid: css('--rule-strong'), gridSoft: css('--rule') });
      colours = { ok: css('--accent'), warn: css('--warn'), violation: css('--bad'),
        zin: css('--ok') || '#2f7d4f', zout: css('--bad'), off: css('--ink-faint'), handle: css('--ink-strong') };
      mod.setOverlayColors({ zones: css('--ink-muted'), target: css('--bad') });
    }
    const applied = AXES.map(a => zone(a, false)), drafted = AXES.map(a => zone(a, true));
    const lim = (zs, k) => zs.map((z, i) => z.on ? z[k] : SPAN[AXES[i]][k === 'min' ? 0 : 1]);
    mod.setZones(applied.some(z => z.on) ? { min: lim(applied, 'min'), max: lim(applied, 'max') } : null);
    const faceCol = {}, edited = new Set();
    AXES.forEach((a, i) => { for (const f of ['min', 'max']) {
      faceCol[a + '.' + f] = colours[faceState(a, f, drafted[i])];
      if (draft.has('c.' + a + '.' + f)) edited.add(a + '.' + f);
    } });
    mod.setZoneFaces(tool === 'orbit' && drafted.some(z => z.on) ? { min: lim(drafted, 'min'), max: lim(drafted, 'max'), on: drafted.map(z => z.on) } : null, faceCol, edited);
    const v = drafted.map(z => z.value);
    mod.setTarget(v.some(x => x == null) ? null : v);
    const zn = cur.zone_now;
    mod.setShapeZones(zones(), { sel, colours: { in: colours.zin, out: colours.zout, off: colours.off, warn: colours.warn, bad: colours.violation, handle: colours.handle },
      state: zn && !zonesEdited() ? { zone: zn.zone, state: zn.state } : null });
    mod.setWorstPoint(zn && zn.state !== 'ok' ? zn.point : null, zn ? (zn.state === 'violation' ? colours.violation : colours.warn) : null);
    binBtn.disabled = !zones()[sel];
    binBtn.title = zones()[sel] ? 'Delete ' + zones()[sel].name + ' (Review and Apply to remove it from the controllers)' : 'Delete the selected zone: select one first';
    setText($('#sf-3d-aside'), tool !== 'orbit' ? '' : sel >= 0 ? 'drag the handles of ' + (zones()[sel] || {}).name : 'click a zone to edit it');
  }
  function onFaceDrag(face, value, done) {
    const a = face[0], f = face.slice(2), z = zone(a, true), lim = cur.bounds.cart_abs;
    let v = Math.round(value / 0.005) * 0.005;
    v = f === 'min' ? Math.min(v, z.max - MIN_GAP) : Math.max(v, z.min + MIN_GAP);
    v = Math.max(-lim, Math.min(lim, v));
    draft.set('c.' + a + '.' + f, +v.toFixed(3));
    paint(true);
    const was = cur.cartesian[a][f];
    setText(stageNote, a.toUpperCase() + ' ' + f + ' ' + fmt(v, 3) + ' m' + (Math.abs(v - was) > 1e-9 ? ' (applied: ' + fmt(was, 3) + ')' : '')
      + (done ? ' · not applied yet: Review changes' : ''));
  }
  const snap = v => Math.round(v / 0.005) * 0.005;
  function nextName(shape) {
    const used = new Set(zones().map(z => z.name));
    for (let k = 1; ; k++) { const n = SHAPE_NAME[shape] + ' ' + k; if (!used.has(n)) return n; }
  }
  const zoneEvents = {
    select: (i) => { if (i !== sel) selectZone(i); },
    change: (i, z, done) => {
      if (z.centre) z.centre = z.centre.map(v => r3(snap(v)));
      if (z.size) z.size = z.size.map(v => r3(Math.max(0.02, snap(v))));
      if (z.radius != null) z.radius = r3(Math.max(0.01, snap(z.radius)));
      if (z.z_min != null) { z.z_min = r3(snap(z.z_min)); z.z_max = r3(Math.max(z.z_min + 0.01, snap(z.z_max))); }
      if (z.points) z.points = z.points.map(p => [r3(snap(p[0])), r3(snap(p[1]))]);
      if (z.yaw_deg != null) z.yaw_deg = Math.round(z.yaw_deg);
      updateZone(i, z);
      if (done) setText(stageNote, z.name + ' moved · not applied yet: Review changes');
    },
    place: (shape, p) => {
      if (zones().length >= ((cur && cur.bounds.max_zones) || 16)) { toast('At most ' + cur.bounds.max_zones + ' zones', true); return; }
      const x = r3(snap(p[0])), y = r3(snap(p[1]));
      const z = { name: nextName(shape), shape, keep: newKeep, enabled: true };
      if (shape === 'box') Object.assign(z, { centre: [x, y, 0.25], size: [0.3, 0.3, 0.5], yaw_deg: 0 });
      else if (shape === 'sphere') Object.assign(z, { centre: [x, y, 0.3], radius: 0.15 });
      else Object.assign(z, { centre: [x, y], radius: 0.15, z_min: 0, z_max: 0.8 });
      const l = clone(zones()); l.push(z); sel = l.length - 1;
      setTool('orbit');
      setZones(l);
      setText(stageNote, z.name + ' added (' + (newKeep === 'in' ? 'keep in' : 'keep out') + '). Drag its handles to size it · not applied yet: Review changes');
    },
    drawing: (n) => { drawn = n; paintStrip(); },
    drawn: (pts) => {
      const lo = parseFloat(drawBottom.value), hi = parseFloat(drawTop.value);
      if (!(hi - lo >= 0.01)) { toast('The outline’s top must be at least 1 cm above its bottom', true); return; }
      const z = { name: nextName('prism'), shape: 'prism', keep: newKeep, enabled: true, points: pts.map(p => [r3(snap(p[0])), r3(snap(p[1]))]), z_min: r3(lo), z_max: r3(hi) };
      const l = clone(zones()); l.push(z); sel = l.length - 1;
      setTool('orbit');
      setZones(l);
      setText(stageNote, z.name + ' drawn with ' + pts.length + ' corners (' + (newKeep === 'in' ? 'keep in' : 'keep out') + ') · not applied yet: Review changes');
    },
  };
  function setTool(t) {
    tool = t; drawn = 0;
    for (const [k, b] of toolBtns) b.setAttribute('aria-pressed', String(k === t));
    if (mod) mod.setZoneTool(t, parseFloat(drawBottom.value) || 0);
    paintStrip(); draw();
  }
  function paintStrip() {
    strip.textContent = '';
    strip.hidden = tool === 'orbit';
    strip.style.top = (tools.offsetHeight + 16) + 'px';           // under the toolbar, however it wraps
    if (tool === 'orbit') { setText(stageNote, 'Click a zone to select it, then drag its handles · drag a workspace face to move it · drag elsewhere to turn · scroll or pinch to zoom. The red dot is the tool point.'); return; }
    const cancel = h('button', { class: 'btn', type: 'button', text: 'Cancel', onclick: () => setTool('orbit') });
    if (tool === 'draw') {
      setText(drawCount, drawn + (drawn === 1 ? ' corner' : ' corners'));
      strip.append(h('span', { text: 'Click the floor at each corner; click the first one or double-click to close.' }), drawCount,
        h('label', { class: 'saf-mm' }, h('span', { class: 'faint', text: 'bottom' }), drawBottom), h('label', { class: 'saf-mm' }, h('span', { class: 'faint', text: 'top' }), drawTop, h('span', { class: 'faint', text: 'm' })),
        h('button', { class: 'btn btn-primary', type: 'button', text: 'Finish', disabled: drawn < 3, onclick: () => mod && mod.finishDraw() }),
        h('button', { class: 'btn', type: 'button', text: 'Undo', disabled: !drawn, onclick: () => mod && mod.undoDraw(false) }), cancel);
      setText(stageNote, 'Drawing an outline (' + (newKeep === 'in' ? 'keep in' : 'keep out') + '): Enter finishes it, Backspace undoes a corner, Esc cancels.');
    } else {
      strip.append(h('span', { text: 'Click the floor where the ' + SHAPE_NAME[tool].toLowerCase() + ' goes (' + (newKeep === 'in' ? 'keep in' : 'keep out') + ').' }), cancel);
      setText(stageNote, 'Placing a ' + SHAPE_NAME[tool].toLowerCase() + ': click the floor · drag to turn the view · Esc stops.');
    }
  }
  drawBottom.addEventListener('change', () => { if (mod && tool === 'draw') mod.setZoneTool('draw', parseFloat(drawBottom.value) || 0); });
  function onKey(e) {
    if (e.target.closest && e.target.closest('input, textarea, select')) return;
    if (e.key === 'Escape' && tool !== 'orbit') { setTool('orbit'); e.preventDefault(); }
    else if (tool === 'draw' && e.key === 'Enter' && mod) { mod.finishDraw(); e.preventDefault(); }
    else if (tool === 'draw' && e.key === 'Backspace' && mod) { mod.undoDraw(false); e.preventDefault(); }
  }
  document.addEventListener('keydown', onKey);
  async function load() {
    try {
      const m = await import('/view3d.js');
      await m.mount(stage);
      if (dead) { m.unmount(); return; }
      mod = m; themeKey = '';
      m.onZoneDrag(onFaceDrag);
      m.onShapeZones(zoneEvents);
      m.setView([0, 0, 0.45], 2.7, -0.7, 1.05);                 // the whole default zone in view
      stage.append(tools, toolAt, strip);                         // above the canvas
      setTool(tool);
      const s = window.FIC.state(); if (s) { m.setJoints(s.joints); m.setGripper(s.gripper); }
      draw();
    } catch (e) {
      setText(stageNote, 'No 3D view: ' + (e && e.message ? e.message : e));
    }
  }
  load();
  const onResize = () => { if (mod) mod.resize(); };
  window.addEventListener('resize', onResize);

  // ── a workspace limit where the tool is ──
  function teach(face) {
    setText(touchErr, '');
    if (!cur) return;
    const p = AXES.map(a => cur.cartesian[a].value);
    if (p.some(x => x == null)) { setText(touchErr, 'The Studio has no tool position from the robot yet.'); return; }
    if (face === 'wall') face = Math.abs(p[0]) >= Math.abs(p[1]) ? (p[0] >= 0 ? 'x.max' : 'x.min') : (p[1] >= 0 ? 'y.max' : 'y.min');
    const mm = parseFloat(margin.value);
    if (!(mm >= 0 && mm <= 200)) { setText(touchErr, 'The margin must be 0 to 200 mm.'); return; }
    const a = face[0], f = face.slice(2), z = zone(a, true), at = cur.cartesian[a].value;
    const v = +(f === 'min' ? at + mm / 1000 : at - mm / 1000).toFixed(3);
    if (Math.abs(v) > cur.bounds.cart_abs) { setText(touchErr, a.toUpperCase() + ' ' + f + ' would be ' + fmt(v, 3) + ' m, beyond ±' + cur.bounds.cart_abs + ' m.'); return; }
    if (z.on && (f === 'min' ? v > z.max - MIN_GAP : v < z.min + MIN_GAP)) {
      setText(touchErr, a.toUpperCase() + ' ' + f + ' at ' + fmt(v, 3) + ' m would leave less than ' + MIN_GAP * 1000 + ' mm to ' + a.toUpperCase() + ' ' + (f === 'min' ? 'max' : 'min') + ' (' + fmt(f === 'min' ? z.max : z.min, 3) + ' m). Change that one first.');
      return;
    }
    draft.set('c.' + a + '.' + f, v);
    if (!z.on) draft.set('c.' + a + '.enabled', true);
    paint(true);
    toast(a.toUpperCase() + ' ' + f + ' set to ' + fmt(v, 3) + ' m (tool at ' + fmt(at, 3) + ' m, ' + mm + ' mm margin). Not applied yet.');
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
        lines.push((widen ? 'WIDER  ' : '       ') + 'Workspace ' + a.toUpperCase() + ' ' + f + ': ' + (f === 'enabled' ? (c[f] ? 'checked' : 'off') + ' -> ' + (draft.get(p) ? 'checked' : 'off') : fmt(c[f], 3) + ' -> ' + fmt(draft.get(p), 3)));
      }
    }
    if (Object.keys(cart).length) out.cartesian = cart;
    if (zonesEdited()) {
      out.zones = zd;
      const before = new Map((s.zones || []).map(z => [z.name, z])), after = new Map(zd.map(z => [z.name, z]));
      for (const z of zd) {
        const b = before.get(z.name);
        const desc = SHAPE_NAME[z.shape].toLowerCase() + ', keep ' + (z.keep || 'out') + (z.enabled === false ? ', off' : '');
        if (!b) lines.push((z.enabled === false ? 'WIDER  ' : '       ') + 'New zone ' + z.name + ' (' + desc + ')');
        else if (JSON.stringify(b) !== JSON.stringify(z)) lines.push(((b.enabled !== false && z.enabled === false) || b.keep !== z.keep ? 'WIDER  ' : '       ') + 'Zone ' + z.name + ' changed (' + desc + ')');
      }
      for (const [n, b] of before) if (!after.has(n)) lines.push((b.enabled !== false ? 'WIDER  ' : '       ') + 'Zone ' + n + ' removed');
    }
    if (bodyEdited()) {
      out.body = bd;
      const b0 = s.body || {};
      lines.push((bd.radius < b0.radius || bd.tool_radius < b0.tool_radius || bd.from_joint > b0.from_joint ? 'WIDER  ' : '       ') + 'The arm checked from joint ' + bd.from_joint
        + ', links ' + fmt(bd.radius * 1000, 0) + ' mm, tool ' + fmt(bd.tool_radius * 1000, 0) + ' mm');
    }
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
    if (!lines.length) { draft.clear(); zd = bd = null; mark(); toast('Nothing differs from the current limits'); return; }
    const wider = lines.some(l => l.startsWith('WIDER'));
    for (const a of AXES) {
      const z = zone(a, true), stop = cur.enforce ? cur.stop_past.m : Infinity;
      if (!z.on || z.value == null) continue;
      const past = Math.max(z.min - z.value, z.value - z.max);
      if (past > 0) lines.push('NOTE   The tool is ' + fmt(past * 1000, 0) + ' mm outside the new workspace ' + a.toUpperCase() + (z.value < z.min ? ' min' : ' max')
        + (past > stop ? ': Stop on violation will stop the robot. Move the arm clear first.' : ': the controller will push it back inside. Move the arm clear first.'));
    }
    if (out.zones) lines.push('NOTE   The controller checks the zones along the arm as soon as they are applied: an arm already inside a keep-out zone (or outside every keep-in zone) is pushed out, and stopped if it is more than ' + fmt(cur.stop_past.m * 1000, 0) + ' mm in.');
    inlineConfirm(askHost, {
      title: wider ? 'Apply these changes? Some of them make the limits wider.' : 'Apply these changes to the safety limits?',
      lines: [...lines, 'The controllers start using them as soon as they are applied, and they are saved with the robot.'],
      label: 'Apply to the controllers', danger: wider,
      onConfirm: async () => {
        const r = await cmd('safety_set', out, true);
        if (r.ok) { draft.clear(); zd = bd = null; if (r.safety) cur = r.safety; toast('Safety limits applied'); paint(true); }
        else setText($('#saf-err'), (r.message || 'The Studio refused the change') + '. Nothing was changed.');
      } });
  }

  function update(s) {
    if (mod) { mod.setJoints(s.joints); mod.setGripper(s.gripper); }
    const sf = s.safety; if (!sf) { setText($('#lede'), 'This Studio reports no safety state.'); return; }
    cur = sf;
    if (shape !== sf.joints.rows.length + ':' + sf.bounds.cart_abs) build(sf);
    setText($('#lede'), 'Limits are published to every robot’s controller; the Studio also checks positions at 10 Hz.');
    setText($('#sf-over'), OVERALL[sf.overall]); setText($('#sf-over-n'), sf.enabled ? 'monitor on' : 'positions are not checked');
    $('#sf-over').style.color = sf.overall === 'violation' ? 'var(--bad)' : sf.overall === 'warn' ? 'var(--warn)' : '';
    setText($('#sf-mon'), sf.enabled ? 'On' : 'Off'); setText($('#sf-mon-n'), 'warn and log');
    setText($('#sf-enf'), sf.enforce ? 'Stop' : 'Warn only'); setText($('#sf-enf-n'), sf.enforce ? 'graceful stop ' + fmt(sf.stop_past.m * 1000, 0) + ' mm past' : 'the robot is not stopped');
    setText($('#sf-thr'), fmt(sf.warning_threshold * 100, 0) + ' %'); setText($('#sf-thr-n'), 'of each range');
    setText($('#sf-c-aside'), Object.values(sf.cartesian).filter(c => c.enabled).length + ' of 3 checked');
    const zl = zones(), on = zl.filter(z => z.enabled !== false).length;
    setText($('#sf-z-aside'), zl.length ? on + ' of ' + zl.length + ' on' : 'none');
    setText($('#sf-j-aside'), sf.joints.enabled ? sf.joints.rows.length + ' joints' : 'not checked');
    setText($('#sf-v-aside'), sf.velocity.enabled ? 'limited' : 'not limited');
    paint(false);
  }
  return { update, dispose() {
    dead = true; window.removeEventListener('resize', onResize); document.removeEventListener('keydown', onKey);
    if (mod) { mod.onZoneDrag(null); mod.onShapeZones(null); mod.setShapeZones([], { sel: -1 }); mod.setZoneTool('orbit'); mod.unmount(); }
  } };
};
})();
