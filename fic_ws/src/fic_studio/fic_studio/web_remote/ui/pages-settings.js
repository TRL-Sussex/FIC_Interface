/* Settings: Processes (the simulator, controller and driver one at a time); the classic page's cards, drawn from the description the Studio
   gives (settings_get): Appearance, Startup, Network, Simulation, Playback
   tolerances, Shortcuts, Extensions, Gamepad, Web Remote and Developer with
   its build buttons. Save writes the Studio's own settings file through its own
   _save_settings and _apply_settings. Nothing here knows a setting by name. */
(() => {
'use strict';
const { $, h, fmt, setText, section: _s, csection, csectionTools, pageTitle, cmd, toast, inlineConfirm, armed } = window.FIC;

window.FIC.pages.settings = function buildSettings(root) {
  const section = (title, aside, ...b) => csection('settings/' + title.split(' · ')[0], title, aside, ...b);
  const st = { cards: [], values: {}, edits: {}, ext: [], extEdits: null, gp: null, web: null };
  const body = h('div', { id: 'st-body', class: 'flow-auto' }), askHost = h('div'), err = h('div', { class: 'bad-line' });
  const head = h('div', { class: 'tn-head' }, h('div', { class: 'spacer' }),
    h('button', { class: 'btn btn-primary', type: 'button', text: 'Save settings', onclick: () => save() }),
    h('button', { class: 'btn', type: 'button', text: 'Discard', title: 'Discard unsaved changes and reload the saved settings', onclick: () => { st.edits = {}; st.extEdits = null; load(); } }),
    h('button', { class: 'btn', type: 'button', text: 'Reset to defaults', onclick: () => inlineConfirm(askHost, { title: 'Reset every setting to its default?', lines: ['The settings file is overwritten.'], label: 'Reset', danger: true,
      onConfirm: async () => { const r = await cmd('settings_reset', { confirm: true }); if (r.ok) { toast('Settings reset'); st.edits = {}; load(); } } }) }),
    h('a', { class: 'btn', href: '#/help', text: '? Help' }));
  root.append(...pageTitle('Settings', ''), head, csectionTools(root), err, askHost, body);
  window.FIC.onSave = () => save();
  const dirty = () => Object.keys(st.edits).length > 0;

  async function load() {
    const r = await cmd('settings_get', {}, true);
    if (!r.ok) { setText($('#lede'), r.message || 'Settings are not available.'); return; }
    Object.assign(st, { cards: r.cards, values: r.values, ext: r.extensions, gp: r.gamepad, web: r.web, file: r.file });
    setText($('#lede'), r.file ? 'Saved in ' + r.file : '');
    draw();
  }
  function fieldEl(f) {
    const key = f.key, cur = key in st.edits ? st.edits[key] : st.values[key];
    const set = v => { if (v === st.values[key]) delete st.edits[key]; else st.edits[key] = v; mark(); };
    let ctl;
    if (f.type === 'bool') { ctl = h('label', { class: 'toggle' }, h('input', { type: 'checkbox', onchange: e => set(e.target.checked) }), h('span', { class: 'track' })); ctl.firstChild.checked = !!cur; }
    else if (f.type === 'choice') {
      ctl = h('select', { class: 'select' }, ...f.choices.map(c => h('option', { value: c.value, text: c.label, disabled: !!c.disabled })));
      ctl.value = cur == null ? '' : cur; ctl.addEventListener('change', () => set(ctl.value));
    } else {
      const num = f.type === 'int' || f.type === 'float';
      ctl = h('input', { class: 'input' + (num ? ' num' : ''), type: num ? 'number' : 'text', 'aria-label': f.label, placeholder: f.placeholder || null });
      if (num) { ctl.min = f.min; ctl.max = f.max; ctl.step = f.step || (f.type === 'int' ? 1 : 'any'); }
      ctl.value = cur == null ? '' : cur;
      ctl.addEventListener('input', () => { ctl.setAttribute('aria-invalid', String(!ctl.validity.valid)); if (ctl.value === '') return; set(num ? parseFloat(ctl.value) : ctl.value); });
    }
    return h('div', { class: 'frow' + (key in st.edits ? ' edited' : ''), 'data-row': key, title: f.hint || null }, h('label', { text: f.label }), ctl, h('span', { class: 'unit faint', text: f.unit || '' }));
  }
  function mark() { for (const r of body.querySelectorAll('[data-row]')) r.classList.toggle('edited', r.dataset.row in st.edits); }
  function draw() {
    body.textContent = '';
    st.dev = [];
    for (const c of st.cards) {
      if (c.title === 'Developer') { st.dev = c.fields; continue; }      // drawn with its build buttons below
      body.append(section(c.title + (c.restart ? ' · restart to apply' : ''), null, c.hint ? h('p', { class: 'faint', style: 'font-size:var(--fs-m);margin-bottom:var(--s4)' }, c.hint) : null,
        h('div', { class: 'fgrid' }, ...c.fields.map(fieldEl))));
    }
    body.prepend(procCard());
    body.append(extCard(), gamepadCard(), webCard(), buildCard());
    const q = new URLSearchParams((location.hash.split('?')[1] || '')).get('build');
    if (q && !st.opened) { st.opened = true; const sec = $('#st-build'); if (sec) { if (sec.csec) sec.csec.set(true, false); sec.scrollIntoView(); if (q === 'workspace' || q === 'clean') startBuild(q); } }
  }

  // ── Processes: the simulator, the controller and the driver one at a time (Start in the top bar runs them together) ──
  const procState = {};
  function procCard() {
    const row = (id, label, key, startText) => {
      const go = h('button', { class: 'btn' + (id === 'simulation' ? ' btn-primary' : ''), type: 'button', icon: 'play', text: startText, title: 'Start the ' + label + ' alone' });
      const stop = h('button', { class: 'btn btn-danger', type: 'button', icon: 'stop', text: 'Stop', title: 'Stop the ' + label });
      armed(go, 'Start?', () => cmd('process', { id, op: 'start', confirm: true }));
      armed(stop, 'Stop?', () => cmd('process', { id, op: 'stop', confirm: true }));
      procState[key] = h('span', { class: 'faint', text: 'stopped' });
      return h('div', { class: 'proc' }, h('span', { class: 'proc-name', text: label }), go, stop, procState[key]);
    };
    // open until folded (its key is outside settings/, whose sections start folded): its buttons are what one comes for
    const sec = csection('settings-processes', 'Processes', null,
      h('p', { class: 'faint', style: 'font-size:var(--fs-m);margin-bottom:var(--s4)' },
        'Start one part on its own. Simulator only runs the simulator without the FIC controller: nothing commands the robot. Start in the top bar runs what the target needs together; Simulation → Processes… lists every process.'),
      h('div', { class: 'procs procs-col' },
        row('simulation', 'simulator', 'SIM', 'Start simulator only (no controller)'),
        row('fic', 'FIC controller (simulation)', 'FIC', 'Start controller'),
        row('driver', 'robot driver', 'DRV', 'Start driver'),
        row('controller', 'FIC controller (hardware)', 'CTL', 'Start controller')));
    update(window.FIC.state());
    return sec;
  }
  function update(s) {
    const p = (s && s.session && s.session.processes) || {};
    for (const [key, on] of [['SIM', p.simulation], ['FIC', p.fic], ['DRV', p.driver], ['CTL', p.controller]]) {
      if (!procState[key]) continue;
      setText(procState[key], on ? 'running' : 'stopped'); procState[key].className = on ? 'ok-t' : 'faint';
    }
  }

  // ── Extensions ──
  function extCard() {
    const rows = st.ext.map(e => {
      const cb = h('input', { type: 'checkbox' }); cb.checked = st.extEdits ? st.extEdits.has(e.id) : e.enabled;
      cb.addEventListener('change', () => { if (!st.extEdits) st.extEdits = new Set(st.ext.filter(x => x.enabled).map(x => x.id)); cb.checked ? st.extEdits.add(e.id) : st.extEdits.delete(e.id); });
      return h('div', { class: 'xrow ext-row' }, h('label', { class: 'toggle' }, cb, h('span', { class: 'track' }), h('span', { text: e.name })), h('div', { class: 'faint', style: 'font-size:var(--fs-m)', text: e.description }));
    });
    return section('Extensions · restart to apply', null, h('p', { class: 'faint', style: 'font-size:var(--fs-m)' }, 'Extensions add optional pages. Toggle them and apply; a restart is needed for changes to take effect.'),
      h('div', { class: 'rows' }, ...(rows.length ? rows : [h('div', { class: 'empty', text: 'No extensions found.' })])),
      h('div', { class: 'btn-row', style: 'margin-top:var(--s4)' }, h('button', { class: 'btn btn-primary', type: 'button', text: 'Apply extensions', onclick: () => {
        if (!st.extEdits) { toast('No changes to apply'); return; }
        const ids = [...st.extEdits];
        inlineConfirm(askHost, { title: 'Save the extension list?', lines: [ids.join(', ') || 'none'], label: 'Apply',
          onConfirm: async () => { const r = await cmd('settings_extensions', { enabled: ids, confirm: true }); if (r.ok) { toast('Saved. Restart the Studio to apply.'); st.extEdits = null; load(); } } });
      } })));
  }

  // ── Gamepad ──
  const gpBox = h('div');
  function gamepadCard() { drawGamepad(); return section('Gamepad', { id: 'gp-aside' }, gpBox); }
  function drawGamepad() {
    const g = st.gp; gpBox.textContent = '';
    if (!g || !g.available) { gpBox.append(h('div', { class: 'empty', text: 'The gamepad is not available on this Studio.' })); return; }
    const dz = h('input', { class: 'input num', type: 'number', min: '0', max: '0.5', step: '0.01', value: String(g.deadzone), 'aria-label': 'Deadzone' });
    dz.addEventListener('change', () => cmd('settings_gamepad', { op: 'deadzone', value: parseFloat(dz.value) }));
    const inv = h('div', { class: 'btn-row' }, ...Object.entries(g.invert).map(([ax, on]) => {
      const cb = h('input', { type: 'checkbox' }); cb.checked = on; cb.addEventListener('change', () => cmd('settings_gamepad', { op: 'invert', axis: ax, value: cb.checked }));
      return h('label', { class: 'toggle' }, cb, h('span', { class: 'track' }), h('span', { text: { LX: 'Left stick X', LY: 'Left stick Y', RX: 'Right stick X', RY: 'Right stick Y' }[ax] }));
    }));
    const map = h('div', { class: 'fgrid' }, ...g.buttons.map(b => {
      const sel = h('select', { class: 'select' }, ...g.actions.map(a => h('option', { value: a, text: a }))); sel.value = b.action;
      sel.addEventListener('change', () => cmd('settings_gamepad', { op: 'map', button: b.name, action: sel.value }));
      return h('div', { class: 'frow' }, h('label', { text: b.name }), sel, h('span'));
    }));
    gpBox.append(h('div', { class: 'toolbar', style: 'margin-top:0' }, h('div', { class: 'field' }, h('label', { text: 'Deadzone' }), dz), h('div', { class: 'field' }, h('label', { text: 'Axis inversion' }), inv)),
      h('div', { class: 'eyebrow', style: 'margin-top:var(--s5)', text: 'Button mapping' }), map,
      h('p', { class: 'faint', style: 'font-size:var(--fs-m);margin-top:var(--s3)' }, 'Fixed controls: ' + g.fixed.join('  ·  ')),
      h('div', { class: 'eyebrow', style: 'margin-top:var(--s5)', text: 'Connected controllers' }), padsEl(g));
  }
  function padsEl(g) {
    if (!g.pads.length) return h('div', { class: 'empty', text: 'No controllers detected. Plug in a USB gamepad.' });
    return h('div', { class: 'rows' }, ...g.pads.map(p => {
      const sel = h('select', { class: 'select' }, ...g.robots.map(r => h('option', { value: r.n, text: r.label }))); sel.value = p.robot;
      sel.addEventListener('change', () => cmd('settings_gamepad', { op: 'pad', path: p.path, robot: Number(sel.value) }));
      return h('div', { class: 'pair' }, h('span', null, p.name || p.path, h('span', { class: 'faint', text: '  ' + p.path + ' · ' + p.space + ' · speed ' + p.speed })), h('span', { class: 'field' }, h('label', { text: 'Robot' }), sel));
    }));
  }

  // ── Web remote ──
  const webBox = h('div');
  function webCard() { drawWeb(); return section('Web Remote', null, webBox); }
  function drawWeb() {
    const w = st.web; webBox.textContent = '';
    if (!w) return;
    const en = h('input', { type: 'checkbox' }); en.checked = w.enabled; en.addEventListener('change', async () => { const r = await cmd('settings_web', { op: 'enable', value: en.checked }); if (r.ok) { st.web = r.web; drawWeb(); } });
    const port = h('input', { class: 'input num', type: 'number', min: '1024', max: '65535', value: String(w.port), 'aria-label': 'Port' });
    webBox.append(h('div', { class: 'btn-row' }, h('label', { class: 'toggle' }, en, h('span', { class: 'track' }), h('span', { text: 'Serve the web remote' })),
      h('div', { class: 'field' }, h('label', { text: 'Port' }), port),
      h('button', { class: 'btn', type: 'button', text: 'Apply', onclick: async () => { const r = await cmd('settings_web', { op: 'port', value: Number(port.value) }); if (r.ok) { st.web = r.web; drawWeb(); toast('Port applied'); } } })),
      h('div', { class: 'faint', style: 'margin-top:var(--s3);font-size:var(--fs-m)' }, w.serving ? w.urls.join('   ') : (w.error || 'Not serving.')),
      h('div', { class: 'btn-row', style: 'margin-top:var(--s3)' }, h('span', { class: 'faint', text: w.clients + ' browser' + (w.clients === 1 ? '' : 's') + ' connected' + (w.holder ? ' · ' + w.holder.name + ' @ ' + w.holder.addr + ' in control' : '') }),
        w.holder ? h('button', { class: 'btn', type: 'button', text: 'Take control back', onclick: async () => { const r = await cmd('settings_web', { op: 'revoke' }); if (r.ok) { st.web = r.web; drawWeb(); } } }) : null),
      h('p', { class: 'faint', style: 'font-size:var(--fs-m);margin-top:var(--s3)' }, 'Open a URL above on a phone on the same network. Anyone who can reach this port can command the robot, one browser at a time; any browser can E-stop. Turn it off on networks you do not trust. Resetting the admin password is done from the desktop window.'));
  }

  // ── Build ──
  const out = h('pre', { class: 'xlog', id: 'st-buildout', text: '' }), pkgSel = h('select', { class: 'select', 'aria-label': 'Package' });
  let timer = null;
  function buildCard() {
    cmd('build', { op: 'packages' }, true).then(r => { if (r.ok) pkgSel.replaceChildren(...r.packages.map(p => h('option', { value: p, text: p }))); });
    const mk = (label, op, danger) => { const b = h('button', { class: 'btn' + (danger ? ' btn-danger' : ''), type: 'button', text: '⟳ ' + label }); b.addEventListener('click', () => startBuild(op)); return b; };
    const dev = section('Developer', null,
      h('div', { class: 'fgrid' }, ...st.dev.map(fieldEl)),
      h('div', { class: 'eyebrow', style: 'margin-top:var(--s5)', text: 'Build' }),
      h('div', { class: 'btn-row', style: 'margin-top:var(--s2)' }, mk('Rebuild Workspace', 'workspace'), h('div', { class: 'field' }, h('label', { text: 'Package' }), pkgSel), mk('Rebuild Package', 'package'), mk('Clean Rebuild', 'clean', true)),
      out);
    dev.id = 'st-build';
    return dev;
  }
  function startBuild(op) {
    const pkg = pkgSel.value;
    inlineConfirm(askHost, { title: op === 'clean' ? 'Delete build, install and log, and rebuild?' : op === 'package' ? 'Build package ' + pkg + '?' : 'Build the whole workspace?', lines: ['colcon runs on the Studio machine; its output appears below.'],
      label: 'Start build', danger: op === 'clean',
      onConfirm: async () => { const r = await cmd('build', { op, package: pkg, confirm: true }); if (r.ok) poll(); } });
  }
  async function poll() {
    clearInterval(timer);
    const tick = async () => {
      const r = await cmd('build', { op: 'status' }, true);
      if (!r.ok) return;
      out.textContent = r.lines.join('\n'); out.scrollTop = out.scrollHeight;
      if (!r.running) { clearInterval(timer); if (r.code != null) toast(r.code === 0 ? 'Build finished' : 'Build failed (exit ' + r.code + ')', r.code !== 0); }
    };
    timer = setInterval(tick, 1000); tick();
  }

  async function save() {
    if (!dirty()) { toast('No changes to save'); return; }
    const restart = Object.keys(st.edits).some(k => k.startsWith('network.'));
    const go = async confirm => {
      const r = await cmd('settings_set', { values: st.edits, confirm }, true);
      if (r.ok) { toast(r.restart ? 'Saved. Restart the Studio for the network settings.' : 'Settings saved'); st.edits = {}; setText(err, ''); load(); }
      else setText(err, r.message || 'Refused');
    };
    if (restart) inlineConfirm(askHost, { title: 'Save network settings?', lines: Object.keys(st.edits).map(k => k + ' -> ' + JSON.stringify(st.edits[k])).concat(['They take effect after a restart.']), label: 'Save', onConfirm: () => go(true) });
    else go(false);
  }
  const gpTimer = setInterval(async () => {
    const [g, w] = await Promise.all([cmd('settings_gamepad', { op: 'state' }, true), cmd('settings_web', { op: 'state' }, true)]);
    if (g.ok && st.gp && JSON.stringify(g.gamepad.pads) !== JSON.stringify(st.gp.pads) && !(document.activeElement && gpBox.contains(document.activeElement))) { st.gp = g.gamepad; drawGamepad(); }
    if (w.ok && JSON.stringify(w.web) !== JSON.stringify(st.web) && !(document.activeElement && webBox.contains(document.activeElement))) { st.web = w.web; drawWeb(); }
  }, 2000);
  load();
  return { update, dispose() { clearInterval(timer); clearInterval(gpTimer); } };
};
})();
