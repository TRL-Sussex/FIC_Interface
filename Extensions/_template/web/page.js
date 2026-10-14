// My Extension — web page. Loaded by the Studio's web UI as an ES module.
// build(root, api) draws the page once; update(state, studio) refreshes it with
// web_state() every poll_ms; dispose() runs when the page is left.
//
// api.ui    the UI kit: section, grid, stats, field, button, buttons, toggle, select,
//           number, text, slider, bar, log, note, table (see the docs page)
// api.call(name, args)  runs web_action(name, args) in the Studio; resolves to its
//           result, or null when it was refused (the reason is already shown)
// api.state()   the last web_state();  api.studio()  the Studio's own snapshot
export default function build(root, api) {
  const { ui } = api;

  // Live readouts
  const readouts = ui.stats([
    { key: 'robot', label: 'Robot' },
    { key: 'x', label: 'End effector x', unit: 'm', digits: 3 },
    { key: 'y', label: 'y', unit: 'm', digits: 3 },
    { key: 'z', label: 'z', unit: 'm', digits: 3 },
  ]);
  const joints = ui.table({ columns: [{ key: 'j', label: 'Joint' }, { key: 'deg', label: 'Angle °', num: true }], empty: 'No joint data yet' });

  // Settings: a slider card and a switch; each change goes to web_action
  const gain = ui.slider({ label: 'Gain', min: 0, max: 1, step: 0.01, digits: 2,
    onChange: v => api.call('set_gain', { value: v }) });
  const enabled = ui.toggle({ label: 'Enabled', onChange: v => api.call('set_enabled', { value: v }) });

  // Actions
  const text = ui.text({ value: 'hello', placeholder: 'Message' });
  const actions = ui.buttons(
    ui.button({ label: 'Say hello', kind: 'primary', onClick: () => api.call('hello') }),
    ui.button({ label: 'Publish', kind: 'solid', onClick: () => api.call('publish', { text: text.value }) }),
    ui.button({ label: 'Reset settings', title: 'Only from the Studio window (manifest local_only)', onClick: () => api.call('reset_settings') }));

  const log = ui.log({ lines: [], height: 220 });
  const status = ui.note('');

  root.append(
    ui.section('Robot', { aside: '' }, readouts, joints),
    ui.section('Settings', ui.grid(2, gain, ui.field('Output', enabled))),
    ui.section('Actions', ui.field('Message', text), actions, status),
    ui.section('Log', log, ui.buttons(ui.button({ label: 'Clear', onClick: () => api.call('clear_log') }))),
  );

  return {
    update(s) {
      readouts.set('robot', s.robot || '—', s.running ? 'controller running' : 'controller stopped');
      readouts.set('x', s.ee ? s.ee[0] : null);
      readouts.set('y', s.ee ? s.ee[1] : null);
      readouts.set('z', s.ee ? s.ee[2] : null);
      joints.set((s.joints_deg || []).map((d, i) => ({ j: 'J' + (i + 1), deg: d.toFixed(1) })));
      gain.set(s.gain);
      enabled.set(s.enabled);
      log.set(s.log);
      status.set(s.enabled ? '' : 'Output is off: switch it on above.', 'warn');
    },
    dispose() {},
  };
}
