/* 3D view: the robot drawn in the browser by the shared static/view3d.js.
   The Studio sends the model once (/api/scene, /api/mesh); the pose comes from
   the joint values already in the state stream, so nothing here knows kinematics. */
(() => {
'use strict';
const { $, h, fmt, setText, section, pageTitle } = window.FIC;

window.FIC.pages.view3d = function buildView3d(root) {
  const stage = h('div', { class: 'stage', id: 'stage', 'aria-label': '3D view of the robot' });
  const note = h('div', { class: 'faint', id: 'stage-note', style: 'font-size:var(--fs-m);margin-top:8px', text: 'Loading the robot…' });
  root.append(
    ...pageTitle('3D view', 'The robot as the Studio sees it, posed from the live joint stream.'),
    h('div', { class: 'stage-wrap' }, stage, note),
    section('End-effector', { id: 'v3-aside' }, h('div', { class: 'kv', id: 'v3-pose' })));
  let mod = null, dead = false, themeKey = '';
  const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  function palette() {
    const key = css('--canvas') + css('--rule-strong');
    if (!mod || key === themeKey) return;
    themeKey = key;
    mod.setPalette({ background: null, grid: css('--rule-strong'), gridSoft: css('--rule') });
  }
  let robotKey = null, loading = false;
  const keyOf = s => (s && ((s.robots && s.robots.current) || (s.studio && s.studio.robot_display))) || '';
  async function load() {
    loading = true;
    try {
      const m = await import('/view3d.js');
      if (mod) { mod.unmount(); mod = null; themeKey = ''; }
      robotKey = keyOf(window.FIC.state());
      setText(note, 'Loading the robot…');
      await m.mount(stage);
      if (dead) { m.unmount(); return; }
      mod = m; setText(note, 'Drag to turn · scroll or pinch to zoom');
      palette();
      const s = window.FIC.state(); if (s) { m.setJoints(s.joints); m.setGripper(s.gripper); }
    } catch (err) {
      setText(note, 'No 3D view: ' + (err && err.message ? err.message : err));
    } finally { loading = false; }
  }
  load();
  const keys = [['x', 'X m', 3], ['y', 'Y m', 3], ['z', 'Z m', 3], ['rx', 'Rx °', 1], ['ry', 'Ry °', 1], ['rz', 'Rz °', 1]];
  const cells = {};
  for (const [k, label] of keys) { cells[k] = h('div', { class: 'v' }); $('#v3-pose').append(h('div', null, h('div', { class: 'k', text: label }), cells[k])); }
  window.addEventListener('resize', onResize);
  function onResize() { if (mod) mod.resize(); }
  return {
    update(s) {
      if (robotKey !== null && !loading && keyOf(s) !== robotKey) load();   // another robot selected
      if (mod) { mod.setJoints(s.joints); mod.setGripper(s.gripper); palette(); }
      const ee = s.pose && s.pose.actual;
      for (const [k, , d] of keys) setText(cells[k], ee ? fmt(ee[k], d) : '—');
      setText($('#v3-aside'), s.gripper && s.gripper.available ? 'gripper ' + fmt(s.gripper.percent, 0) + ' %' : 'no gripper');
    },
    dispose() { dead = true; window.removeEventListener('resize', onResize); if (mod) mod.unmount(); },
  };
};
})();
