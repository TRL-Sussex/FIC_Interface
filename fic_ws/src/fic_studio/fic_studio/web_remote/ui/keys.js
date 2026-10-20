/* Keyboard Control in the web UI: the page reads the drive keys itself.

   The classic window that used to read them is hidden behind this page, and
   only the page knows when someone is typing in one of its boxes. So the page
   tells the Studio which drive keys are held: at once on every change, again
   every 100 ms while one is down, and an empty list when the last is let go.
   The Studio lets go of any key it has not heard about for 0.3 s
   (keyboard_mixin.WEB_STALE_S), so a dropped connection cannot leave the arm
   moving. The mapping itself (which key moves what, how fast) is the Studio's:
   keyboard/driver.py. */
(() => {
'use strict';
const REPEAT_MS = 100;
const CODES = {
  ArrowUp: 'UP', ArrowDown: 'DOWN', Comma: ',', Period: '.', Tab: 'TAB',
  ShiftLeft: 'SHIFT', ShiftRight: 'SHIFT', ControlLeft: 'CTRL', ControlRight: 'CTRL',
};
const HOLD = new Set(['W', 'S', 'A', 'D', 'Q', 'E', 'J', 'L', 'O', 'C', 'UP', 'DOWN', ',', '.']);
const MODS = new Set(['SHIFT', 'CTRL']);
const TAPS = new Set(['TAB', '1', '2', '3', '4', '5', '6', '7', '8', '9']);
const MODE_LETTERS = new Set(['J', 'C', 'F', 'P', 'E']);

const KEYS = { ArrowUp: 'UP', ArrowDown: 'DOWN', ',': ',', '.': '.', Tab: 'TAB', Shift: 'SHIFT', Control: 'CTRL' };

/** The driver's name for a key, by its place on the keyboard (Shift turns 1 into "!", the code stays).
    Events with no code (some synthetic and remote-desktop ones) fall back to the key. */
function name(e) {
  const code = (e && e.code) || '';
  if (!code) {
    const key = (e && e.key) || '';
    if (KEYS[key]) return KEYS[key];
    return /^[a-zA-Z0-9]$/.test(key) ? key.toUpperCase() : null;
  }
  if (CODES[code]) return CODES[code];
  let m = /^Key([A-Z])$/.exec(code);
  if (m) return m[1];
  m = /^(?:Digit|Numpad)([0-9])$/.exec(code);
  return m ? m[1] : null;
}

/** Say whether the Studio's keyboard driver uses this key. */
function isOurs(n) { return HOLD.has(n) || MODS.has(n) || TAPS.has(n); }

/** The mode keys, which need Shift: 'j' 'c' (joint / Cartesian), '1'-'4' (mode), 'f' 'p' 'e' (options); else null. */
function modeKey(e) {
  if (!e || !e.shiftKey || e.ctrlKey || e.altKey || e.metaKey) return null;
  const n = name(e);
  if (MODE_LETTERS.has(n)) return n.toLowerCase();
  return /^[1-4]$/.test(n || '') ? n : null;
}

/** What is held, sent through ``send(keys, tap)``. ``timers`` is for the tests. */
function Driver(send, timers) {
  const T = timers || { set: (f, ms) => setInterval(f, ms), clear: id => clearInterval(id) };
  const held = new Set();
  let timer = null;
  const list = () => [...held].sort();
  const driving = () => [...held].some(k => HOLD.has(k));
  function pace() {
    if (driving() && timer === null) timer = T.set(() => send(list()), REPEAT_MS);
    else if (!driving() && timer !== null) { T.clear(timer); timer = null; }
  }
  function change(fn) {
    const was = driving();
    fn();
    if (was || driving()) send(list());
    pace();
  }
  return {
    down(n) {
      if (TAPS.has(n)) { send(list(), n); return; }
      if (isOurs(n) && !held.has(n)) change(() => held.add(n));
    },
    up(n) { if (held.has(n)) change(() => held.delete(n)); },
    clear() { if (held.size) change(() => held.clear()); },
    held: list,
  };
}

const api = { name, isOurs, modeKey, Driver, MODS, REPEAT_MS };
if (typeof module !== 'undefined' && module.exports) module.exports = api;
if (typeof window !== 'undefined') (window.FIC = window.FIC || {}).keys = api;
})();
