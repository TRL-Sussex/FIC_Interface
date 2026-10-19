/* Remote clients (another computer, a phone): sign-in, sign-up, what this account may use,
   the "view only" and "offline" notes. The desktop window never runs any of this
   (FIC.local is true there). The server enforces every rule; this file only keeps the
   page from offering what it would refuse. */
(() => {
'use strict';
const { $, h, store, icon } = window.FIC;

// New page -> the account permission (accounts.PAGES) that opens it. Settings and Robot
// Config change the Studio's own configuration, so a remote client never sees them.
const NEEDS = { dashboard: 'dashboard', topic_monitor: 'dashboard', multi_robot: 'control', locomotion: 'control', tuning: 'control',
  safety: 'control', view3d: 'view3d', session: 'session', log: 'log', extensions: 'extensions', help: null, users: 'admin' };
const R = { auth: null, signup: false };

R.allows = id => {
  if (!(id in NEEDS)) return false;
  // Managing users is not one of accounts.PAGES — it is the admin role itself,
  // which is why it is spelled out here rather than ticked per account. The
  // server refuses /api/users to anyone else regardless.
  if (NEEDS[id] === 'admin') return !!(R.auth && R.auth.admin);
  return NEEDS[id] === null || !R.auth || (R.auth.pages || []).includes(NEEDS[id]);
};
R.viewOnly = () => !!R.auth && R.auth.control === false;
R.filterPages = pages => pages.filter(p => R.allows(p.id));
R.notAllowed = id => root => {
  root.append(h('h1', { class: 'title', text: 'Not available' }),
    h('p', { class: 'lede', text: id === 'settings' || id === 'robot_config'
      ? 'This page changes the Studio itself, so it is only in the Studio window.' : 'Your account does not include this page. Ask an admin.' }));
  return () => {};
};

async function signedIn(auth) {
  R.auth = auth;
  document.body.classList.remove('signed-out');
  document.body.classList.toggle('view-only', R.viewOnly());
  if (auth.name) {
    const out = h('button', { class: 'btn', type: 'button', text: 'Sign out', onclick: async () => { await window.FIC.post('/api/logout', {}); location.reload(); } });
    $('#menubar').insertBefore(h('div', { class: 'acct' }, h('span', { class: 'acct-name', text: auth.name }),
      auth.sim_only ? h('span', { class: 'chip', text: 'simulation only', title: 'Ask an admin for hardware access' }) : null, out), $('#theme'));
  }
  R.tick();
}

R.gate = async () => {
  let d = null;
  try { d = await (await fetch('/api/session', { cache: 'no-store' })).json(); } catch (e) { /* unreachable: look open, the state fetch says offline */ }
  R.signup = !!(d && d.signup);
  if (!d || !d.ok || d.auth.signed_in) { await signedIn(d && d.ok ? d.auth : { signed_in: true, pages: [], control: true }); if (!d || !d.ok) R.auth = null; return true; }
  showLogin('');
  return false;
};

function showLogin(msg) {
  document.body.classList.add('signed-out');
  let making = false;
  const user = h('input', { class: 'input', id: 'lg-user', autocomplete: 'username', autocapitalize: 'none', spellcheck: 'false', 'aria-label': 'User name', placeholder: 'user name' });
  user.value = store.get('fic.user') || '';
  const pass = h('input', { class: 'input', id: 'lg-pass', type: 'password', autocomplete: 'current-password', 'aria-label': 'Password', placeholder: 'password' });
  const say = h('div', { class: 'lg-msg', role: 'alert', text: msg || '' });
  const go = h('button', { class: 'btn btn-primary', type: 'submit', text: 'Sign in' });
  const swap = h('button', { class: 'btn', type: 'button', text: 'Create an account', hidden: !R.signup });
  const note = h('p', { class: 'faint lg-note', text: 'You will have the simulation only until an admin allows more.', hidden: true });
  swap.addEventListener('click', () => {
    making = !making; go.textContent = making ? 'Create account' : 'Sign in'; swap.textContent = making ? 'I have an account' : 'Create an account';
    pass.autocomplete = making ? 'new-password' : 'current-password'; note.hidden = !making; say.textContent = '';
  });
  const form = h('form', { class: 'lg-card', onsubmit: async ev => {
    ev.preventDefault(); go.disabled = true;
    const r = await window.FIC.post(making ? '/api/register' : '/api/login', { user: user.value.trim(), password: pass.value });
    go.disabled = false;
    if (!r.ok) {
      say.textContent = r.error === 'too_many_tries' ? 'Too many tries. Wait ' + (r.retry_after || 60) + ' seconds.'
        : r.error === 'offline' ? 'The Studio is not reachable.'
        : (r.message && r.error !== 'bad_login' ? r.message : 'Wrong user or password.');
      return;
    }
    store.set('fic.user', user.value.trim()); location.reload();
  } },
  h('div', { class: 'lg-brand' }, icon ? icon('diamond') : null, h('span', { text: 'FIC Studio' })),
  h('h1', { class: 'title', text: 'Sign in' }), h('p', { class: 'lede', text: 'Remote control of the Studio.' }),
  h('label', { class: 'field' }, h('span', { class: 'eyebrow', text: 'User' }), user),
  h('label', { class: 'field' }, h('span', { class: 'eyebrow', text: 'Password' }), pass),
  note, say, h('div', { class: 'btn-row' }, go, swap),
  h('div', { class: 'lg-estop' }, h('button', { class: 'estop', type: 'button', text: 'E-STOP', 'aria-label': 'Emergency stop', onclick: window.FIC.fireEstop }),
    h('span', { class: 'faint', text: 'Stops the robot without signing in.' })));
  $('#main').replaceChildren(form);
  (user.value ? pass : user).focus({ focusVisible: false });
}
R.signedOut = msg => showLogin(msg);

// One slim note under the bars: offline wins over view-only.
let offlineSince = 0;
R.tick = () => {
  const n = $('#rnote'); if (!n) return;
  if (document.body.classList.contains('signed-out')) { n.hidden = true; return; }
  const L = window.FIC.link;
  if (!L.open) offlineSince = offlineSince || Date.now(); else offlineSince = 0;
  let text = '';
  if (offlineSince && Date.now() - offlineSince > 1500) text = 'Offline. Reconnecting to the Studio...';
  else if (R.viewOnly()) text = 'View only: this account cannot command the robot. E-STOP still works.';
  else if (R.auth && R.auth.sim_only) text = 'Simulation only for this account. Settings, files and build stay in the Studio window.';
  n.hidden = !text; n.textContent = text; n.classList.toggle('bad', !!offlineSince);
};
setInterval(() => { if (!window.FIC.local) R.tick(); }, 1000);
window.FIC.remote = R;
})();
