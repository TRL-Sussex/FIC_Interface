/* Users page: who may open the web remote, and what they may see.

Admins only — the server refuses /api/users to anyone else, and this page is
left out of the sidebar for everyone else, so neither half depends on the
other being right.

The store itself is web_remote/accounts.py: one JSON file shared between the
Studios of a lab, newest version winning, so an edit here reaches the others.
Passwords are scrypt-hashed and never sent back, so there is nothing to show
for one — only a way to set a new one. */
(() => {
'use strict';
const { h, pageTitle, post, toast, inlineConfirm } = window.FIC;

//: The page permissions an admin ticks per user, in the order accounts.PAGES has them.
const PAGE_LABELS = {
  control: 'Control', view3d: '3D view', dashboard: 'Dashboard',
  session: 'Processes', log: 'Logs', extensions: 'Extensions',
};

window.FIC.pages.users = function buildUsers(root) {
  const askHost = h('div');                 // inlineConfirm opens the question in place
  const box = h('div', { class: 'users-box' });
  root.append(...pageTitle('Users', 'Who may open the web remote, and what they may see.'), askHost, box);
  let users = [], pages = [], dead = false;

  async function load() {
    let r;
    try { r = await (await fetch('/api/users', { cache: 'no-store' })).json(); }
    catch (e) { r = { ok: false, message: String(e) }; }
    if (dead) return;
    if (!r || r.ok === false) {
      box.replaceChildren(h('div', { class: 'empty', text: r && r.error === 'admin_only'
        ? 'Only an admin may manage users.' : ((r && r.message) || 'Could not read the user list.') }));
      return;
    }
    users = r.users || [];
    pages = r.pages || Object.keys(PAGE_LABELS);
    render();
  }

  async function save(body, done) {
    const r = await post('/api/users', body);
    if (!r || r.ok === false) {
      toast((r && r.message) || (r && r.error) || 'Refused', true);
      return false;
    }
    users = r.users || users;
    render();
    if (done) toast(done);
    return true;
  }

  // ── One person ────────────────────────────────────────────────────────
  function card(u) {
    const isAdmin = u.role === 'admin';
    const rows = [];

    const role = h('select', { class: 'select' },
      h('option', { value: 'user', text: 'User' }),
      h('option', { value: 'admin', text: 'Admin' }));
    role.value = u.role;
    role.addEventListener('change', () => save({ user: u.name, role: role.value },
      u.name + ' is now ' + (role.value === 'admin' ? 'an admin' : 'a user')));
    rows.push(h('div', { class: 'xrow' }, h('label', { text: 'Role' }), role));

    // The ticks are always shown and always editable, for an admin too. The
    // role opens every page whatever is stored, but the stored list is what
    // the account falls back to the moment the role goes back to user — so
    // hiding it left an admin's row looking like it had nothing to set, and
    // hid a setting that does matter.
    const ticks = h('div', { class: 'users-pages' });
    for (const p of pages) {
      const cb = h('input', { type: 'checkbox' });
      cb.checked = (u.pages || []).includes(p);
      cb.addEventListener('change', () => {
        const want = pages.filter(x => ticks.querySelector('[data-p="' + x + '"]').checked);
        save({ user: u.name, pages: want });
      });
      cb.dataset.p = p;
      ticks.append(h('label', { class: 'toggle' }, cb, h('span', { class: 'track' }),
        h('span', { text: PAGE_LABELS[p] || p })));
    }
    rows.push(h('div', { class: 'xrow' }, h('label', { text: 'Pages' }), ticks));
    if (isAdmin) {
      rows.push(h('div', { class: 'xrow' },
        h('label', { text: '' }),
        h('div', { class: 'faint', text:
          'Being an admin already opens every page, so these ticks do not limit ' +
          u.name + ' now — they are what the account keeps if the role goes back to user.' })));
    }

    const sim = h('input', { type: 'checkbox' });
    sim.checked = !!(isAdmin ? u.stored_sim_only : u.sim_only);
    sim.addEventListener('change', () => save({ user: u.name, sim_only: sim.checked },
      sim.checked ? u.name + ' is limited to simulation' : u.name + ' may use a real arm'));
    rows.push(h('div', { class: 'xrow' }, h('label', { text: 'Hardware' }),
      h('div', {},
        h('label', { class: 'toggle' }, sim, h('span', { class: 'track' }),
          h('span', { text: 'Simulation only' })),
        isAdmin ? h('div', { class: 'faint', text: 'An admin is never limited to simulation.' }) : null)));

    if ((u.ext_hidden || []).length) {
      rows.push(h('div', { class: 'xrow' }, h('label', { text: 'Hidden extensions' }),
        h('div', { class: 'faint', text: u.ext_hidden.join(', ') + ' — their own choice, on the Extensions page' })));
    }

    const pw = h('input', { class: 'input', type: 'password', autocomplete: 'new-password',
                            placeholder: 'New password', 'aria-label': 'New password for ' + u.name });
    const setPw = h('button', { class: 'btn', type: 'button', text: 'Set password', onclick: async () => {
      if (!pw.value) { toast('Type a new password first', true); return; }
      if (await save({ user: u.name, password: pw.value }, 'Password set for ' + u.name)) pw.value = '';
    } });
    rows.push(h('div', { class: 'xrow' }, h('label', { text: 'Password' }),
      h('div', { class: 'btn-row' }, pw, setPw)));

    const del = h('button', { class: 'btn btn-danger', type: 'button', text: 'Remove', onclick: () => {
      inlineConfirm(askHost, {
        title: 'Remove ' + u.name + '?',
        lines: ['They will not be able to sign in again.', 'This cannot be undone.'],
        label: 'Remove', danger: true,
        onConfirm: () => save({ user: u.name, remove: true }, u.name + ' removed'),
      });
    } });
    rows.push(h('div', { class: 'btn-row users-del' }, del));

    return h('section', { class: 'section users-card' },
      h('div', { class: 'xhead' },
        h('h2', { text: u.name }),
        h('span', { class: 'chip', text: isAdmin ? 'admin' : (u.sim_only ? 'simulation only' : 'user') })),
      h('div', { class: 'xbody' }, ...rows));
  }

  // ── Add someone ───────────────────────────────────────────────────────
  function addCard() {
    const name = h('input', { class: 'input', type: 'text', placeholder: 'Name', autocomplete: 'off' });
    const pw = h('input', { class: 'input', type: 'password', placeholder: 'Password', autocomplete: 'new-password' });
    const go = h('button', { class: 'btn btn-primary', type: 'button', text: 'Add user', onclick: async () => {
      if (!name.value.trim()) { toast('A name, please', true); return; }
      if (!pw.value) { toast('A password, please', true); return; }
      if (await save({ user: name.value.trim(), password: pw.value, role: 'user', sim_only: true },
                     'Added ' + name.value.trim())) { name.value = ''; pw.value = ''; }
    } });
    return h('section', { class: 'section' },
      h('div', { class: 'xhead' }, h('h2', { text: 'Add a user' })),
      h('div', { class: 'xbody' },
        h('p', { class: 'faint', text: 'They start with the default pages, limited to simulation. Change that below once they exist.' }),
        h('div', { class: 'btn-row' }, name, pw, go)));
  }

  function render() {
    box.replaceChildren(addCard(), ...users.map(card));
  }

  load();
  // An object, not a bare function: the router treats a returned function as
  // update(state) and calls it as soon as the page is built, which would set
  // `dead` before the first fetch came back and leave the page empty.
  return {
    update() {},
    dispose() { dead = true; },
  };
};
})();
