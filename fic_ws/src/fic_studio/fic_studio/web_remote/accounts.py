"""Who may open the web remote, and what they may see there.

The web remote used to be open to anyone who could reach the port. With
accounts it asks for a name and password, and each person sees only the pages
an admin allowed them.

* an **admin** may do everything, including managing users;
* anyone else sees the pages listed in their own ``pages``, and may only
  command the robot with the ``control`` page among them;
* and anyone with the ``extensions`` page may put individual extensions away
  with ``ext_hidden``, which is their own view and grants nothing.

The store is one JSON file (``accounts.json``, mode 0600) holding a version, the
users and their permissions. Passwords are never stored — only a scrypt hash
with a per-user salt. The same file is shared between a lab's Studios, newest
version winning, so an admin edits users in one place (see ``sync``).

Nothing here is Qt or ROS: it is the plain store, tested on its own.
"""
import hashlib
import json
import os
import secrets
import time

#: Pages the web remote can show. An admin ticks these per user.
PAGES = ('control', 'view3d', 'dashboard', 'session', 'log', 'extensions')
#: What a new user gets when an admin does not say otherwise.
DEFAULT_PAGES = ('view3d', 'dashboard')
#: What someone who signed themselves up gets: the whole page, but simulation
#: only, until an admin says otherwise.
SIGNUP_PAGES = ('control', 'view3d', 'dashboard', 'session')
ROLES = ('admin', 'user')
#: What an admin picks for a person, in place of ticking pages one by one. The
#: store keeps role, pages and sim_only as before (older Studios of the lab read
#: it unchanged); a preset only fills them in, and ``kind`` reads them back.
#:   observer      watches: 3D view, dashboard and log; can never command a robot
#:   sim_operator  commands robots in simulation only
#:   operator      commands real arms too
#:   admin         everything, and manages users
PRESETS = {
    'observer': {'role': 'user', 'pages': ('view3d', 'dashboard', 'log'), 'sim_only': True},
    'sim_operator': {'role': 'user', 'pages': ('control', 'view3d', 'dashboard', 'session', 'log'),
                     'sim_only': True},
    'operator': {'role': 'user', 'pages': ('control', 'view3d', 'dashboard', 'session', 'log',
                                           'extensions'), 'sim_only': False},
    'admin': {'role': 'admin', 'pages': PAGES, 'sim_only': False},
}
KINDS = ('observer', 'sim_operator', 'operator', 'admin')
#: scrypt parameters (interactive login, ~100 ms on a robot PC).
_N, _R, _P = 2 ** 14, 8, 1


def hash_password(password, salt=None):
    """Return (salt, hash) as hex for ``password``."""
    if not password:
        raise ValueError('a password is required')
    salt = salt or secrets.token_hex(16)
    digest = hashlib.scrypt(password.encode('utf-8'), salt=bytes.fromhex(salt),
                            n=_N, r=_R, p=_P, dklen=32)
    return salt, digest.hex()


class Accounts:
    """The users of one lab, loaded from and saved to a JSON file."""

    def __init__(self, path, users=None, version=0, stamp=0.0):
        """Hold ``users`` (a list of dicts); ``path`` is where ``save`` writes."""
        self.path = str(path)
        self.users = list(users or [])
        self.version = int(version)
        #: When this list was last changed. Two labs' Studios that made their
        #: own first admin at the same version settle on the older one.
        self.stamp = float(stamp or 0.0)

    # ── the file ─────────────────────────────────────────────────────────
    @classmethod
    def load(cls, path):
        """Read the store, or return an empty one when the file is absent or broken."""
        try:
            with open(path) as handle:
                blob = json.load(handle)
        except (OSError, ValueError):
            return cls(path)
        if not isinstance(blob, dict):
            return cls(path)
        users = [u for u in blob.get('users', []) if isinstance(u, dict) and u.get('name')]
        return cls(path, users, blob.get('version', 0), blob.get('stamp', 0))

    def save(self):
        """Write the store, readable only by this user (it holds password hashes)."""
        blob = self.as_dict()
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = self.path + '.new'
        with open(tmp, 'w') as handle:
            json.dump(blob, handle, indent=2, sort_keys=True)
        os.chmod(tmp, 0o600)
        os.replace(tmp, self.path)

    def as_dict(self):
        """Return the store as JSON-able data (hashes included: for Studios, not browsers)."""
        return {'version': self.version, 'stamp': self.stamp,
                'updated': time.time(), 'users': self.users}

    # ── users ────────────────────────────────────────────────────────────
    def get(self, name):
        """Return the user record for ``name``, or None."""
        lowered = str(name).strip().lower()
        for user in self.users:
            if user['name'].lower() == lowered:
                return user
        return None

    def check(self, name, password):
        """Return the user if the password matches, else None."""
        user = self.get(name)
        if not user or not password:
            return None
        try:
            _, digest = hash_password(password, user.get('salt', ''))
        except (ValueError, TypeError):
            return None
        return user if secrets.compare_digest(digest, user.get('hash', '')) else None

    def set_user(self, name, password=None, role=None, pages=None, sim_only=None,
                 ext_hidden=None):
        """Create or update a user; return the record.

        Only what is given changes, so an admin can set permissions without
        knowing anyone's password.
        """
        name = str(name).strip()
        if not name:
            raise ValueError('a name is required')
        user = self.get(name)
        if user is None:
            if not password:
                raise ValueError('a new user needs a password')
            user = {'name': name, 'role': 'user', 'pages': list(DEFAULT_PAGES)}
            self.users.append(user)
        if password:
            user['salt'], user['hash'] = hash_password(password)
        if role is not None:
            if role not in ROLES:
                raise ValueError('role must be one of %s' % (ROLES,))
            user['role'] = role
        if pages is not None:
            user['pages'] = [p for p in pages if p in PAGES]
        if sim_only is not None:
            user['sim_only'] = bool(sim_only)
        if ext_hidden is not None:
            user['ext_hidden'] = sorted({str(i)[:64] for i in ext_hidden if str(i).strip()})
        self._changed()
        return user

    def apply_preset(self, name, preset, password=None):
        """Give ``name`` one of :data:`PRESETS` (creating them when a password is given)."""
        if preset not in PRESETS:
            raise ValueError('preset must be one of %s' % (KINDS,))
        p = PRESETS[preset]
        user = self.get(name)
        if (user is not None and user.get('role') == 'admin' and p['role'] != 'admin'
                and len(self.admins()) <= 1):
            raise ValueError('this is the only admin')
        return self.set_user(name, password=password, role=p['role'], pages=list(p['pages']),
                             sim_only=p['sim_only'])

    def set_hidden_extensions(self, name, ids):
        """Set which extensions ``name`` has hidden from their own page list.

        Separate from :meth:`set_user` because this is the one thing a person
        changes about their *own* account: which of the extensions this Studio
        loads they want in their sidebar. It is a display preference, so it
        needs no admin and no control lease — and it cannot grant anything,
        only hide. An unknown name is refused rather than created.
        """
        user = self.get(name)
        if user is None:
            raise ValueError('no such user')
        user['ext_hidden'] = sorted({str(i)[:64] for i in ids if str(i).strip()})
        self._changed()
        return user

    def sign_up(self, name, password):
        """Make an account for someone who asked for one themselves.

        They get the page, and the simulation only: nothing they do can reach
        an arm until an admin turns that off. A name already taken is refused
        rather than joined, so nobody can walk into someone else's account.
        """
        name = str(name).strip()
        if len(name) < 2:
            raise ValueError('that name is too short')
        if len(str(password or '')) < 8:
            raise ValueError('a password of at least 8 characters, please')
        if self.get(name) is not None:
            raise ValueError('that name is taken')
        return self.set_user(name, password=password, role='user',
                             pages=list(SIGNUP_PAGES), sim_only=True)

    def remove(self, name):
        """Delete a user. The last admin cannot be removed."""
        user = self.get(name)
        if user is None:
            return False
        if user.get('role') == 'admin' and len(self.admins()) <= 1:
            raise ValueError('this is the only admin')
        self.users.remove(user)
        self._changed()
        return True

    def _changed(self):
        self.version += 1
        self.stamp = time.time()

    def admins(self):
        """Return the admin users."""
        return [u for u in self.users if u.get('role') == 'admin']

    def ensure_admin(self, name='admin'):
        """Create ``name`` as an admin with a random password when there is no admin.

        Returns the password to show once (a lab's first Studio has to hand the
        first login over somehow), or None when an admin already exists.
        """
        if self.admins():
            return None
        password = secrets.token_urlsafe(9)
        self.set_user(name, password=password, role='admin', pages=list(PAGES))
        return password

    def reset_admin(self, name=None, password=None):
        """Give an admin a new password; return (name, password).

        For a lab that is locked out of its own web remote: the Studio is the
        machine itself, so it may set this without knowing the old password.
        ``password`` None means make one up. Without ``name`` it is the first
        admin, or a new ``admin`` account when every one of them is gone.
        """
        if name is None:
            admins = self.admins()
            name = admins[0]['name'] if admins else 'admin'
        password = password or secrets.token_urlsafe(9)
        self.set_user(name, password=password, role='admin', pages=list(PAGES))
        return name, password

    # ── what a user may see ──────────────────────────────────────────────
    @staticmethod
    def allowed_pages(user):
        """Return the pages ``user`` may see (an admin: all of them)."""
        if not user:
            return []
        if user.get('role') == 'admin':
            return list(PAGES)
        return [p for p in user.get('pages', ()) if p in PAGES]

    @staticmethod
    def may_control(user):
        """Return True if ``user`` may command the robot."""
        return 'control' in Accounts.allowed_pages(user)

    @staticmethod
    def is_admin(user):
        """Return True for an admin."""
        return bool(user) and user.get('role') == 'admin'

    @staticmethod
    def sim_only(user):
        """Return True if this user may only use a simulated robot."""
        if not user or user.get('role') == 'admin':
            return False
        return bool(user.get('sim_only'))

    @staticmethod
    def kind(user):
        """What this person amounts to: 'admin', 'operator', 'sim_operator' or 'observer'.

        Read from what is stored, so an account whose pages were ticked by hand
        still gets a name: without the control page it is an observer, with it
        an operator (simulation only or not).
        """
        if not user:
            return 'observer'
        if user.get('role') == 'admin':
            return 'admin'
        if not Accounts.may_control(user):
            return 'observer'
        return 'sim_operator' if Accounts.sim_only(user) else 'operator'

    @staticmethod
    def hidden_extensions(user):
        """Return the extension ids this user keeps out of their page list.

        Hidden, not allowed: the machine's ``extensions.enabled`` decides what
        the Studio loads at all, and this only decides what one person sees of
        it. Empty by default, so an extension switched on for the lab shows up
        for everyone until they put it away themselves.
        """
        if not user:
            return []
        return [str(i) for i in user.get('ext_hidden', ()) or ()]

    def public(self):
        """Return the user list for the admin page: no hashes, no salts.

        ``pages`` is what is *stored* on the account and ``effective_pages``
        what the person actually gets, which differ for an admin: the role
        opens every page whatever the stored list says. Reporting only the
        effective list made an admin's row look like it had nothing to set,
        and made a stored list that does matter — the moment the role is
        changed back to user — invisible.
        """
        return [{'name': u['name'], 'role': u.get('role', 'user'),
                 'pages': [p for p in u.get('pages', ()) if p in PAGES],
                 'effective_pages': self.allowed_pages(u),
                 'sim_only': self.sim_only(u),
                 'stored_sim_only': bool(u.get('sim_only')),
                 'ext_hidden': self.hidden_extensions(u),
                 'kind': self.kind(u)}
                for u in sorted(self.users, key=lambda u: u['name'].lower())]

    # ── sharing between the Studios of one lab ───────────────────────────
    def merge(self, blob):
        """Take ``blob`` (another Studio's store) when it wins; return True if taken.

        Newer wins. At the same version — four Studios that each made their own
        first admin when the lab was switched on — the older list wins, so they
        settle on one instead of each keeping its own.
        """
        if not isinstance(blob, dict):
            return False
        version, stamp = int(blob.get('version', -1)), float(blob.get('stamp', 0) or 0)
        if version < self.version:
            return False
        if version == self.version and not (self.stamp and stamp and stamp < self.stamp):
            return False
        users = [u for u in blob.get('users', []) if isinstance(u, dict) and u.get('name')]
        if not users:
            return False
        self.users = users
        self.version, self.stamp = version, stamp
        return True
