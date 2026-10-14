"""Signing in to the web remote: sessions on top of the accounts store.

A browser posts a name and password once; it gets a random session token in a
cookie and sends that with every later request. Tokens live in memory, so
restarting a Studio signs everyone out, and they expire after ``LIFETIME_S``
of no use.

Kept apart from the store (``accounts``) and from the server so it can be
tested on its own, with the clock injected.
"""
import secrets
import time

#: A session with nothing done on it for this long is dropped.
LIFETIME_S = 12 * 3600
#: Cookie the browser sends back. HttpOnly, so no page script can read it.
COOKIE = 'fic_session'
#: After this many wrong passwords in a row, that name waits.
MAX_FAILURES = 8
LOCKOUT_S = 60.0


class Auth:
    """Sessions for one Studio's web remote."""

    def __init__(self, accounts, clock=time.monotonic):
        """Sign people in against ``accounts`` (an ``Accounts`` store)."""
        self.accounts = accounts
        self._clock = clock
        self._sessions = {}          # token -> [user name, last used]
        self._failures = {}          # name -> [count, when it locked]

    # ── signing in and out ───────────────────────────────────────────────
    def sign_up(self, name, password):
        """Make an account and sign it in; return its token. Raises ValueError."""
        self.accounts.sign_up(name, password)
        return self.login(name, password)

    def login(self, name, password):
        """Return a new session token, or None if the name or password is wrong."""
        key = str(name).strip().lower()
        if self.locked_for(key) > 0:
            return None
        user = self.accounts.check(name, password)
        if user is None:
            count, _ = self._failures.get(key, (0, 0.0))
            self._failures[key] = (count + 1, self._clock())
            return None
        self._failures.pop(key, None)
        token = secrets.token_urlsafe(32)
        self._sessions[token] = [user['name'], self._clock()]
        return token

    def locked_for(self, name):
        """Return the seconds this name must wait before trying again."""
        count, when = self._failures.get(str(name).strip().lower(), (0, 0.0))
        if count < MAX_FAILURES:
            return 0.0
        left = LOCKOUT_S - (self._clock() - when)
        return max(0.0, left)

    def logout(self, token):
        """End one session."""
        self._sessions.pop(token or '', None)

    def user_for(self, token):
        """Return the signed-in user for ``token``, or None; touches the session."""
        entry = self._sessions.get(token or '')
        if entry is None:
            return None
        name, last = entry
        now = self._clock()
        if now - last > LIFETIME_S:
            self._sessions.pop(token, None)
            return None
        entry[1] = now
        user = self.accounts.get(name)
        if user is None:                    # the account was removed meanwhile
            self._sessions.pop(token, None)
        return user

    def sessions(self):
        """Return the number of live sessions."""
        now = self._clock()
        self._sessions = {t: e for t, e in self._sessions.items() if now - e[1] <= LIFETIME_S}
        return len(self._sessions)

    def sign_out_everyone(self):
        """Drop every session (an admin changed who may do what)."""
        self._sessions.clear()

    # ── what the page is told about itself ───────────────────────────────
    def describe(self, user):
        """Return what the page needs to know about the person using it."""
        if user is None:
            return {'signed_in': False, 'pages': [], 'admin': False, 'control': False}
        return {
            'signed_in': True,
            'name': user['name'],
            'role': user.get('role', 'user'),
            'pages': self.accounts.allowed_pages(user),
            'admin': self.accounts.is_admin(user),
            'control': self.accounts.may_control(user),
            'sim_only': self.accounts.sim_only(user),
        }
