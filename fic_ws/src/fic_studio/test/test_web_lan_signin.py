"""The Docker launcher's --lan: FIC_WEB_ADMIN_PASSWORD turns sign-in on."""

from fic_studio.mixins import web_remote_mixin
from fic_studio.mixins.web_remote_mixin import ADMIN_PASSWORD_ENV, WebRemoteMixin
from fic_studio.web_remote.accounts import Accounts


class _Studio(WebRemoteMixin):
    """Just enough of the Studio for the accounts setup."""

    def __init__(self, settings):
        self.settings = settings
        self.logged = []

    def log_message(self, level, message, source=''):
        self.logged.append((level, message))


def test_settings_keep_sign_in_off_without_the_password(monkeypatch):
    monkeypatch.delenv(ADMIN_PASSWORD_ENV, raising=False)
    studio = _Studio({'web_remote': {'accounts': False}})
    assert studio._web_settings()['accounts'] is False


def test_the_password_turns_sign_in_on(monkeypatch):
    monkeypatch.setenv(ADMIN_PASSWORD_ENV, 'Xy12ab34')
    studio = _Studio({'web_remote': {'accounts': False, 'signup': True}})
    assert studio._web_settings()['accounts'] is True
    assert studio._web_settings()['signup'] is False      # admin only


def test_admin_gets_the_password_every_start(monkeypatch, tmp_path):
    path = str(tmp_path / 'accounts.json')
    monkeypatch.setattr(web_remote_mixin, 'ACCOUNTS_FILE', path)
    monkeypatch.setenv(ADMIN_PASSWORD_ENV, 'first-start')
    auth = _Studio({})._web_open_accounts()
    assert auth is not None
    assert Accounts.load(path).check('admin', 'first-start') is not None

    # The next start brings a new password, and the old one stops working.
    monkeypatch.setenv(ADMIN_PASSWORD_ENV, 'second-start')
    _Studio({})._web_open_accounts()
    saved = Accounts.load(path)
    assert saved.is_admin(saved.check('admin', 'second-start'))
    assert saved.check('admin', 'first-start') is None
