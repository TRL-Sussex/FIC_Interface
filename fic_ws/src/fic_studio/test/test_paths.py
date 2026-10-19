"""Where the Studio keeps machine state and recordings (fic_studio.paths), and the settings
file being written whole."""
import os
import stat

from fic_studio import paths
import pytest
import yaml


@pytest.fixture
def clean_env(monkeypatch):
    for k in (paths.EXPERIMENTS_ENV, paths.STATE_ENV, paths.SETTINGS_ENV, 'XDG_DATA_HOME'):
        monkeypatch.delenv(k, raising=False)
    return monkeypatch


def test_experiments_stay_in_a_writable_checkout(clean_env, tmp_path):
    repo = tmp_path / 'repo' / 'experiments'
    assert paths.experiments_dir(repo, settings_path=os.devnull) == str(repo)


def test_a_read_only_checkout_records_in_the_data_home(clean_env, tmp_path):
    ro = tmp_path / 'ro'
    ro.mkdir()
    ro.chmod(stat.S_IRUSR | stat.S_IXUSR)
    clean_env.setenv('XDG_DATA_HOME', str(tmp_path / 'xdg'))
    try:
        if os.access(ro, os.W_OK):              # running as root: nothing is read-only
            pytest.skip('root can write anywhere')
        got = paths.experiments_dir(ro / 'experiments', settings_path=os.devnull)
        assert got == str(tmp_path / 'xdg' / 'fic_studio' / 'experiments')
    finally:
        ro.chmod(stat.S_IRWXU)


def test_the_setting_and_the_environment_move_it(clean_env, tmp_path):
    s = tmp_path / 'settings.yaml'
    s.write_text(yaml.safe_dump({'paths': {'experiments': str(tmp_path / 'chosen')}}))
    assert paths.experiments_dir(tmp_path / 'repo', settings_path=str(s)) == str(tmp_path / 'chosen')
    clean_env.setenv(paths.EXPERIMENTS_ENV, str(tmp_path / 'env'))
    assert paths.experiments_dir(tmp_path / 'repo', settings_path=str(s)) == str(tmp_path / 'env')


def test_state_and_settings_follow_their_variables(clean_env, tmp_path):
    assert paths.state_dir() == os.path.expanduser('~/.fic_studio')
    assert paths.settings_file() == os.path.expanduser('~/.robot_selector_gui_settings.yaml')
    clean_env.setenv(paths.STATE_ENV, str(tmp_path / 'st'))
    clean_env.setenv(paths.SETTINGS_ENV, str(tmp_path / 's.yaml'))
    assert paths.state_path('audit.log') == str(tmp_path / 'st' / 'audit.log')
    assert paths.settings_file() == str(tmp_path / 's.yaml')


def test_settings_are_saved_whole(tmp_path):
    from fic_studio.mixins.settings_mixin import SettingsMixin

    class W(SettingsMixin):
        SETTINGS_FILE = str(tmp_path / 's.yaml')
        logs = []

        def log_message(self, level, text, source=''):
            self.logs.append((level, text))
    w = W()
    w.settings = {'general': {'auto_save': True}, 'paths': {'experiments': ''}}
    w._save_settings()
    assert yaml.safe_load(open(w.SETTINGS_FILE)) == w.settings
    assert not os.path.exists(w.SETTINGS_FILE + '.new')
    assert w.logs[-1][0] == 'INFO'


def test_settings_page_offers_the_folder_and_refuses_a_relative_one(clean_env, tmp_path):
    from fic_studio.web_remote import settings_page
    from fic_studio.web_remote.common import Refused
    spec = {'key': 'paths.experiments', 'label': 'Experiments folder', 'type': 'text'}
    with pytest.raises(Refused):
        settings_page._validate(spec, 'relative/dir')
    assert settings_page._validate(spec, str(tmp_path / 'new')) == str(tmp_path / 'new')
    assert 'paths.experiments' in settings_page.RESTART_KEYS
