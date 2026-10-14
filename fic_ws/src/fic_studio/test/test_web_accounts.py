"""Who may open the web remote, and what they may see."""
import json
import os

from fic_studio.web_remote.accounts import Accounts, hash_password, PAGES
import pytest


@pytest.fixture
def store(tmp_path):
    return Accounts(str(tmp_path / 'accounts.json'))


class TestPasswords:
    def test_a_password_is_never_stored(self, store):
        store.set_user('alice', password='correct horse', role='user')
        blob = json.dumps(store.as_dict())
        assert 'correct horse' not in blob
        assert store.get('alice')['hash'] and store.get('alice')['salt']

    def test_the_right_password_is_accepted_and_others_are_not(self, store):
        store.set_user('alice', password='correct horse')
        assert store.check('alice', 'correct horse')['name'] == 'alice'
        assert store.check('alice', 'wrong') is None
        assert store.check('alice', '') is None
        assert store.check('nobody', 'correct horse') is None

    def test_names_are_matched_whatever_the_case(self, store):
        store.set_user('Alice', password='pw')
        assert store.check('alice', 'pw') and store.check('ALICE', 'pw')

    def test_two_users_with_one_password_get_different_hashes(self, store):
        store.set_user('a', password='same')
        store.set_user('b', password='same')
        assert store.get('a')['hash'] != store.get('b')['hash']

    def test_hashing_needs_a_password(self):
        with pytest.raises(ValueError):
            hash_password('')


class TestPermissions:
    def test_a_new_user_sees_only_the_default_pages(self, store):
        store.set_user('bob', password='pw')
        assert set(store.allowed_pages(store.get('bob'))) < set(PAGES)
        assert not store.may_control(store.get('bob'))

    def test_an_admin_sees_everything(self, store):
        store.set_user('root', password='pw', role='admin', pages=['log'])
        user = store.get('root')
        assert set(store.allowed_pages(user)) == set(PAGES)
        assert store.may_control(user) and store.is_admin(user)

    def test_an_admin_can_grant_and_revoke_pages(self, store):
        store.set_user('bob', password='pw')
        store.set_user('bob', pages=['control', 'view3d', 'nonsense'])
        assert store.allowed_pages(store.get('bob')) == ['control', 'view3d']
        assert store.may_control(store.get('bob'))
        store.set_user('bob', pages=['view3d'])
        assert not store.may_control(store.get('bob'))

    def test_permissions_change_without_touching_the_password(self, store):
        store.set_user('bob', password='pw')
        before = store.get('bob')['hash']
        store.set_user('bob', pages=['log'], role='user')
        assert store.get('bob')['hash'] == before
        assert store.check('bob', 'pw')

    def test_a_new_user_needs_a_password(self, store):
        with pytest.raises(ValueError):
            store.set_user('ghost')
        with pytest.raises(ValueError):
            store.set_user('bob', password='pw', role='wizard')

    def test_the_admin_page_never_shows_hashes(self, store):
        store.set_user('alice', password='pw', role='admin')
        assert json.dumps(store.public()).find('hash') == -1
        assert store.public()[0]['name'] == 'alice'


class TestFirstRunAndRemoval:
    def test_the_first_run_makes_an_admin_with_a_password_to_show_once(self, store):
        password = store.ensure_admin()
        assert password and store.check('admin', password)
        assert store.ensure_admin() is None          # only when there is no admin

    def test_the_last_admin_cannot_be_removed(self, store):
        store.ensure_admin()
        with pytest.raises(ValueError):
            store.remove('admin')
        store.set_user('second', password='pw', role='admin')
        assert store.remove('admin')
        assert store.get('admin') is None

    def test_removing_someone_who_is_not_there(self, store):
        assert store.remove('nobody') is False


class TestFile:
    def test_saved_and_loaded_again(self, tmp_path):
        path = str(tmp_path / 'accounts.json')
        store = Accounts(path)
        store.set_user('alice', password='pw', role='admin')
        store.save()
        again = Accounts.load(path)
        assert again.check('alice', 'pw') and again.version == store.version

    def test_only_this_user_can_read_it(self, tmp_path):
        path = str(tmp_path / 'accounts.json')
        store = Accounts(path)
        store.set_user('alice', password='pw')
        store.save()
        assert oct(os.stat(path).st_mode & 0o777) == '0o600'

    def test_a_missing_or_broken_file_gives_an_empty_store(self, tmp_path):
        assert Accounts.load(str(tmp_path / 'nothing.json')).users == []
        broken = tmp_path / 'broken.json'
        broken.write_text('{ not json')
        assert Accounts.load(str(broken)).users == []

    def test_a_newer_store_from_another_studio_is_taken(self, store):
        store.set_user('alice', password='pw')
        other = Accounts('/tmp/other.json')
        other.set_user('bob', password='pw')
        other.version = store.version + 5
        assert store.merge(other.as_dict())
        assert store.get('bob') and store.get('alice') is None

    def test_an_older_or_empty_store_is_ignored(self, store):
        store.set_user('alice', password='pw')
        version = store.version
        assert not store.merge({'version': version - 1, 'users': [{'name': 'x'}]})
        assert not store.merge({'version': version + 9, 'users': []})
        assert not store.merge('nonsense')
        assert store.version == version and store.get('alice')


def test_at_the_same_version_the_older_list_wins(tmp_path):
    """Four Studios switched on together settle on one first admin."""
    mine = Accounts(str(tmp_path / 'mine.json'))
    mine.ensure_admin()
    theirs = Accounts(str(tmp_path / 'theirs.json'))
    theirs.ensure_admin()
    theirs.stamp = mine.stamp - 1.0              # their list was made first
    assert mine.merge(theirs.as_dict())
    assert mine.stamp == theirs.stamp
    # and it settles: neither takes the other's again
    assert not mine.merge(theirs.as_dict())
    assert not theirs.merge(mine.as_dict())


def test_a_newer_list_still_wins_whenever_it_was_stamped(tmp_path):
    mine = Accounts(str(tmp_path / 'mine.json'))
    mine.ensure_admin()
    theirs = Accounts(str(tmp_path / 'theirs.json'))
    theirs.ensure_admin()
    theirs.set_user('sam', password='sam-pass')  # version 2, stamped later
    assert mine.merge(theirs.as_dict())
    assert [u['name'] for u in mine.public()] == ['admin', 'sam']


def test_reset_admin_makes_a_password_for_the_first_admin(tmp_path):
    accounts = Accounts(str(tmp_path / 'a.json'))
    accounts.set_user('ada', password='first', role='admin')
    accounts.set_user('sam', password='sam-pass')
    name, password = accounts.reset_admin()
    assert name == 'ada' and len(password) >= 8
    assert accounts.check('ada', password) is not None
    assert accounts.check('ada', 'first') is None
    assert accounts.allowed_pages(accounts.get('ada')) == list(PAGES)


def test_reset_admin_takes_a_password_and_a_name(tmp_path):
    accounts = Accounts(str(tmp_path / 'a.json'))
    accounts.set_user('ada', password='first', role='admin')
    name, password = accounts.reset_admin(name='sam', password='chosen-one')
    assert (name, password) == ('sam', 'chosen-one')
    assert accounts.get('sam')['role'] == 'admin'       # sam is an admin now
    assert accounts.check('sam', 'chosen-one') is not None


def test_reset_admin_brings_back_an_admin_when_there_is_none(tmp_path):
    accounts = Accounts(str(tmp_path / 'a.json'))
    accounts.set_user('sam', password='sam-pass')
    name, password = accounts.reset_admin()
    assert name == 'admin'
    assert accounts.is_admin(accounts.check('admin', password))
    assert accounts.version == 2                        # the lab takes this one


def test_signing_yourself_up_gets_the_simulation_only(tmp_path):
    accounts = Accounts(str(tmp_path / 'a.json'))
    user = accounts.sign_up('yasir', 'a-good-password')
    assert user['role'] == 'user'
    assert accounts.sim_only(user) is True
    assert accounts.allowed_pages(user) == ['control', 'view3d', 'dashboard', 'session']
    assert accounts.check('yasir', 'a-good-password') is not None


def test_signing_up_refuses_a_taken_name_or_a_weak_password(tmp_path):
    accounts = Accounts(str(tmp_path / 'a.json'))
    accounts.sign_up('yasir', 'a-good-password')
    for name, password, why in (('yasir', 'another-one', 'taken'),
                                ('y', 'a-good-password', 'too short'),
                                ('sam', 'short', 'at least 8')):
        with pytest.raises(ValueError) as exc:
            accounts.sign_up(name, password)
        assert why in str(exc.value)


def test_an_admin_lifts_the_simulation_limit(tmp_path):
    accounts = Accounts(str(tmp_path / 'a.json'))
    accounts.sign_up('yasir', 'a-good-password')
    accounts.set_user('yasir', sim_only=False, pages=list(PAGES))
    assert accounts.sim_only(accounts.get('yasir')) is False
    assert accounts.public()[0]['sim_only'] is False


def test_an_admin_is_never_simulation_only(tmp_path):
    accounts = Accounts(str(tmp_path / 'a.json'))
    accounts.set_user('ada', password='first', role='admin', sim_only=True)
    assert accounts.sim_only(accounts.get('ada')) is False


class TestHiddenExtensions:
    """A person's own view of the extension list: hides, never reveals."""

    def test_nothing_is_hidden_until_someone_hides_it(self, store):
        store.set_user('alice', password='pw')
        assert Accounts.hidden_extensions(store.get('alice')) == []

    def test_hiding_is_remembered_sorted_and_deduplicated(self, store):
        store.set_user('alice', password='pw')
        store.set_hidden_extensions('alice', ['sequences', 'gaze', 'sequences', '  '])
        assert Accounts.hidden_extensions(store.get('alice')) == ['gaze', 'sequences']

    def test_an_empty_list_brings_them_all_back(self, store):
        store.set_user('alice', password='pw')
        store.set_hidden_extensions('alice', ['gaze'])
        store.set_hidden_extensions('alice', [])
        assert Accounts.hidden_extensions(store.get('alice')) == []

    def test_hiding_grants_nothing(self, store):
        """It must not be a way to widen your own account."""
        store.sign_up('student', 'password123')
        before = dict(store.get('student'))
        store.set_hidden_extensions('student', ['control', 'admin', 'gaze'])
        after = store.get('student')
        assert after['pages'] == before['pages']
        assert after['role'] == before['role']
        assert Accounts.sim_only(after) is True

    def test_an_unknown_name_is_refused_rather_than_created(self, store):
        with pytest.raises(ValueError):
            store.set_hidden_extensions('nobody', ['gaze'])
        assert store.get('nobody') is None

    def test_an_admin_may_set_it_for_someone_else(self, store):
        store.set_user('alice', password='pw')
        store.set_user('alice', ext_hidden=['teach_mode'])
        assert Accounts.hidden_extensions(store.get('alice')) == ['teach_mode']

    def test_it_survives_a_save_and_load(self, store):
        store.set_user('alice', password='pw')
        store.set_hidden_extensions('alice', ['gaze'])
        store.save()
        again = Accounts.load(store.path)
        assert Accounts.hidden_extensions(again.get('alice')) == ['gaze']

    def test_the_admin_page_can_see_it(self, store):
        store.set_user('alice', password='pw')
        store.set_hidden_extensions('alice', ['gaze'])
        row = [u for u in store.public() if u['name'] == 'alice'][0]
        assert row['ext_hidden'] == ['gaze']

    def test_changing_it_bumps_the_version_so_other_studios_take_it(self, store):
        store.set_user('alice', password='pw')
        before = store.version
        store.set_hidden_extensions('alice', ['gaze'])
        assert store.version > before


class TestPublicShowsStoredAndEffective:
    """An admin's row must still have something to set."""

    def test_an_admin_reports_its_stored_pages_not_just_the_effective_ones(self, store):
        store.set_user('root', password='pw', role='admin', pages=['log'])
        row = [u for u in store.public() if u['name'] == 'root'][0]
        assert row['pages'] == ['log']                   # what is stored
        assert set(row['effective_pages']) == set(PAGES)  # what the role grants

    def test_for_a_user_the_two_agree(self, store):
        store.set_user('alice', password='pw', pages=['log', 'view3d'])
        row = [u for u in store.public() if u['name'] == 'alice'][0]
        assert row['pages'] == row['effective_pages'] == ['view3d', 'log'] or \
               sorted(row['pages']) == sorted(row['effective_pages'])

    def test_an_admin_keeps_its_stored_pages_when_demoted(self, store):
        store.set_user('root', password='pw', role='admin', pages=['log'])
        store.set_user('root', role='user')
        assert store.allowed_pages(store.get('root')) == ['log']

    def test_stored_sim_only_survives_being_an_admin(self, store):
        store.set_user('alice', password='pw', sim_only=True)
        store.set_user('alice', role='admin')
        row = [u for u in store.public() if u['name'] == 'alice'][0]
        assert row['sim_only'] is False          # an admin is never limited
        assert row['stored_sim_only'] is True    # but the setting is still there
