# Copyright 2025 Adnan Takriti
# SPDX-License-Identifier: MIT
"""Sharing the user list between the Studios of one lab."""
import json

from fic_studio.web_remote.accounts import Accounts
from fic_studio.web_remote.sync import AccountsSync, SHARE_S


class FakeNode:
    """Enough of a ROS node to run the sync: it remembers what was published."""

    def __init__(self):
        self.published = []
        self.on_message = None
        self.destroyed = []

    def create_publisher(self, _type, topic, _depth):
        node = self

        class Pub:
            def publish(self, msg):
                node.published.append(json.loads(msg.data))
        return Pub()

    def create_subscription(self, _type, topic, callback, _depth):
        self.on_message = callback
        return object()

    def destroy_publisher(self, handle):
        self.destroyed.append(handle)

    def destroy_subscription(self, handle):
        self.destroyed.append(handle)


class Msg:
    def __init__(self, data):
        self.data = data


def store(tmp_path, name='a.json'):
    accounts = Accounts(str(tmp_path / name))
    accounts.set_user('admin', password='first-one', role='admin')
    return accounts


def test_share_publishes_the_store(tmp_path):
    node = FakeNode()
    sync = AccountsSync(node, store(tmp_path), clock=lambda: 100.0)
    assert sync.share(force=True)
    assert node.published[-1]['users'][0]['name'] == 'admin'
    assert node.published[-1]['version'] == 1


def test_share_is_rate_limited_but_can_be_forced(tmp_path):
    now = [100.0]
    node = FakeNode()
    sync = AccountsSync(node, store(tmp_path), clock=lambda: now[0])
    assert sync.share()
    assert not sync.share()                       # too soon
    assert sync.share(force=True)                 # an edit goes out at once
    now[0] += SHARE_S + 0.1
    assert sync.share()


def test_a_newer_store_wins_and_is_saved(tmp_path):
    node = FakeNode()
    mine = store(tmp_path)
    changed = []
    sync = AccountsSync(node, mine, on_changed=lambda: changed.append(True))
    theirs = Accounts(str(tmp_path / 'b.json'))
    theirs.set_user('admin', password='first-one', role='admin')
    theirs.set_user('sam', password='sam-pass', pages=['view3d'])
    node.on_message(Msg(json.dumps(theirs.as_dict())))
    assert [u['name'] for u in sync.accounts.public()] == ['admin', 'sam']
    assert changed == [True]
    # what it took is on disk, so a restart keeps it
    assert [u['name'] for u in Accounts.load(mine.path).public()] == ['admin', 'sam']


def test_an_older_or_equal_store_is_ignored(tmp_path):
    node = FakeNode()
    mine = store(tmp_path)
    mine.set_user('sam', password='sam-pass')     # version 2
    changed = []
    AccountsSync(node, mine, on_changed=lambda: changed.append(True))
    older = Accounts(str(tmp_path / 'b.json'))
    older.set_user('admin', password='other', role='admin')   # version 1
    node.on_message(Msg(json.dumps(older.as_dict())))
    assert [u['name'] for u in mine.public()] == ['admin', 'sam']
    assert changed == []


def test_rubbish_on_the_topic_is_ignored(tmp_path):
    node = FakeNode()
    mine = store(tmp_path)
    AccountsSync(node, mine)
    for data in ('not json', '[]', '{}', '{"version": 99, "users": []}'):
        node.on_message(Msg(data))
    assert [u['name'] for u in mine.public()] == ['admin']


def test_close_gives_the_handles_back(tmp_path):
    node = FakeNode()
    AccountsSync(node, store(tmp_path)).close()
    assert len(node.destroyed) == 2
