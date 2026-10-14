"""Tests for the contact run recorder."""

import csv
import json

from fic_contact.recorder import Recorder


def test_writes_requested_columns(tmp_path):
    """The CSV header must be exactly the configured columns."""
    rec = Recorder(['t', 'link'])
    rec.add({'t': 0.0, 'link': 3})
    path = tmp_path / 'data.csv'
    rec.write_csv(str(path))
    with open(path, newline='', encoding='utf-8') as handle:
        rows = list(csv.reader(handle))
    assert rows[0] == ['t', 'link']
    assert rows[1] == ['0.0', '3']


def test_absent_values_are_empty_not_zero(tmp_path):
    """A channel with no sample is blank, never a fabricated zero."""
    rec = Recorder(['t', 'link', 'force'])
    rec.add({'t': 0.0, 'link': 3})
    path = tmp_path / 'data.csv'
    rec.write_csv(str(path))
    with open(path, newline='', encoding='utf-8') as handle:
        rows = list(csv.reader(handle))
    assert rows[1][2] == ''


def test_missing_channels_are_reported():
    """Channels that never produced a sample are named."""
    rec = Recorder(['t', 'link', 'force'])
    rec.add({'t': 0.0, 'link': 3})
    assert rec.missing_channels() == ['force']


def test_meta_is_written_as_json(tmp_path):
    """meta.json must round-trip."""
    rec = Recorder(['t'])
    path = tmp_path / 'meta.json'
    rec.write_meta(str(path), {'robot': 'KinovaGen3_7DOF'})
    with open(path, encoding='utf-8') as handle:
        assert json.load(handle)['robot'] == 'KinovaGen3_7DOF'
