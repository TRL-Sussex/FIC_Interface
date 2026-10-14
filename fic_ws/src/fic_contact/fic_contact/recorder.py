"""Buffer contact-run samples and write data.csv plus meta.json.

A channel that never produced a sample is written as an empty cell and
listed under ``missing_channels`` — never as a zero, which would be
indistinguishable from a real no-contact reading.
"""

import csv
import json


class Recorder:
    """Column-oriented run buffer."""

    def __init__(self, columns):
        """Configure the recorder with its exact column list."""
        self._columns = list(columns)
        self._rows = []
        self._seen = set()

    def add(self, row):
        """Append one sample; absent keys stay absent."""
        self._rows.append(dict(row))
        self._seen.update(k for k, v in row.items() if v is not None)

    def missing_channels(self):
        """Columns that never received a value."""
        return [c for c in self._columns if c not in self._seen]

    def write_csv(self, path):
        """Write the buffered rows as a flat SI-unit CSV."""
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(
                handle, fieldnames=self._columns, restval='')
            writer.writeheader()
            for row in self._rows:
                writer.writerow(
                    {k: v for k, v in row.items() if v is not None})

    def write_meta(self, path, meta):
        """Write the run's metadata, including missing channels."""
        payload = dict(meta)
        payload['missing_channels'] = self.missing_channels()
        payload['samples'] = len(self._rows)
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2)
