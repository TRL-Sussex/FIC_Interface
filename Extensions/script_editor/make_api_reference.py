"""
Write web/api_reference.json, the API list of the Script Editor's web page.

    python3 -m Extensions.script_editor.make_api_reference    (from the repository root)

The list is robot_api.api_reference(), as on the Classic page. test_web_page.py
fails when the file is out of date.
"""
import json
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from Extensions.script_editor.robot_api import RobotAPI, api_reference  # noqa: E402

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web', 'api_reference.json')


def build():
    """The JSON text of the API list."""
    api = RobotAPI(host=None, node=None, rid='robot1', target='simulation',
                   marshal=lambda fn, block=True, timeout=10.0: fn(),
                   is_stopped=lambda: False)
    return json.dumps({'entries': api_reference(api)}, indent=1, ensure_ascii=False) + '\n'


def main():
    """Write the file."""
    os.makedirs(os.path.dirname(PATH), exist_ok=True)
    with open(PATH, 'w', encoding='utf-8') as fh:
        fh.write(build())
    print('Wrote', PATH)


if __name__ == '__main__':
    main()
