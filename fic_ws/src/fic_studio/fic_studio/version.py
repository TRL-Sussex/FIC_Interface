"""The Studio's version.

Numbers are given at release only. When Development is released to main (and main is published),
the release takes the next number (a new minor version for features, a patch version for fixes
only): ``.github/set-version.sh`` writes it everywhere and turns CHANGELOG.md's "Unreleased" entry
into that release's. Between releases nothing is bumped; a build that is not a release shows the
last release and the commit it was built from (``5.11.0 · dev 85617ecb``), so two builds can still
be told apart. Compatibility numbers keep their own versions: the FIC-link protocol
(``fic_link.h``) and the MCU firmware.
"""
import subprocess

#: The last release: the version of a released tree, the base of a development build.
RELEASE = '6.0.0'

_build = None


def build():
    """The commit a development build was made from; '' for a release (HEAD tagged v<RELEASE>) or
    a tree without git (a container, an installed copy)."""
    global _build
    if _build is None:
        _build = ''
        try:
            from fic_studio.config import FIC_INTERFACE_DIR
            git = ['git', '-C', FIC_INTERFACE_DIR]
            tags = subprocess.run(git + ['tag', '--points-at', 'HEAD'], capture_output=True, text=True,
                                  timeout=2).stdout.split()
            if 'v' + RELEASE not in tags:
                _build = subprocess.run(git + ['rev-parse', '--short=8', 'HEAD'], capture_output=True,
                                        text=True, timeout=2).stdout.strip()
        except Exception:
            _build = ''
    return _build


def display():
    """What the window title, the splash and About show: '5.11.0', or '5.11.0 · dev 85617ecb'."""
    b = build()
    return RELEASE + (' · dev ' + b if b else '')
