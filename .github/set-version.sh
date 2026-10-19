#!/usr/bin/env bash
# Gives a release its number, everywhere at once. Run on Development just before releasing it to
# main (numbers are given at release only: between releases nothing is bumped and the Studio shows
# "<last release> · dev <commit>").
#
#   .github/set-version.sh 5.12.0 "Short title of the release"
#
# Writes the number into fic_studio/version.py (RELEASE), fic_studio's package.xml and setup.py,
# fic_hardware's package.xml, the docs badge (fic_docs/index.html) and the Settings page subtitle,
# and turns CHANGELOG.md's "## [Unreleased]" heading into "## [<version>] — <date> — <title>" with
# a new empty "## [Unreleased]" above it. The "What's new" box on the docs home page and the docs
# changelog are written by hand. Commit the result, then release.
set -euo pipefail
ver=${1:-}
title=${2:-}
if ! [[ "$ver" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || [ -z "$title" ]; then
    echo "usage: $0 X.Y.Z \"title\"" >&2
    exit 2
fi
cd "$(git rev-parse --show-toplevel)"
studio=fic_ws/src/fic_studio
sed -i -E "s/^RELEASE = '[0-9.]+'/RELEASE = '$ver'/" $studio/fic_studio/version.py
sed -i -E "0,/<version>[0-9.]+<\/version>/s//<version>$ver<\/version>/" $studio/package.xml fic_ws/src/fic_hardware/package.xml
sed -i -E "s/^    version='[0-9.]+',/    version='$ver',/" $studio/setup.py
sed -i -E "s/(<span class=\"version-badge\"[^>]*>)v[0-9.]+/\1v$ver/" fic_docs/index.html
sed -i -E "s/(settings\.yaml<\/code> &mdash; )v[0-9.]+/\1v$ver/" fic_docs/pages/settings.html
if ! grep -q '^## \[Unreleased\]' CHANGELOG.md; then
    echo "CHANGELOG.md has no '## [Unreleased]' entry to release" >&2
    exit 1
fi
python3 - "$ver" "$title" <<'EOF'
import datetime, sys
ver, title = sys.argv[1], sys.argv[2]
s = open('CHANGELOG.md').read()
head = '## [%s] — %s — %s' % (ver, datetime.date.today().isoformat(), title)
s = s.replace('## [Unreleased]', '## [Unreleased]\n\n' + head, 1)
open('CHANGELOG.md', 'w').write(s)
EOF
grep -n "RELEASE = \|<version>\|version='\|version-badge\|&mdash; v" $studio/fic_studio/version.py $studio/package.xml \
    fic_ws/src/fic_hardware/package.xml $studio/setup.py fic_docs/index.html fic_docs/pages/settings.html
grep -n -m2 '^## \[' CHANGELOG.md
