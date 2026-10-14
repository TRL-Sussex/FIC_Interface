#!/bin/bash
# web-admin-password.sh — set the web remote admin password from a terminal
# (a headless board has no Studio window to reset it from).
#
#   bash fic_install/web-admin-password.sh            new random password, printed
#   bash fic_install/web-admin-password.sh 'secret'   that password
#
# Restart Studio afterwards so it reads it: systemctl --user restart fic-studio
set -u
FIC_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYTHONPATH="$FIC_ROOT/fic_ws/src/fic_studio" python3 - "${1:-}" <<'PYEOF'
import os, sys
from fic_studio.web_remote.accounts import Accounts
path = os.path.expanduser('~/.fic_studio/accounts.json')
acc = Accounts.load(path)
name, password = acc.reset_admin(password=sys.argv[1] or None)
acc.save()
print("web remote login:  %s / %s" % (name, password))
PYEOF
