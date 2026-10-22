#!/bin/zsh
# FIC Studio installer for macOS: double-click in Finder (it opens in Terminal), or run it.
# install.sh finds the system and runs its installer (macOS: fic_install/macos/install-fic-studio-macos.sh).
cd "$(dirname "$0")" || exit 1
bash ./install.sh "$@"
status=$?
echo
[ $status -eq 0 ] && echo "Done. Open FIC Control Studio from Launchpad or ~/Applications." || echo "The installer stopped (exit $status); see the messages above."
read -r "?Press Return to close this window."
exit $status
