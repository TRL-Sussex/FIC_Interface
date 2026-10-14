# shellcheck shell=sh
# ROS 2 and the FIC workspace for login shells too (e.g. `docker exec <ctr> bash -lc ...`).
if [ -n "${BASH_VERSION:-}" ] && [ -f /etc/fic/env.sh ]; then
    . /etc/fic/env.sh
fi
