# shellcheck shell=bash
# ROS 2 and the FIC workspace, for every shell and launcher in the image.
# Sourced by /etc/bash.bashrc, /etc/profile.d/fic.sh, `fic` and the desktop
# session; only once per shell.

if [ -z "${_FIC_ENV_LOADED:-}" ]; then
    # ROS's setup scripts read unset variables: never source them under set -u.
    case $- in *u*) _fic_nounset=1; set +u ;; *) _fic_nounset= ;; esac

    # shellcheck disable=SC1090,SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    # shellcheck disable=SC1091
    source "${FIC_ROOT}/fic_ws/install/setup.bash"

    # Extensions import one another as `Extensions.<name>`, which resolves
    # only with the repository root on the Python path.
    case ":${PYTHONPATH:-}:" in
        *":${FIC_ROOT}:"*) ;;
        *) export PYTHONPATH="${FIC_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" ;;
    esac

    if [ -n "${_fic_nounset}" ]; then set -u; fi
    unset _fic_nounset
    _FIC_ENV_LOADED=1
fi
