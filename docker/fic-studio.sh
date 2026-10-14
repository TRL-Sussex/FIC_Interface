#!/usr/bin/env bash
#
# FIC Studio in Docker: macOS and Linux launcher.
# (Windows: use fic-studio.cmd, or fic-studio.ps1 in PowerShell.)
#
#   ./fic-studio.sh              Start FIC Studio and open it in your browser
#   ./fic-studio.sh stop         Stop it (your settings and files are kept)
#   ./fic-studio.sh restart      Stop, then start again
#   ./fic-studio.sh update       Download the newest version and restart
#   ./fic-studio.sh logs         Show what it is doing (Ctrl-C to leave)
#   ./fic-studio.sh shell        Open a terminal inside it
#   ./fic-studio.sh status       Is it running, which version, what is saved
#   ./fic-studio.sh reset        Forget all settings and saved robots
#   ./fic-studio.sh uninstall    Remove FIC Studio from Docker completely
#
# Options (for start, restart and update):
#   --native        Linux: open the windows on your own desktop instead of
#                   in the browser (uses your GPU if Docker can reach it)
#   --dev           Start the Dev Console instead of the Studio
#   --desktop       Start only the desktop (launch programs from its taskbar)
#   --port N        Browser port on this computer (default 6080)
#   --web-port N    Port of the Studio's web remote (default 8088)
#   --lan           Let phones on your network open the Studio's web remote
#                   (they sign in as admin, with a password printed at start)
#   --folder DIR    Shared folder (default ~/FIC_Studio)
#   --image REF     Image to run (default ghcr.io/trl-sussex/fic_interface:latest)
#   --no-browser    Do not open the browser
#
# The desktop gets a new random password every start, which the browser link
# carries, so other users of this computer cannot open it.
#
# Everything the Studio saves is kept in a Docker volume (fic-studio-home);
# recordings, datasets and your own extensions go to the shared folder.

set -euo pipefail

IMAGE="${FIC_IMAGE:-ghcr.io/trl-sussex/fic_interface:latest}"
NAME="${FIC_CONTAINER:-fic-studio}"
VOLUME="${FIC_VOLUME:-fic-studio-home}"
FOLDER="${FIC_FOLDER:-${HOME}/FIC_Studio}"
PORT="${FIC_PORT:-6080}"
WEB_PORT="${FIC_WEB_PORT:-8088}"
NATIVE=0
LAN=0
OPEN_BROWSER=1
MODE=studio

OS="$(uname -s)"

say()  { printf '%s\n' "$*"; }
step() { printf '\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33mWarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31mError:\033[0m %s\n' "$*" >&2; exit 1; }

usage() { sed -n '3,/^$/p' "$0" | sed 's/^# \{0,1\}//'; }

# ───────────────────────────────────────────────────────────── arguments ───
COMMAND=start
if [ $# -gt 0 ] && [[ "$1" != -* ]]; then COMMAND="$1"; shift; fi
while [ $# -gt 0 ]; do
    case "$1" in
        --native)     NATIVE=1 ;;
        --dev)        MODE=dev ;;
        --desktop)    MODE=desktop ;;
        --lan)        LAN=1 ;;
        --no-browser) OPEN_BROWSER=0 ;;
        --port)       PORT="${2:?--port needs a number}"; shift ;;
        --web-port)   WEB_PORT="${2:?--web-port needs a number}"; shift ;;
        --folder)     FOLDER="${2:?--folder needs a path}"; shift ;;
        --image)      IMAGE="${2:?--image needs an image reference}"; shift ;;
        -h|--help)    usage; exit 0 ;;
        *) die "unknown option '$1' (see ./fic-studio.sh --help)" ;;
    esac
    shift
done
if [ "${NATIVE}" = 1 ] && [ "${MODE}" = desktop ]; then
    die "--desktop needs the browser desktop; use --native on its own, or with --dev."
fi

# ──────────────────────────────────────────────────────────────── docker ───
ensure_docker() {
    if ! command -v docker >/dev/null 2>&1; then
        say "Docker is not installed."
        if [ "${OS}" = "Darwin" ]; then
            say "Install Docker Desktop for Mac: https://docs.docker.com/desktop/setup/install/mac-install/"
        else
            say "Install Docker Engine, then log out and back in:"
            say "  curl -fsSL https://get.docker.com | sudo sh"
            say "  sudo usermod -aG docker \$USER"
        fi
        exit 1
    fi

    local err
    if err="$(docker info 2>&1 >/dev/null)"; then return 0; fi

    if [[ "${err}" == *"permission denied"* ]]; then
        die "you are not allowed to use Docker yet. Run
    sudo usermod -aG docker \$USER
then log out and back in, and try again."
    fi

    if [ "${OS}" = "Darwin" ]; then
        step "Starting Docker Desktop"
        open -ga Docker 2>/dev/null || die "could not start Docker Desktop; start it from Applications."
    elif command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files docker.service >/dev/null 2>&1 \
            && ! systemctl is-active --quiet docker; then
        die "the Docker service is not running. Start it with:  sudo systemctl start docker"
    elif command -v docker-desktop >/dev/null 2>&1 || [ -d /opt/docker-desktop ]; then
        step "Starting Docker Desktop"
        systemctl --user start docker-desktop 2>/dev/null || true
    else
        die "Docker is not running:
${err}"
    fi

    local _
    for _ in $(seq 1 90); do
        docker info >/dev/null 2>&1 && return 0
        sleep 2
    done
    die "Docker did not start within 3 minutes. Start it by hand and try again."
}

container_state() {
    # (For a missing container docker prints an empty line, then fails.)
    local state
    state="$(docker container inspect -f '{{.State.Status}}' "${NAME}" 2>/dev/null)" || state=""
    state="$(printf '%s' "${state}" | tr -d '[:space:]')"
    echo "${state:-none}"
}

check_resources() {
    local cpus mem_gb
    cpus="$(docker info --format '{{.NCPU}}' 2>/dev/null || echo 0)"
    mem_gb="$(( $(docker info --format '{{.MemTotal}}' 2>/dev/null || echo 0) / 1024 / 1024 / 1024 ))"
    if [ "${cpus}" -gt 0 ] && [ "${cpus}" -lt 4 ]; then
        warn "Docker can use only ${cpus} CPU(s); the simulation runs better with 4 or more."
        [ "${OS}" = "Darwin" ] && warn "Raise it in Docker Desktop > Settings > Resources."
    fi
    if [ "${mem_gb}" -gt 0 ] && [ "${mem_gb}" -lt 4 ]; then
        warn "Docker has only about ${mem_gb} GB of memory; 4 GB or more is recommended."
    fi
}

pull_image() {
    step "Downloading FIC Studio (${IMAGE}); the first time takes a few minutes"
    local out
    if ! out="$(docker pull "${IMAGE}" 2>&1 | tee /dev/stderr)"; then
        case "${out}" in
            *"manifest unknown"*|*"not found"*|*"does not exist"*)
                die "${IMAGE} does not exist (yet). Check the name and tag, or use --image." ;;
            *denied*|*unauthorized*)
                die "not allowed to download ${IMAGE}. If it is private, log in first:
    docker login ${IMAGE%%/*}" ;;
            *)
                die "could not download ${IMAGE} (see the message above)." ;;
        esac
    fi
}

ensure_image() {
    docker image inspect "${IMAGE}" >/dev/null 2>&1 || pull_image
}

# ─────────────────────────────────────────────────────── start the Studio ───
PASSWORD=""
browser_url() {
    echo "http://localhost:${PORT}/vnc.html${PASSWORD:+?password=${PASSWORD}}"
}

new_password() {
    LC_ALL=C tr -dc 'A-Za-z0-9' < /dev/urandom 2>/dev/null | head -c 8 || true
}

container_env() {
    docker container inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${NAME}" 2>/dev/null \
        | sed -n "s/^$1=//p"
}

# rootless Docker and Docker Desktop for Linux map the host user to root in
# the container: running as root there is running as the host user.
host_is_container_root() {
    local info
    info="$(docker info --format '{{.OperatingSystem}} {{json .SecurityOptions}}' 2>/dev/null || true)"
    case "${info}" in *rootless*) return 0 ;; esac
    # Docker Desktop for Linux maps you to root; from WSL on Windows it does not.
    if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then return 1; fi
    grep -qi 'docker desktop' <<< "${info}"
}

open_url() {
    [ "${OPEN_BROWSER}" = 1 ] || return 0
    if [ "${OS}" = "Darwin" ]; then
        # (macOS hands the link to the browser without a command line.)
        open "$1" >/dev/null 2>&1 || true
    elif command -v xdg-open >/dev/null 2>&1 && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
        # A browser started by xdg-open keeps its link on its command line,
        # which every user of this computer can read. So open a page that only
        # you can read, which forwards to the link, and delete it shortly.
        # (In the shared folder: snap browsers cannot read /tmp.)
        local page="${FOLDER}/open-fic-studio.html"
        ( umask 077
          printf '<!DOCTYPE html><meta charset="utf-8"><title>FIC Studio</title>\n<script>location.replace(%s)</script>\n' \
              "\"$1\"" > "${page}" ) || return 0
        xdg-open "${page}" >/dev/null 2>&1 &
        ( sleep 60; rm -f "${page}" ) >/dev/null 2>&1 &
    fi
}

lan_address() {
    if [ "${OS}" = "Darwin" ]; then
        ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || true
    else
        ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<NF;i++) if ($i=="src") print $(i+1)}' || true
    fi
}

native_args() {
    [ "${OS}" = "Linux" ] || die "--native works on Linux only. On macOS the Studio opens in the browser."
    [ -n "${DISPLAY:-}" ] || die "--native needs a desktop session (DISPLAY is not set)."
    [ -d /tmp/.X11-unix ] || die "--native: no X11 socket in /tmp/.X11-unix."

    # Allow the container (which runs as you) to open windows on your display.
    if command -v xhost >/dev/null 2>&1; then
        xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 \
            || warn "xhost failed; windows may not be allowed to open."
    else
        warn "xhost is not installed (package x11-xserver-utils); windows may not be allowed to open."
    fi

    RUN_ARGS+=(-e FIC_DISPLAY=host -e "DISPLAY=${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix)
    # (The entrypoint adds the user to the groups of the GPU device nodes.)
    # The host's IPC namespace, so X11 shared memory (used by software
    # OpenGL) reaches the right segments; with a private /dev/shm, so ROS 2's
    # shared-memory files stay in the container.
    # shellcheck disable=SC2054  # the commas belong to --mount's value
    RUN_ARGS+=(--ipc=host --mount type=tmpfs,destination=/dev/shm,tmpfs-size=2147483648)

    # GPU: Intel/AMD through Mesa, NVIDIA through the NVIDIA container toolkit.
    if [ -d /dev/dri ]; then
        RUN_ARGS+=(--device /dev/dri)
    fi
    if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
        RUN_ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
        say "Using the NVIDIA GPU."
    fi
}

start() {
    ensure_docker
    local state
    state="$(container_state)"

    if [ "${state}" = "running" ]; then
        say "FIC Studio is already running."
        if docker container inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${NAME}" \
                | grep -qx 'FIC_DISPLAY=host'; then
            say "Its windows are on your desktop."
        else
            PORT="$(docker port "${NAME}" 6080/tcp 2>/dev/null | head -n1 | sed 's/.*://' || echo "${PORT}")"
            PASSWORD="$(container_env FIC_VNC_PASSWORD)"
            say "Open $(browser_url)"
            open_url "$(browser_url)"
        fi
        WEB_PORT="$(docker port "${NAME}" 8088/tcp 2>/dev/null | head -n1 | sed 's/.*://' || echo "${WEB_PORT}")"
        lan_message "$(container_env FIC_WEB_ADMIN_PASSWORD)"
        say "To start it with different options:  ./fic-studio.sh restart [options]"
        return 0
    fi

    ensure_image
    check_resources

    [ "${state}" = "none" ] || docker rm -f "${NAME}" >/dev/null

    mkdir -p "${FOLDER}"
    FOLDER="$(cd "${FOLDER}" && pwd)"
    [ "${NATIVE}" = 1 ] || PASSWORD="$(new_password)"

    RUN_ARGS=(
        --name "${NAME}"
        --hostname fic-studio
        --detach
        -v "${VOLUME}:/home/fic"
        -v "${FOLDER}:/home/fic/FIC_Studio"
        -e "FIC_WEB_REMOTE_PORT=${WEB_PORT}"
    )
    # The password, and the link carrying it, go through the environment
    # (-e NAME): a command line is visible to every user of this computer.
    export FIC_URL FIC_VNC_PASSWORD
    FIC_URL="$(browser_url)"
    FIC_VNC_PASSWORD="${PASSWORD}"
    RUN_ARGS+=(-e FIC_URL)
    [ -n "${PASSWORD}" ] && RUN_ARGS+=(-e FIC_VNC_PASSWORD)

    # Linux: run as you, so files in the shared folder are yours. (Your login
    # group, not the current one: after `newgrp docker` that is `docker`.)
    if [ "${OS}" = "Linux" ]; then
        if host_is_container_root; then
            RUN_ARGS+=(-e FIC_UID=0 -e FIC_GID=0)
        else
            local gid
            gid="$(id -g "$(id -un 2>/dev/null)" 2>/dev/null || id -g)"
            RUN_ARGS+=(-e "FIC_UID=$(id -u)" -e "FIC_GID=${gid}")
        fi
    fi

    # The web remote (the Studio's phone page).
    if [ "${LAN}" = 1 ]; then
        local ip
        ip="$(lan_address)"
        RUN_ARGS+=(-p "${WEB_PORT}:8088")
        RUN_ARGS+=(-e "FIC_WEB_REMOTE_HOSTS=${ip:+${ip},}localhost")
        # Open to the network, so sign-in is on: the account admin gets a new
        # password every start (through the environment, like the desktop's).
        export FIC_WEB_ADMIN_PASSWORD
        FIC_WEB_ADMIN_PASSWORD="$(new_password)"
        RUN_ARGS+=(-e FIC_WEB_ADMIN_PASSWORD)
    else
        RUN_ARGS+=(-p "127.0.0.1:${WEB_PORT}:8088")
    fi

    if [ "${NATIVE}" = 1 ]; then
        native_args
    else
        # Qt, the X server and ROS 2 shared memory need more than 64 MB.
        RUN_ARGS+=(--shm-size 2g)
        # Only this computer can reach the desktop, and only with the
        # password of this start.
        RUN_ARGS+=(-p "127.0.0.1:${PORT}:6080")
    fi

    step "Starting FIC Studio"
    if ! docker run "${RUN_ARGS[@]}" "${IMAGE}" "${MODE}" >/dev/null; then
        docker rm -f "${NAME}" >/dev/null 2>&1 || true
        die "Docker could not start FIC Studio (see the message above). If a port is
in use, choose another with --port N (browser) or --web-port N (web remote)."
    fi

    if [ "${NATIVE}" = 1 ]; then
        wait_until_running
        say ""
        say "FIC Studio is starting on your desktop."
    else
        wait_for_desktop
        say ""
        say "FIC Studio is running:  $(browser_url)"
        say "(The link includes this session's password; ./fic-studio.sh shows it again.)"
        open_url "$(browser_url)"
    fi
    lan_message "${FIC_WEB_ADMIN_PASSWORD:-}"
    say "Your shared folder:     ${FOLDER}"
    say "Stop it with:           ./fic-studio.sh stop"
}

# Where phones open the web remote, and the sign-in they need (--lan only).
lan_message() {
    [ -n "$1" ] || return 0
    local ip
    ip="$(lan_address)"
    say "Phones on your network: http://${ip:-<this computer>}:${WEB_PORT}/  (sign in as admin, password $1)"
}

wait_until_running() {
    sleep 3
    if [ "$(container_state)" != "running" ]; then
        docker logs --tail 40 "${NAME}" >&2 || true
        die "FIC Studio stopped while starting (see the messages above)."
    fi
}

# The desktop's page answers (on 127.0.0.1 only, and never via a proxy).
desktop_answers() {
    if command -v curl >/dev/null 2>&1; then
        curl --noproxy '*' -fsS -o /dev/null --max-time 5 "http://127.0.0.1:${PORT}/vnc.html" 2>/dev/null
    else
        docker exec "${NAME}" /usr/local/bin/fic-healthcheck >/dev/null 2>&1
    fi
}

wait_for_desktop() {
    local _
    printf 'Waiting for the desktop'
    for _ in $(seq 1 120); do
        if [ "$(container_state)" != "running" ]; then
            printf '\n'
            docker logs --tail 40 "${NAME}" >&2 || true
            die "FIC Studio stopped while starting (see the messages above)."
        fi
        if desktop_answers; then
            printf ' ready.\n'
            return 0
        fi
        printf '.'
        sleep 1
    done
    printf '\n'
    warn "the desktop is taking longer than usual; try $(browser_url) in a moment."
}

# ─────────────────────────────────────────────────────────────── commands ───
confirm() {
    local reply
    read -r -p "$1 [y/N] " reply
    [[ "${reply}" =~ ^[Yy] ]]
}

# Run a command inside the container as `fic`, with its groups.
exec_as_fic() {
    docker exec "${NAME}" gosu fic "$@"
}

# The same, attached to this terminal (or to piped input).
exec_as_fic_interactive() {
    # (No empty-array expansion: macOS's bash 3.2 rejects it under set -u.)
    if [ -t 0 ] && [ -t 1 ]; then
        docker exec -it "${NAME}" gosu fic "$@"
    else
        docker exec -i "${NAME}" gosu fic "$@"
    fi
}

case "${COMMAND}" in
    start)
        start ;;
    stop)
        ensure_docker
        if [ "$(container_state)" = "running" ]; then
            step "Stopping FIC Studio (your settings are saved)"
            docker stop -t 20 "${NAME}" >/dev/null
        else
            say "FIC Studio is not running."
        fi ;;
    restart)
        ensure_docker
        # Make sure the image can be used before stopping the running one.
        ensure_image
        [ "$(container_state)" = "none" ] || docker stop -t 20 "${NAME}" >/dev/null
        start ;;
    update)
        ensure_docker
        pull_image
        if [ "$(container_state)" != "none" ]; then
            step "Replacing the running version (your settings are kept)"
            docker stop -t 20 "${NAME}" >/dev/null 2>&1 || true
            docker rm "${NAME}" >/dev/null
        fi
        start ;;
    logs)
        ensure_docker
        [ "$(container_state)" != "none" ] \
            || die "FIC Studio has not been started yet (start it with ./fic-studio.sh)."
        docker logs -f --tail 200 "${NAME}" ;;
    shell)
        ensure_docker
        [ "$(container_state)" = "running" ] || die "FIC Studio is not running (start it with ./fic-studio.sh)."
        exec_as_fic_interactive fic shell ;;
    status)
        ensure_docker
        state="$(container_state)"
        say "Container: ${state}"
        image_ref="${IMAGE}"
        if [ "${state}" != "none" ]; then
            docker container inspect -f 'Image:     {{.Config.Image}}' "${NAME}"
            image_ref="$(docker container inspect -f '{{.Image}}' "${NAME}")"
        fi
        created="$(docker image inspect -f '{{.Created}}' "${image_ref}" 2>/dev/null | tr -d '\n' || true)"
        if [ -n "${created}" ]; then
            say "Created:   ${created}"
        else
            say "Image:     ${IMAGE} (not downloaded yet)"
        fi
        if [ "${state}" = "running" ]; then
            exec_as_fic fic-state status || true
        fi ;;
    reset)
        ensure_docker
        confirm "Forget all FIC Studio settings, tuning and robots added in the Studio? (The shared folder ${FOLDER} is kept.)" \
            || { say "Nothing changed."; exit 0; }
        docker rm -f "${NAME}" >/dev/null 2>&1 || true
        if docker volume inspect "${VOLUME}" >/dev/null 2>&1 \
                && ! docker volume rm "${VOLUME}" >/dev/null; then
            die "could not remove the settings volume ${VOLUME} (see the message above)."
        fi
        say "Done. Start again with ./fic-studio.sh" ;;
    uninstall)
        ensure_docker
        confirm "Remove the FIC Studio container, its settings volume and the image? (The shared folder ${FOLDER} is kept.)" \
            || { say "Nothing changed."; exit 0; }
        docker rm -f "${NAME}" >/dev/null 2>&1 || true
        if docker volume inspect "${VOLUME}" >/dev/null 2>&1; then
            docker volume rm "${VOLUME}" >/dev/null && say "Removed the settings volume ${VOLUME}." \
                || warn "could not remove the settings volume ${VOLUME}."
        fi
        if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
            docker image rm "${IMAGE}" >/dev/null && say "Removed the image ${IMAGE}." \
                || warn "could not remove the image ${IMAGE} (is another container using it?)."
        fi
        say "FIC Studio has been removed from Docker." ;;
    help)
        usage ;;
    *)
        die "unknown command '${COMMAND}' (see ./fic-studio.sh --help)" ;;
esac
