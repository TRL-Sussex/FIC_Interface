# syntax=docker/dockerfile:1
#
# FIC Studio: simulation image
#
# One image for Windows, macOS (Apple Silicon and Intel) and Linux. It is built
# for linux/amd64 and linux/arm64 and published as a single multi-arch tag, so
# every machine pulls a native build.
#
#   * The GUI runs on a virtual desktop (TigerVNC) and is shown in the browser
#     through noVNC at http://localhost:6080. The host needs only Docker.
#     On Linux the same image can also show its windows on the host's own
#     desktop (FIC_DISPLAY=host, see docker/fic-studio.sh --native).
#   * The simulator is `mujoco_native`, the Studio's CPU MuJoCo backend, on
#     the `mujoco` Python wheel (native on amd64 and arm64). The optional GPU
#     backend (mujoco_newton) needs CUDA and is left out (.dockerignore).
#   * Real-robot control is out of scope: no vendor driver SDKs are included.
#
# Build it with docker/build.sh (local) or let CI publish it
# (.github/workflows/docker-image.yml). Run it with docker/fic-studio.sh,
# docker/fic-studio.ps1 or docker/compose.yaml.

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

# The MuJoCo release the simulator (and the contact twin, the playground and
# the model tools) run on. Pinned so an image rebuild cannot change physics.
ARG MUJOCO_VERSION=3.10.0
ARG NOVNC_VERSION=1.7.0
ARG NOVNC_SHA256=b1003a11b6e6e8d8f7f5e5586daae7f8ca651d8aee0aa155ff9ac841c48f52c6

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ─────────────────────────────────────────────── build + ROS dependencies ───
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake pkg-config git curl ca-certificates \
        python3-dev python3-pip python3-setuptools python3-numpy python3-yaml \
        python3-colcon-common-extensions \
        libeigen3-dev libyaml-cpp-dev libtinyxml2-dev \
        ros-${ROS_DISTRO}-pinocchio \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-std-srvs \
        ros-${ROS_DISTRO}-urdfdom-headers \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────── desktop, GUI and runtime tools ───
RUN apt-get update && apt-get install -y --no-install-recommends \
        # Qt for the Studio; QtWebEngine for the in-container docs browser.
        python3-pyqt5 python3-pyqt5.qtwebengine \
        python3-scipy python3-matplotlib python3-tk \
        # Software OpenGL (Mesa llvmpipe): no host GPU is needed anywhere.
        libgl1 libglx-mesa0 libgl1-mesa-dri libegl1 libegl-mesa0 libglu1-mesa \
        mesa-utils \
        # The virtual desktop: X server + VNC in one process, window manager,
        # taskbar, and the websocket bridge noVNC connects through.
        tigervnc-standalone-server tigervnc-tools websockify \
        openbox tint2 xterm pcmanfm adwaita-icon-theme \
        x11-utils x11-xserver-utils xdotool dbus-x11 xdg-utils \
        file shared-mime-info librsvg2-common \
        # Fonts named in the Studio's style sheets, and emoji for the
        # extension icons in the sidebar.
        fonts-dejavu-core fonts-ubuntu fonts-jetbrains-mono \
        fonts-roboto-unhinted fonts-noto-color-emoji \
        # Tools the Studio calls: pkill (stop/E-STOP), ip (web remote),
        # ping (teleop latency).
        procps iproute2 iputils-ping \
        tini gosu sudo less nano patchelf \
    # Script Editor's sidebar icon is an Egyptian hieroglyph: take just that
    # font from fonts-noto-core instead of installing the whole package.
    && cd /tmp && apt-get download fonts-noto-core \
    && dpkg-deb -x fonts-noto-core_*.deb /tmp/noto \
    && install -D -m 0644 \
         /tmp/noto/usr/share/fonts/truetype/noto/NotoSansEgyptianHieroglyphs-Regular.ttf \
         /usr/local/share/fonts/NotoSansEgyptianHieroglyphs-Regular.ttf \
    && rm -rf /tmp/noto /tmp/fonts-noto-core_*.deb && fc-cache -f >/dev/null \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────── pip packages ───
# numpy stays at the Ubuntu version (ROS, scipy and matplotlib are built
# against it) and setuptools below 80 (80 removed what
# `colcon build --symlink-install` needs). The constraint file stays active
# in the image, so a later `pip install` cannot break either.
COPY docker/rootfs/etc/fic/pip-constraints.txt /etc/fic/pip-constraints.txt
ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_CONSTRAINT=/etc/fic/pip-constraints.txt \
    PIP_NO_CACHE_DIR=1 \
    PIP_ROOT_USER_ACTION=ignore
RUN pip3 install "mujoco==${MUJOCO_VERSION}" "ikpy>=3.4" \
    && python3 -c "import mujoco, numpy; print('mujoco', mujoco.__version__, 'numpy', numpy.__version__)"

# noVNC: the browser client. Vendored from the release tag because the
# Ubuntu package predates the defaults.json it is configured with.
RUN curl -fsSL -o /tmp/novnc.tgz \
        "https://github.com/novnc/noVNC/archive/refs/tags/v${NOVNC_VERSION}.tar.gz" \
    && echo "${NOVNC_SHA256}  /tmp/novnc.tgz" | sha256sum -c - \
    && mkdir -p /opt/novnc \
    && tar -xzf /tmp/novnc.tgz -C /opt/novnc --strip-components=1 \
    && rm /tmp/novnc.tgz \
    && cd /opt/novnc \
    # No "Leave site?" prompt: closing or reloading the page loses nothing,
    # the session keeps running in the container and the page reconnects.
    && grep -q 'window.addEventListener("beforeunload", UI.handleBeforeUnload);' app/ui.js \
    && sed -i 's|window.addEventListener("beforeunload", UI.handleBeforeUnload);|/* FIC Studio: no leave-page prompt */|' app/ui.js \
    # The page is FIC Studio, not noVNC (noVNC's licence files are kept).
    && grep -q '^const PAGE_TITLE = "noVNC";' app/ui.js \
    && sed -i 's|^const PAGE_TITLE = "noVNC";|const PAGE_TITLE = "FIC Studio";|' app/ui.js \
    && grep -q 'document.title = e.detail.name + " - " + PAGE_TITLE;' app/ui.js \
    && sed -i 's|document.title = e.detail.name + " - " + PAGE_TITLE;|document.title = e.detail.name;|' app/ui.js \
    && sed -i -e 's|<title>noVNC</title>|<title>FIC Studio</title>|' \
              -e 's|type="image/x-icon" href="app/images/icons/novnc.ico"|type="image/svg+xml" href="app/images/icons/fic-studio.svg"|' \
              -e '/novnc-ios-/d' \
              vnc.html \
    && grep -q 'fic-studio.svg' vnc.html \
    # Development files: nothing the page needs.
    && rm -rf docs tests snap utils po .github karma.conf.cjs eslint.config.mjs \
              vnc_lite.html README.md .gitignore .gitmodules \
    # The page reads its version from package.json; keep just that.
    && printf '{"version": "%s"}\n' "${NOVNC_VERSION}" > package.json

# ───────────────────────────────────────────────────────────────── user ───
# The Ubuntu base ships a user `ubuntu` with UID 1000; it is replaced by
# `fic`. The entrypoint can move `fic` to the host user's UID at start, so
# files written to a shared folder belong to that user on Linux hosts.
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && groupadd -g 1000 fic \
    && useradd -m -u 1000 -g fic -s /bin/bash fic \
    && echo 'fic ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/fic \
    && chmod 0440 /etc/sudoers.d/fic \
    && install -d -o fic -g fic /opt/fic

# ────────────────────────────────────────────────────────────── workspace ───
# The workspace is built in place, at the path it runs from, with
# --symlink-install: the tuning file the Studio saves (fic_library/config) is
# the one the controller reads through install/, which does not survive a
# copy or a move, so src/, build/ and install/ all stay in the image.
#
# Everything is group-writable (group `fic`): the Studio writes into the
# workspace, and the entrypoint may run `fic` under the host user's UID.
ENV FIC_ROOT=/opt/fic
WORKDIR /opt/fic
USER fic

# The C++ packages come first, in their own layer, so that GUI-only changes
# do not recompile them.
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX fic_library/ fic_library/
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX fic_ws/src/communication/ fic_ws/src/communication/
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX fic_ws/src/menagerie/     fic_ws/src/menagerie/
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX fic_ws/src/fic_simulation/ fic_ws/src/fic_simulation/
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX fic_ws/src/fic_hardware/  fic_ws/src/fic_hardware/

ARG COLCON_ARGS="--symlink-install --event-handlers console_cohesion+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
RUN umask 0002 \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd fic_ws \
    && colcon build ${COLCON_ARGS} \
         --packages-select communication menagerie fic_simulation fic_hardware \
    && rm -rf log

# Everything else in the workspace (the Studio, the native simulator, contact
# and locomotion). Building the whole tree also picks up packages added later.
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX fic_ws/src/ fic_ws/src/
RUN umask 0002 \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd fic_ws \
    # The C++ packages were built above from these same sources; CI checkouts
    # give files new timestamps, which would otherwise rebuild them all.
    && colcon build ${COLCON_ARGS} --packages-skip-build-finished \
    && rm -rf log

COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX Extensions/ Extensions/
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX fic_docs/   fic_docs/
# The scripting API reference the Studio's Help points to.
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX docs/scripting/ docs/scripting/
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX fic_api/    fic_api/
COPY --chown=fic:fic --chmod=u=rwX,g=rwX,o=rX README.md ./

USER root
# The standalone Python API, so scripts and notebooks can drive the robot.
RUN pip3 install -e /opt/fic/fic_api \
    && chown -R fic:fic /opt/fic/fic_api

# ─────────────────────────────────────────────── desktop configuration ───
# (Fixed modes: a checkout's own permissions must not reach /etc or /usr.)
COPY --chmod=u=rwX,go=rX docker/rootfs/ /
COPY fic_install/fic-control-studio.svg /usr/share/icons/hicolor/scalable/apps/fic-control-studio.svg
RUN chmod 0755 /usr/local/bin/fic /usr/local/bin/fic-* \
    && /usr/local/bin/fic-image-setup

# ────────────────────────────────────────────────────────────── runtime ───
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:1 \
    FIC_DISPLAY=vnc \
    FIC_SCREEN=1600x1000 \
    NOVNC_PORT=6080 \
    PYTHONUNBUFFERED=1 \
    FIC_WEB_REMOTE_HOSTS=localhost \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ROS_LOG_DIR=/tmp/ros_log \
    QT_X11_NO_MITSHM=1 \
    PYGLFW_LIBRARY_VARIANT=x11 \
    LD_LIBRARY_PATH=/usr/local/lib/fic/gl \
    XDG_RUNTIME_DIR=/tmp/runtime-fic \
    BROWSER=/usr/local/bin/fic-browser

EXPOSE 6080 8088

LABEL org.opencontainers.image.title="FIC Studio" \
      org.opencontainers.image.description="FIC Control Studio with MuJoCo simulation, shown in the browser. Simulation only." \
      org.opencontainers.image.source="https://github.com/TRL-Sussex/FIC_Interface" \
      org.opencontainers.image.vendor="TRL, University of Sussex"

# `docker exec` lands in the workspace, where the controllers expect to run.
WORKDIR /opt/fic/fic_ws

HEALTHCHECK --interval=30s --timeout=5s --start-period=60s --retries=3 \
    CMD ["/usr/local/bin/fic-healthcheck"]

ENTRYPOINT ["/usr/bin/tini", "-g", "--", "/usr/local/bin/fic-entrypoint"]
CMD ["studio"]
