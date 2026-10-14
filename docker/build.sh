#!/usr/bin/env bash
#
# Build the FIC Studio image locally. This is for maintainers: users run the
# image CI publishes (.github/workflows/docker-image.yml).
#
#   docker/build.sh                        build for this computer's architecture
#   docker/build.sh --test                 ...then run the self-test in it
#   docker/build.sh --platform linux/arm64 build for another architecture
#                                          (emulated, so much slower)
#   docker/build.sh --save                 ...then write docker/dist/<name>.tar.gz,
#                                          to hand out where downloads are slow:
#                                          users run  docker load -i <file>
#   docker/build.sh --tag REF              image name (default fic-studio:local)
#
# Try the result with:   docker/fic-studio.sh --image fic-studio:local

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"

TAG="fic-studio:local"
PLATFORM=""
TEST=0
SAVE=0

while [ $# -gt 0 ]; do
    case "$1" in
        --test)     TEST=1 ;;
        --save)     SAVE=1 ;;
        --platform) PLATFORM="${2:?--platform needs a value, e.g. linux/arm64}"; shift ;;
        --tag)      TAG="${2:?--tag needs an image name}"; shift ;;
        -h|--help)  sed -n '3,/^$/p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

command -v docker >/dev/null || { echo "Docker is not installed." >&2; exit 1; }
docker info >/dev/null 2>&1 || { echo "Docker is not running." >&2; exit 1; }

# buildx is not everywhere: Ubuntu's docker.io package ships without it. Its
# --load only means anything to buildx, so plain `docker build` goes without.
if docker buildx version >/dev/null 2>&1; then
    BUILD=(buildx build --load)
else
    BUILD=(build)
    [ -z "${PLATFORM}" ] || { echo "Building for another architecture needs buildx: apt install docker-buildx" >&2; exit 1; }
fi

args=(--file "${HERE}/Dockerfile" --tag "${TAG}")
if [ -n "${PLATFORM}" ]; then
    args+=(--platform "${PLATFORM}")
    if [ "${PLATFORM#linux/}" != "$(docker version -f '{{.Server.Arch}}')" ]; then
        # Emulating another architecture needs QEMU registered once.
        docker run --privileged --rm tonistiigi/binfmt --install all >/dev/null
    fi
fi

# Some networks — a university's, typically — leave Docker's own DNS unable to
# resolve inside containers: the daemon still pulls images, but the first
# `apt-get update` of the build fails with "Temporary failure resolving". The
# host itself is fine, so borrow its network for the build when that is so.
distro="$(awk -F= '/^ARG ROS_DISTRO=/{print $2; exit}' "${HERE}/Dockerfile")"
base="$(awk '/^FROM /{print $2; exit}' "${HERE}/Dockerfile" | sed "s/\${ROS_DISTRO}/${distro:-jazzy}/")"
if [ -z "${PLATFORM}" ] && ! docker run --rm "${base}" \
        getent hosts packages.ros.org >/dev/null 2>&1; then
    echo "── containers cannot resolve names here — building with the host's network"
    args+=(--network host)
fi

echo "── building ${TAG}${PLATFORM:+ for ${PLATFORM}}"
docker "${BUILD[@]}" "${args[@]}" \
    --label "org.opencontainers.image.revision=$(git -C "${ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)" \
    "${ROOT}"
docker image ls "${TAG}"

if [ "${TEST}" = 1 ]; then
    echo "── self-test"
    docker run --rm --shm-size 2g ${PLATFORM:+--platform "${PLATFORM}"} "${TAG}" fic-selftest
fi

if [ "${SAVE}" = 1 ]; then
    mkdir -p "${HERE}/dist"
    out="${HERE}/dist/$(echo "${TAG}" | tr '/:' '__')${PLATFORM:+-${PLATFORM#linux/}}.tar.gz"
    echo "── saving ${out}"
    docker save "${TAG}" | gzip -1 > "${out}"
    ls -lh "${out}"
    echo "Users load it with:  docker load -i $(basename "${out}")"
    echo "and start it with:   ./fic-studio.sh --image ${TAG}"
fi
