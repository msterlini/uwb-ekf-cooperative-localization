#!/usr/bin/env bash
# Starts the simulation in Docker using the GPU of the host when there is one.
#
#   ./run.sh            windows on the desktop (Linux with X11 or XWayland, WSL2 with WSLg),
#                       on macOS it opens the desktop in the browser
#   ./run.sh web        desktop in the browser at http://localhost:6080/vnc.html (any OS)
#   ./run.sh headless   no windows, records bagfiles/$BAG_NAME.bag for $DURATION seconds
#
# The GPU is detected automatically, GPU=nvidia|dri|none forces a choice.
set -euo pipefail
cd "$(dirname "$0")"

# Docker Desktop on macOS has no X server nor GPU devices for the containers
if [ "$(uname -s)" = Darwin ]; then
    default_service=web
else
    default_service=sim
fi
service="${1:-$default_service}"

detect_gpu() {
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
        if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia || command -v nvidia-ctk >/dev/null 2>&1; then
            echo nvidia
            return
        fi
        echo "NVIDIA GPU found but nvidia-container-toolkit is not installed, it will not be used." >&2
    fi
    if [ -e /dev/dri/renderD128 ]; then
        echo dri
    else
        echo none
    fi
}

gpu="${GPU:-$(detect_gpu)}"
files=(-f docker-compose.yml)

case "$service" in
    sim)
        if [ -z "${DISPLAY:-}" ]; then
            echo "No DISPLAY found: use './run.sh web' to open the simulation in the browser." >&2
            exit 1
        fi
        command -v xhost >/dev/null 2>&1 && xhost +local:docker >/dev/null
        case "$gpu" in
            nvidia) files+=(-f docker/compose.nvidia.yml); echo "GPU: NVIDIA" ;;
            dri)    files+=(-f docker/compose.dri.yml);    echo "GPU: AMD/Intel (Mesa)" ;;
            none)   echo "GPU: none, software rendering (the simulation will be slower)" ;;
            *)      echo "Unknown GPU=$gpu, use nvidia, dri or none" >&2; exit 1 ;;
        esac
        exec docker compose "${files[@]}" up sim
        ;;
    web)
        echo "Browser desktop on a virtual display: software rendering"
        exec docker compose "${files[@]}" up web
        ;;
    headless)
        echo "Headless run on a virtual display: software rendering"
        exec docker compose "${files[@]}" run --rm headless
        ;;
    *)
        echo "Usage: $0 [sim|web|headless]" >&2
        exit 1
        ;;
esac
