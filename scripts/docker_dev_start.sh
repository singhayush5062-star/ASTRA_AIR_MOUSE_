#!/bin/bash
# Automated NIDAR Development Environment Starter for Teammates & CI
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="ros-noetic-workspace"
CONTAINER_NAME="ros_workspace"

echo "============================================================"
echo "[1/3] Configuring host X11 display authorization..."
echo "============================================================"
if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker || true
else
    echo "Warning: xhost not found on host. Ensure X11 sockets allow local connections."
fi

echo "============================================================"
echo "[2/3] Checking for Docker development image ($IMAGE_NAME)..."
echo "============================================================"
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Image '$IMAGE_NAME' not found locally. Building from $REPO_ROOT/docker/Dockerfile..."
    docker build -t "$IMAGE_NAME" -f "$REPO_ROOT/docker/Dockerfile" "$REPO_ROOT/docker"
else
    echo "Image '$IMAGE_NAME' found and ready."
fi

echo "============================================================"
echo "[3/3] Managing persistent NIDAR development container ($CONTAINER_NAME)..."
echo "============================================================"
if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    IS_RUNNING=$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || echo "false")
    if [ "$IS_RUNNING" = "true" ]; then
        echo "Container '$CONTAINER_NAME' is already running. Attaching interactive shell..."
        docker exec -it "$CONTAINER_NAME" /bin/bash
    else
        echo "Starting existing stopped container '$CONTAINER_NAME'..."
        docker start "$CONTAINER_NAME"
        docker exec -it "$CONTAINER_NAME" /bin/bash
    fi
else
    echo "Creating new persistent container '$CONTAINER_NAME'..."
    docker run -it \
        --net=host \
        --name "$CONTAINER_NAME" \
        --ipc=host \
        --privileged \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e LIBGL_ALWAYS_SOFTWARE=0 \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$REPO_ROOT:/home/developer/NIDAR" \
        -w /home/developer/NIDAR \
        "$IMAGE_NAME" /bin/bash
fi
