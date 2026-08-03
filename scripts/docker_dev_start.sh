#!/bin/bash
# Automated NIDAR Development Environment Starter for Teammates & CI
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="ros-noetic-workspace"
CONTAINER_NAME="nidar_dev_container"

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
echo "[3/3] Launching interactive NIDAR development container..."
echo "============================================================"
# Remove existing container if running/stopped with the same name
if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    echo "Cleaning up previous stopped container titled $CONTAINER_NAME..."
    docker rm -f "$CONTAINER_NAME" || true
fi

# Execute container with host networking, GUI X11 forwarding, and workspace volume binding
docker run -it --rm \
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
