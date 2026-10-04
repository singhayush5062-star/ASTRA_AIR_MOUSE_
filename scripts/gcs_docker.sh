#!/bin/bash
# NIDAR GCS in its own Docker container (operator laptop). Run from anywhere in the repo.
# Does not touch the dev container (ros_workspace) or its catkin build: separate image
# (nidar-gcs), separate container (nidar_gcs), the UI and nidar_msgs are built inside the image.
#
#   scripts/gcs_docker.sh image     build the image (docker/Dockerfile.gcs), ~5-10 min the first time
#   scripts/gcs_docker.sh start     create/start the container -> http://localhost:8000 -> HARDWARE
#   scripts/gcs_docker.sh logs      follow the GCS log
#   scripts/gcs_docker.sh restart   restart (after backend code / backend/.env changes)
#   scripts/gcs_docker.sh stop      stop the container
#   scripts/gcs_docker.sh shell     a shell in the running container (ROS environment loaded)
#   scripts/gcs_docker.sh rm        remove the container (the image and the repo stay)
#
# Env: GCS_PORT=8001 if 8000 is taken (e.g. a GCS already running in ros_workspace).
# After frontend changes: image, then rm + start. See hardware/DEPLOYMENT.md §8.
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=${IMAGE:-nidar-gcs}
CONTAINER=${CONTAINER:-nidar_gcs}
PORT=${GCS_PORT:-8000}
IN_REPO=/home/developer/NIDAR

port_busy() { (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null; }

case "${1:-}" in
image)
    DOCKER_BUILDKIT=1 docker build -t "$IMAGE" -f "$REPO_ROOT/docker/Dockerfile.gcs" \
        --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
        "$REPO_ROOT/catkin_ws/src"
    ;;
start)
    docker image inspect "$IMAGE" > /dev/null 2>&1 || "$0" image
    if docker container inspect "$CONTAINER" > /dev/null 2>&1; then
        docker start "$CONTAINER" > /dev/null
    else
        if port_busy "$PORT"; then
            echo "[gcs_docker] port $PORT is in use (a GCS in ros_workspace?). Stop it, or: GCS_PORT=8001 $0 start"
            exit 1
        fi
        # host network: MAVLink UDP 14550 from the Jetson / SITL, and ROS to the Jetson's master.
        # /dev + dialout: USB telemetry radios (T12 ground unit, SiK) and the FC's USB port,
        # including ones plugged in after the container started.
        docker run -d --name "$CONTAINER" --restart unless-stopped \
            --net=host --privileged -v /dev:/dev --group-add dialout --group-add video \
            -e PORT="$PORT" -v "$REPO_ROOT:$IN_REPO" \
            "$IMAGE" > /dev/null
    fi
    echo "[gcs_docker] container $CONTAINER started -> http://localhost:$PORT  (logs: $0 logs)"
    for _ in $(seq 1 20); do port_busy "$PORT" && { echo "[gcs_docker] GCS is up"; exit 0; }; sleep 1; done
    echo "[gcs_docker] GCS not answering yet -- check: $0 logs"
    ;;
logs)    docker logs -f "$CONTAINER" ;;
restart) docker restart "$CONTAINER" > /dev/null && echo "[gcs_docker] restarted" ;;
stop)    docker stop "$CONTAINER" > /dev/null && echo "[gcs_docker] stopped" ;;
shell)   docker exec -it "$CONTAINER" bash ;;
rm)      docker rm -f "$CONTAINER" > /dev/null && echo "[gcs_docker] removed $CONTAINER" ;;
*)       sed -n '2,15p' "$0"; exit 1 ;;
esac
