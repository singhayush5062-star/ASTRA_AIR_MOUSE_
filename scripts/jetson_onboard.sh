#!/bin/bash
# NIDAR onboard (Jetson Orin Nano) helper. Run on the Jetson from the repo root.
#
#   scripts/jetson_onboard.sh check         host checks: JetPack, Docker, devices, network
#   scripts/jetson_onboard.sh image         build the onboard image (docker/Dockerfile.jetson)
#   scripts/jetson_onboard.sh start         create/start the persistent container nidar_onboard
#   scripts/jetson_onboard.sh build         catkin build inside it (skips the simulation packages)
#   scripts/jetson_onboard.sh bringup       start the flight stack (hw_bringup.sh) -- never arms
#                                           (LIDAR=0 CAMERA=0 SKIP_PARITY=1 are passed through)
#   scripts/jetson_onboard.sh stop          stop the flight stack (refuses while armed)
#   scripts/jetson_onboard.sh shell         a shell in the container
#
# Image options (env): BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.4.1 TORCH_WHEEL=<url>  (JetPack 5
# GPU detector, see hardware/DEPLOYMENT.md §3). Everything else: catkin_ws/src/nidar_config/config/hardware.yaml.
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=${IMAGE:-nidar-onboard}
CONTAINER=${CONTAINER:-nidar_onboard}
IN_REPO=/home/developer/NIDAR
HW_CFG="$REPO_ROOT/catkin_ws/src/nidar_config/config/hardware.yaml"
SKIP_PKGS="velodyne_gazebo_plugins velodyne_simulator velodyne_description nidar_sim nidar_gcs"

hw() { python3 -c "import yaml; c=yaml.safe_load(open('$HW_CFG'))['hardware']; print($1)"; }
in_container() {  # -it only with a terminal (systemd has none); forwards hw_bringup.sh's switches
    local args=()
    [ -t 0 ] && args+=(-it)
    for v in LIDAR CAMERA SKIP_PARITY; do [ -n "${!v:-}" ] && args+=(-e "$v=${!v}"); done
    docker exec "${args[@]}" "$CONTAINER" bash -lc "$*"
}

case "${1:-}" in
check)
    echo "== JetPack / L4T";  cat /etc/nv_tegra_release 2>/dev/null || echo "not a Jetson?"
    echo "== power mode";     (nvpmodel -q 2>/dev/null || echo "nvpmodel not found") | tail -2
    echo "== docker";         docker --version && (docker info 2>/dev/null | grep -i runtime || true)
    echo "== groups";         id -nG | tr ' ' '\n' | grep -E "^(docker|dialout|video)$" || true
    echo "== FC serial";      ls -l /dev/ttyTHS* /dev/ttyACM* 2>/dev/null || echo "none"
    echo "== USB serial";     ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null || echo "none"
    echo "== cameras";        ls /dev/video* 2>/dev/null || echo "none"
    LIDAR_IP=$(hw "c['lidar']['lidar_ip']"); HOST_IP=$(hw "c['lidar']['host_ip']")
    JETSON_IP=$(hw "c['network']['jetson_ip']")
    echo "== network (hardware.yaml: jetson $JETSON_IP, LiDAR host $HOST_IP -> Mid-360 $LIDAR_IP)"
    ip -4 -brief addr
    ip -4 addr | grep -q " $HOST_IP/" || echo "  !! $HOST_IP is not configured. Static IP on the LiDAR port, e.g.:
     sudo nmcli con add type ethernet ifname eth0 con-name livox ipv4.method manual ipv4.addresses $HOST_IP/24
     sudo nmcli con up livox"
    ping -c 1 -W 1 "$LIDAR_IP" > /dev/null 2>&1 && echo "  Mid-360 $LIDAR_IP answers" || echo "  !! Mid-360 $LIDAR_IP does not answer ping"
    ip -4 addr | grep -q " $JETSON_IP/" || echo "  !! $JETSON_IP (hardware.yaml network.jetson_ip) is not on any interface: the GCS ROS link will fail"
    ;;
image)
    docker build -t "$IMAGE" -f "$REPO_ROOT/docker/Dockerfile.jetson" \
        --build-arg BASE_IMAGE="${BASE_IMAGE:-ros:noetic-ros-base-focal}" \
        --build-arg TORCH_WHEEL="${TORCH_WHEEL:-}" \
        --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
        "$REPO_ROOT/docker"
    ;;
start)
    if docker container inspect "$CONTAINER" > /dev/null 2>&1; then
        docker start "$CONTAINER" > /dev/null
    else
        RUNTIME=()
        docker info 2>/dev/null | grep -qi "nvidia" && RUNTIME=(--runtime nvidia)
        # host network: LiDAR UDP, MAVLink UDP to the GCS, ROS to the GCS. privileged + /dev:
        # the FC UART, USB serial adapters and cameras, including ones plugged in later.
        docker run -d --name "$CONTAINER" --restart unless-stopped \
            --net=host --ipc=host --privileged "${RUNTIME[@]}" \
            -v /dev:/dev -v "$REPO_ROOT:$IN_REPO" -w "$IN_REPO" \
            "$IMAGE" sleep infinity > /dev/null
    fi
    echo "container $CONTAINER running (scripts/jetson_onboard.sh shell)"
    ;;
build)
    in_container "cd $IN_REPO/catkin_ws && source /opt/ros/noetic/setup.bash && \
        catkin config --profile jetson --extend /opt/livox_ws/devel --skiplist $SKIP_PKGS \
            --cmake-args -DCMAKE_BUILD_TYPE=Release > /dev/null && \
        catkin build --profile jetson -j\$(nproc) && \
        python3 $IN_REPO/catkin_ws/src/nidar_config/scripts/apply_hardware_config.py --check"
    ;;
bringup)
    in_container "$IN_REPO/catkin_ws/src/nidar_bringup/scripts/hw_bringup.sh"
    ;;
stop)
    in_container "$IN_REPO/catkin_ws/src/nidar_bringup/scripts/hw_stop.sh ${2:-}"
    ;;
shell)
    in_container "cd $IN_REPO && exec bash"
    ;;
*)
    sed -n '2,14p' "$0"; exit 1 ;;
esac
