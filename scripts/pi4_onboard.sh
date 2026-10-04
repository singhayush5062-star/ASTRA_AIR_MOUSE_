#!/bin/bash
# NIDAR onboard helper for the Raspberry Pi 4 (64-bit OS). Full guide: hardware/PI4_DEPLOYMENT.md
#
# On the DEV PC (x86, builds the arm64 image under QEMU; the Pi never compiles):
#   scripts/pi4_onboard.sh image              build nidar-onboard-pi4 (2-4 h the first time)
#   scripts/pi4_onboard.sh deploy <user@pi>   stream the image to the Pi over ssh (docker load)
#   scripts/pi4_onboard.sh export [file]      or save it to a .tar.gz (USB stick / scp)
#   scripts/pi4_onboard.sh push               or publish it to the team registry (ghcr.io, private)
#
# On the PI 4 (from the repo root, branch pi4_deployment):
#   scripts/pi4_onboard.sh setup              one-time host setup: UART, groups, eth0 addresses (sudo)
#   scripts/pi4_onboard.sh check              host checks: OS, UART, network, LiDAR/camera/GCS, heat
#   scripts/pi4_onboard.sh load <file>        docker load an exported image
#   scripts/pi4_onboard.sh pull [tag]         or pull it from the team registry (default: latest)
#   scripts/pi4_onboard.sh start              create/start the container nidar_onboard
#   scripts/pi4_onboard.sh bringup            start the flight stack (hw_bringup.sh) -- never arms
#                                             (LIDAR=0 CAMERA=0 SKIP_PARITY=1 are passed through)
#   scripts/pi4_onboard.sh stop               stop the flight stack (refuses while armed)
#   scripts/pi4_onboard.sh status             container, image vs repo version, CPU load/temp
#   scripts/pi4_onboard.sh shell              a shell in the container (ROS environment loaded)
#   scripts/pi4_onboard.sh autostart          install the systemd service (bringup at boot)
#   scripts/pi4_onboard.sh rm                 remove the container (image and repo stay)
#
# Settings: catkin_ws/src/nidar_config/config/hardware.yaml.
# Env: IMAGE, CONTAINER, JOBS (image), REGISTRY (push/pull, default ghcr.io/singhayush5062-star).
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=${IMAGE:-nidar-onboard-pi4}
BASE_IMAGE=${BASE_IMAGE:-nidar-onboard-pi4-base}
CONTAINER=${CONTAINER:-nidar_onboard}
REGISTRY=${REGISTRY:-ghcr.io/singhayush5062-star}
IN_REPO=/home/developer/NIDAR
HW_CFG="$REPO_ROOT/catkin_ws/src/nidar_config/config/hardware.yaml"

say()  { echo "[pi4_onboard] $*"; }
warn() { echo "  !! $*"; }
die()  { echo "[pi4_onboard] ERROR: $*" >&2; exit 1; }
hw()   {
    python3 -c "import yaml" 2> /dev/null || die "python3-yaml missing: sudo apt install -y python3-yaml"
    python3 -c "import yaml; c=yaml.safe_load(open('$HW_CFG'))['hardware']; print($1)"
}
on_pi() { [ "$(uname -m)" = "aarch64" ]; }
in_container() {  # -it only with a terminal (systemd has none); forwards hw_bringup.sh's switches
    local args=()
    [ -t 0 ] && args+=(-it)
    for v in LIDAR CAMERA SKIP_PARITY; do [ -n "${!v:-}" ] && args+=(-e "$v=${!v}"); done
    docker exec "${args[@]}" "$CONTAINER" bash -lc "$*"
}
repo_sha() { git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown; }
image_sha() { docker run --rm --entrypoint cat "$IMAGE" /opt/nidar_ws/GIT_SHA 2>/dev/null || echo none; }

case "${1:-}" in
# ── dev PC ──────────────────────────────────────────────────────────────────────────────────
image)
    if ! on_pi && [ ! -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
        say "registering QEMU arm64 emulation (until the next reboot of this PC)"
        docker run --privileged --rm tonistiigi/binfmt --install arm64 > /dev/null
    fi
    # --network=host: apt inside Docker's bridge network has produced "Hash Sum mismatch" on
    # some networks (packages.ros.org); the host network avoids it.
    say "1/2 base image $BASE_IMAGE (ROS, MAVROS, Livox, detector runtime)"
    DOCKER_BUILDKIT=1 docker build --platform linux/arm64 --network=host -t "$BASE_IMAGE" \
        -f "$REPO_ROOT/docker/Dockerfile.jetson" \
        --build-arg USER_UID="${PI_UID:-1000}" --build-arg USER_GID="${PI_GID:-1000}" \
        "$REPO_ROOT/docker"
    say "2/2 $IMAGE (NIDAR workspace compiled, commit $(repo_sha))"
    DOCKER_BUILDKIT=1 docker build --platform linux/arm64 --network=host -t "$IMAGE" \
        -f "$REPO_ROOT/docker/Dockerfile.pi4" \
        --build-arg BASE_IMAGE="$BASE_IMAGE" --build-arg JOBS="${JOBS:-3}" \
        --build-arg GIT_SHA="$(repo_sha)" \
        "$REPO_ROOT"
    say "done: $(docker image inspect "$IMAGE" --format '{{.Architecture}} {{.Size}}') bytes. Next: $0 deploy <user@pi>"
    ;;
deploy)
    [ -n "${2:-}" ] || die "usage: $0 deploy <user@pi-address>"
    say "streaming $IMAGE to $2 (compressed, ~2-3 GB; several minutes over Wi-Fi)"
    docker save "$IMAGE" | gzip -1 | ssh "$2" 'gunzip | docker load'
    say "loaded on $2. There: git pull, then scripts/pi4_onboard.sh rm && scripts/pi4_onboard.sh start"
    ;;
push)
    # Private package linked to the GitHub repo (the image's org.opencontainers.image.source
    # label); needs `docker login ghcr.io` with a token that has write:packages.
    SHA=$(image_sha); [ "$SHA" != none ] || die "image $IMAGE not built ($0 image)"
    [ "$SHA" = "$(repo_sha)" ] || say "note: image built from $SHA, repo at $(repo_sha)"
    for tag in "$SHA" latest; do
        docker tag "$IMAGE" "$REGISTRY/$IMAGE:$tag"
        docker push "$REGISTRY/$IMAGE:$tag" || die "push failed: docker login ghcr.io (token with write:packages)?"
    done
    say "pushed $REGISTRY/$IMAGE:{$SHA,latest}. On the Pi: $0 pull"
    ;;
export)
    OUT=${2:-$PWD/${IMAGE}_$(image_sha).tar.gz}
    say "saving $IMAGE to $OUT"
    docker save "$IMAGE" | gzip -1 > "$OUT"
    say "$(du -h "$OUT" | cut -f1)  -> copy to the Pi, then: scripts/pi4_onboard.sh load $(basename "$OUT")"
    ;;
# ── Raspberry Pi 4 ──────────────────────────────────────────────────────────────────────────
setup)
    on_pi || die "setup is for the Raspberry Pi (this is $(uname -m))"
    BOOT=/boot/firmware; [ -f $BOOT/config.txt ] || BOOT=/boot
    [ -f $BOOT/config.txt ] || die "no config.txt in /boot/firmware or /boot"
    HOST_IP=$(hw "c['lidar']['host_ip']"); ONBOARD_IP=$(hw "c['network']['jetson_ip']")
    IFACE=${IFACE:-eth0}
    cat <<EOF
[pi4_onboard] one-time setup of this Pi (sudo; backups as *.nidar.bak):
  1. $BOOT/config.txt   enable_uart=1, dtoverlay=disable-bt   (FC on the PL011 UART /dev/ttyAMA0)
  2. $BOOT/cmdline.txt  remove the serial console (console=serial0/ttyAMA0)
  3. disable hciuart and serial-getty@ttyAMA0 (they hold the UART)
  4. add $USER to the docker and dialout groups
  5. $IFACE static addresses $HOST_IP/24 (Mid-360) + $ONBOARD_IP/24 (A8 mini, T12, GCS), no gateway
     !! if you are logged in over $IFACE, you will lose that session: use Wi-Fi or a keyboard.
  Then reboot.
EOF
    if [ "${2:-}" != "-y" ]; then read -r -p "apply? [y/N] " a; [ "$a" = y ] || exit 1; fi
    sudo cp -n $BOOT/config.txt $BOOT/config.txt.nidar.bak
    for line in enable_uart=1 dtoverlay=disable-bt; do
        grep -qx "$line" $BOOT/config.txt || echo "$line" | sudo tee -a $BOOT/config.txt > /dev/null
    done
    sudo cp -n $BOOT/cmdline.txt $BOOT/cmdline.txt.nidar.bak
    sudo sed -i -E 's/ ?console=(serial0|ttyAMA0|ttyS0),[0-9]+//g' $BOOT/cmdline.txt
    sudo systemctl disable --now hciuart.service 2>/dev/null || true
    sudo systemctl mask serial-getty@ttyAMA0.service serial-getty@serial0.service 2>/dev/null || true
    sudo usermod -aG docker,dialout "$USER"
    if command -v nmcli > /dev/null 2>&1; then          # Raspberry Pi OS (NetworkManager)
        sudo nmcli con delete nidar-$IFACE > /dev/null 2>&1 || true
        sudo nmcli con add type ethernet ifname "$IFACE" con-name nidar-$IFACE \
            ipv4.method manual ipv4.addresses "$HOST_IP/24,$ONBOARD_IP/24" \
            ipv4.never-default yes ipv6.method disabled connection.autoconnect-priority 100 > /dev/null
        sudo nmcli con up nidar-$IFACE > /dev/null || warn "nidar-$IFACE not up yet (cable?)"
    else                                                # Ubuntu Server (netplan)
        sudo tee /etc/netplan/60-nidar.yaml > /dev/null <<EOF
# NIDAR drone network (scripts/pi4_onboard.sh setup): Mid-360 + A8 mini / T12 / GCS on $IFACE
network:
  version: 2
  ethernets:
    $IFACE:
      dhcp4: false
      addresses: [$HOST_IP/24, $ONBOARD_IP/24]
EOF
        sudo chmod 600 /etc/netplan/60-nidar.yaml
        sudo netplan apply
    fi
    say "done. Reboot now (sudo reboot), then: scripts/pi4_onboard.sh check"
    ;;
check)
    set +e   # report every problem, do not stop at the first
    on_pi || warn "this is $(uname -m), not the Pi: host checks are meaningful on the Pi only"
    echo "== system";   . /etc/os-release 2>/dev/null; echo "  $PRETTY_NAME, $(uname -m), $(nproc) cores, $(free -g | awk '/Mem:/{print $2}') GB RAM"
    [ "$(uname -m)" = "aarch64" ] || warn "a 64-bit OS (aarch64) is required"
    echo "== docker";   docker --version 2>/dev/null || warn "docker not installed (curl -fsSL https://get.docker.com | sh)"
    id -nG | tr ' ' '\n' | grep -qx docker  || warn "$USER not in group docker (setup, then log out/in)"
    id -nG | tr ' ' '\n' | grep -qx dialout || warn "$USER not in group dialout"
    echo "== image";    docker image inspect "$IMAGE" > /dev/null 2>&1 \
        && echo "  $IMAGE built from $(image_sha), repo at $(repo_sha)" || warn "image $IMAGE not loaded (deploy/load)"
    FCU_URL=$(hw "c['fcu']['url']"); FCU_DEV=${FCU_URL%%:*}
    echo "== flight controller ($FCU_URL)"
    [ -e "$FCU_DEV" ] && ls -l "$FCU_DEV" || warn "$FCU_DEV missing (setup + reboot; FC TELEM2 -> pins 8/10/6)"
    grep -q -E "console=(serial0|ttyAMA0)" /proc/cmdline && warn "serial console still on the UART (setup, reboot)"
    for f in /boot/firmware/config.txt /boot/config.txt; do
        [ -f $f ] && { grep -qx "dtoverlay=disable-bt" $f || warn "dtoverlay=disable-bt missing in $f"; break; }
    done
    echo "== network";  ip -4 -brief addr | grep -v "^lo"
    HOST_IP=$(hw "c['lidar']['host_ip']"); LIDAR_IP=$(hw "c['lidar']['lidar_ip']")
    ONBOARD_IP=$(hw "c['network']['jetson_ip']"); GCS_IP=$(hw "c['network']['gcs_ip']")
    CAM=$(hw "c['camera']['source']"); CAM_IP=$(echo "$CAM" | sed -n -E 's#^rtsp://([^/:]+).*#\1#p')
    for ip in $HOST_IP $ONBOARD_IP; do ip -4 addr | grep -q " $ip/" || warn "$ip (hardware.yaml) not on any interface (setup)"; done
    # The LiDAR and drone subnets must exist on ONE interface only (e.g. home Wi-Fi on 192.168.1.x).
    for net in "${HOST_IP%.*}." "${ONBOARD_IP%.*}."; do
        n=$(ip -4 -o addr | awk '{print $2, $4}' | grep -F " $net" | awk '{print $1}' | sort -u | wc -l)
        [ "$n" -gt 1 ] && warn "subnet ${net}x is on $n interfaces: move the Wi-Fi to another network (PI4_DEPLOYMENT.md §4)"
    done
    for t in "Mid-360:$LIDAR_IP" "A8 mini:$CAM_IP" "GCS laptop:$GCS_IP"; do
        name=${t%%:*}; ip=${t#*:}; [ -n "$ip" ] || continue
        ping -c 1 -W 1 "$ip" > /dev/null 2>&1 && echo "  $name $ip answers" || warn "$name $ip does not answer ping"
    done
    echo "== power / heat"
    if command -v vcgencmd > /dev/null 2>&1; then
        echo "  $(vcgencmd measure_temp)  $(vcgencmd get_throttled)  (must be throttled=0x0)"
    elif [ -r /sys/class/thermal/thermal_zone0/temp ]; then
        echo "  temp=$(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 ))'C"
    fi
    ;;
pull)
    docker pull --platform linux/arm64 "$REGISTRY/$IMAGE:${2:-latest}" \
        || die "pull failed: docker login ghcr.io -u <github-user> (token with read:packages), and access to the package"
    docker tag "$REGISTRY/$IMAGE:${2:-latest}" "$IMAGE"
    say "image $IMAGE = $REGISTRY/$IMAGE:${2:-latest} (built from $(image_sha)). Next: $0 rm; $0 start"
    ;;
load)
    [ -f "${2:-}" ] || die "usage: $0 load <nidar-onboard-pi4_*.tar.gz>"
    gunzip -c "$2" | docker load
    ;;
start)
    docker image inspect "$IMAGE" > /dev/null 2>&1 || die "image $IMAGE not loaded (deploy from the PC, or load <file>)"
    if docker container inspect "$CONTAINER" > /dev/null 2>&1; then
        docker start "$CONTAINER" > /dev/null
    else
        # host network: LiDAR UDP, A8 mini RTSP, MAVLink UDP + ROS to the GCS. privileged + /dev:
        # the FC UART and USB serial adapters, including ones plugged in later.
        docker run -d --name "$CONTAINER" --restart unless-stopped \
            --net=host --ipc=host --privileged \
            -v /dev:/dev -v "$REPO_ROOT:$IN_REPO" -w "$IN_REPO" \
            "$IMAGE" > /dev/null
    fi
    [ "$(image_sha)" = "$(repo_sha)" ] || say "note: image built from $(image_sha), repo at $(repo_sha) -- C++ changes need a new image"
    say "container $CONTAINER running. Next: $0 bringup"
    ;;
bringup)
    in_container "$IN_REPO/catkin_ws/src/nidar_bringup/scripts/hw_bringup.sh"
    ;;
stop)
    in_container "$IN_REPO/catkin_ws/src/nidar_bringup/scripts/hw_stop.sh ${2:-}"
    ;;
status)
    set +e
    docker ps -a --filter "name=^${CONTAINER}$" --format '{{.Names}}  {{.Status}}  ({{.Image}})'
    echo "image built from $(image_sha), repo at $(repo_sha)"
    docker stats --no-stream --format '{{.Name}}  CPU {{.CPUPerc}}  MEM {{.MemUsage}}' "$CONTAINER" 2>/dev/null || true
    command -v vcgencmd > /dev/null 2>&1 && echo "$(vcgencmd measure_temp)  $(vcgencmd get_throttled)"
    LAST=$(ls -1d "$REPO_ROOT"/logs/hw/*/ 2>/dev/null | tail -1); [ -n "$LAST" ] && echo "last run logs: $LAST"
    ;;
shell)
    in_container "cd $IN_REPO && exec bash"
    ;;
autostart)
    sed "s/@USER@/$USER/; s#@REPO@#$REPO_ROOT#; s#scripts/jetson_onboard.sh#scripts/pi4_onboard.sh#g; s/the Jetson boots/the Pi boots/" \
        "$REPO_ROOT/catkin_ws/src/nidar_hardware/systemd/nidar-onboard.service" \
        | sudo tee /etc/systemd/system/nidar-onboard.service > /dev/null
    sudo systemctl daemon-reload && sudo systemctl enable nidar-onboard
    say "enabled: the stack starts at boot (never arms). Follow: journalctl -u nidar-onboard -f"
    ;;
rm)
    docker rm -f "$CONTAINER" > /dev/null && say "removed $CONTAINER"
    ;;
*)
    sed -n '2,26p' "$0"; exit 1 ;;
esac
