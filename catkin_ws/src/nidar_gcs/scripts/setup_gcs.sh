#!/bin/bash
# One-time setup for the NIDAR GCS (backend + web UI). Safe to re-run.
#
#   catkin_ws/src/nidar_gcs/scripts/setup_gcs.sh            # deps + frontend build
#   catkin_ws/src/nidar_gcs/scripts/setup_gcs.sh --no-build # deps only
#
# Everything is installed per-user under $NIDAR_GCS_HOME (default ~/.local/share/nidar_gcs):
#   pydeps/  backend Python packages (pip --target; never touches the system/ROS site-packages)
#   node/    a Node.js LTS runtime used only to build the frontend
# The built UI lands in nidar_gcs/frontend/dist and is served by the backend on :8000.
set -eo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GCS_HOME="${NIDAR_GCS_HOME:-$HOME/.local/share/nidar_gcs}"
NODE_MAJOR="${NIDAR_GCS_NODE_MAJOR:-22}"
BUILD=1
[ "${1:-}" = "--no-build" ] && BUILD=0

mkdir -p "$GCS_HOME"

echo "[setup_gcs] Python deps -> $GCS_HOME/pydeps"
python3 -m pip install --disable-pip-version-check --upgrade --target "$GCS_HOME/pydeps" \
    -r "$PKG_DIR/backend/requirements-runtime.txt"
# Optional: ONNX runtime for the Hardware page's laptop-side YOLO on the FC/Jetson stream.
# The simulation path does not need it (detection runs onboard in nidar_perception).
python3 -m pip install --disable-pip-version-check --upgrade --target "$GCS_HOME/pydeps" onnxruntime \
    || echo "[setup_gcs] onnxruntime not installed (Hardware camera inference disabled; simulation unaffected)"
# onnxruntime drags numpy into the target dir; it would shadow the system numpy (the one the
# system cv2 / cv_bridge are built against) inside the backend process. Use the system copy.
rm -rf "$GCS_HOME/pydeps"/numpy "$GCS_HOME/pydeps"/numpy-*.dist-info "$GCS_HOME/pydeps"/numpy.libs

if [ -x "$GCS_HOME/node/bin/node" ] && "$GCS_HOME/node/bin/node" -v | grep -q "^v${NODE_MAJOR}\."; then
    echo "[setup_gcs] Node $("$GCS_HOME/node/bin/node" -v) already installed"
else
    case "$(uname -m)" in
        x86_64)  NODE_ARCH=x64 ;;
        aarch64) NODE_ARCH=arm64 ;;
        *) echo "[setup_gcs] unsupported arch $(uname -m)"; exit 1 ;;
    esac
    NODE_VER=$(curl -fsSL https://nodejs.org/dist/index.json | python3 -c "
import json, sys
rel = [r for r in json.load(sys.stdin) if r['version'].startswith('v${NODE_MAJOR}.') and r['lts']]
print(rel[0]['version'])")
    TARBALL="node-${NODE_VER}-linux-${NODE_ARCH}.tar.xz"
    echo "[setup_gcs] downloading Node ${NODE_VER} (${NODE_ARCH})"
    TMP=$(mktemp -d)
    curl -fsSL "https://nodejs.org/dist/${NODE_VER}/${TARBALL}" -o "$TMP/$TARBALL"
    curl -fsSL "https://nodejs.org/dist/${NODE_VER}/SHASUMS256.txt" -o "$TMP/SHASUMS256.txt"
    (cd "$TMP" && grep " ${TARBALL}\$" SHASUMS256.txt | sha256sum -c -)
    rm -rf "$GCS_HOME/node" && mkdir -p "$GCS_HOME/node"
    tar -xJf "$TMP/$TARBALL" -C "$GCS_HOME/node" --strip-components=1
    rm -rf "$TMP"
fi
export PATH="$GCS_HOME/node/bin:$PATH"

if [ "$BUILD" = "1" ]; then
    echo "[setup_gcs] building frontend"
    cd "$PKG_DIR/frontend"
    npm ci --no-audit --no-fund
    npm run build
    echo "[setup_gcs] UI built -> $PKG_DIR/frontend/dist"
fi
echo "[setup_gcs] done. Start the GCS with: catkin_ws/src/nidar_gcs/scripts/start_gcs.sh"
