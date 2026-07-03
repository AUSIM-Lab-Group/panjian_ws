#!/usr/bin/env bash
set -Eeuo pipefail

VENDOR_ROOT="${IGIBSON_VENDOR_ROOT:-/home/lxr20/lxr/igibson_vendor}"
REPO_DIR="${VENDOR_ROOT}/iGibson"
DOCKER_DIR="${IGIBSON_ROS_DOCKER_DIR:-${REPO_DIR}/docker/igibson-ros}"
CONTAINER_NAME="${IGIBSON_CONTAINER_NAME:-igibson_ros}"
IGIBSON_ASSETS_DIR="${IGIBSON_ASSETS_DIR:-/home/lxr20/lxr/igibson_assets}"
PANJIAN_WS="${PANJIAN_WS:-/home/lxr20/lxr/panjian_ws}"
IGIBSON_INTEGRATION_DIR="${IGIBSON_INTEGRATION_DIR:-${PANJIAN_WS}/integrations/igibson}"
IGIBSON_CONTAINER_INTEGRATION_DIR="${IGIBSON_CONTAINER_INTEGRATION_DIR:-/workspace/integrations/igibson}"

mkdir -p "$IGIBSON_ASSETS_DIR"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set. Start this from an X11 desktop session." >&2
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null
fi

if [[ "${IGIBSON_USE_OFFICIAL_RUN_GUI:-0}" == "1" && -x "$DOCKER_DIR/run_gui.sh" ]]; then
  echo "Starting iGibson ROS container with official run_gui.sh"
  cd "$DOCKER_DIR"
  exec ./run_gui.sh
fi

if [[ ! -d "$IGIBSON_INTEGRATION_DIR" ]]; then
  echo "Missing integration directory to mount: $IGIBSON_INTEGRATION_DIR" >&2
  exit 1
fi

IGIBSON_IMAGE="${IGIBSON_IMAGE:-igibson/igibson-ros:latest}"
echo "Starting iGibson ROS container for image $IGIBSON_IMAGE"
echo "Mounting integration files: $IGIBSON_INTEGRATION_DIR -> $IGIBSON_CONTAINER_INTEGRATION_DIR"
exec docker run --rm -it \
  --name "$CONTAINER_NAME" \
  --gpus all \
  --network host \
  --ipc host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$IGIBSON_ASSETS_DIR":/root/.igibson:rw \
  -v "$IGIBSON_INTEGRATION_DIR":"$IGIBSON_CONTAINER_INTEGRATION_DIR":rw \
  "$IGIBSON_IMAGE" \
  bash
