#!/usr/bin/env bash
set -Eeuo pipefail

VENDOR_ROOT="${IGIBSON_VENDOR_ROOT:-/home/lxr20/lxr/igibson_vendor}"
REPO_DIR="${VENDOR_ROOT}/iGibson"
DOCKER_DIR="${IGIBSON_ROS_DOCKER_DIR:-${REPO_DIR}/docker/igibson-ros}"
DOCKERFILE="${DOCKER_DIR}/Dockerfile"

if [[ ! -d "$DOCKER_DIR" ]]; then
  echo "Missing iGibson ROS Docker directory: $DOCKER_DIR" >&2
  echo "Run integrations/igibson/scripts/02_prepare_igibson_vendor.sh first." >&2
  exit 1
fi

if [[ ! -x "$DOCKER_DIR/build.sh" ]]; then
  chmod +x "$DOCKER_DIR/build.sh"
fi

patch_miniconda_installer() {
  local old_line="RUN curl -LO http://repo.continuum.io/miniconda/Miniconda-latest-Linux-x86_64.sh"
  local new_line="RUN curl -fsSL -o Miniconda-latest-Linux-x86_64.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh"
  local old_rl_line="RUN pip install --no-cache-dir pytest ray[default,rllib] stable-baselines3 && rm -rf /root/.cache"
  local pinned_rl_line_without_build_tools="RUN pip install --no-cache-dir pytest 'ray[default,rllib]==1.13.0' stable-baselines3==1.5.0 && rm -rf /root/.cache"
  local pinned_rl_line_without_pip="RUN pip install --no-cache-dir setuptools==65.5.0 wheel==0.38.4 && pip install --no-cache-dir pytest 'ray[default,rllib]==1.13.0' stable-baselines3==1.5.0 && rm -rf /root/.cache"
  local new_rl_line="RUN pip install --no-cache-dir pip==23.3.2 setuptools==65.5.0 wheel==0.38.4 && pip install --no-cache-dir pytest 'ray[default,rllib]==1.13.0' stable-baselines3==1.5.0 && rm -rf /root/.cache"

  if [[ ! -f "$DOCKERFILE" ]]; then
    echo "Missing iGibson ROS Dockerfile: $DOCKERFILE" >&2
    exit 1
  fi

  if grep -Fq "$old_line" "$DOCKERFILE"; then
    perl -0pi -e 's#RUN curl -LO http://repo\.continuum\.io/miniconda/Miniconda-latest-Linux-x86_64\.sh#RUN curl -fsSL -o Miniconda-latest-Linux-x86_64.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh#' "$DOCKERFILE"
    echo "Patched stale Miniconda download URL in $DOCKERFILE"
  elif grep -Fq "$new_line" "$DOCKERFILE"; then
    echo "Miniconda download URL already patched in $DOCKERFILE"
  else
    echo "Miniconda download line has changed upstream; leaving $DOCKERFILE unchanged."
  fi

  if grep -Fq "conda tos accept --override-channels" "$DOCKERFILE"; then
    echo "Conda ToS accept step already present in $DOCKERFILE"
  elif grep -Fq "RUN conda update -y conda" "$DOCKERFILE"; then
    perl -0pi -e 's#RUN conda update -y conda#RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main \&\& \\\n    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r\nRUN conda update -y conda#' "$DOCKERFILE"
    echo "Patched non-interactive Conda ToS accept step in $DOCKERFILE"
  else
    echo "Conda update line has changed upstream; cannot insert ToS accept step automatically." >&2
    exit 1
  fi

  if grep -Fq "$old_rl_line" "$DOCKERFILE"; then
    OLD_RL_LINE="$old_rl_line" NEW_RL_LINE="$new_rl_line" perl -0pi -e 's#\Q$ENV{OLD_RL_LINE}\E#$ENV{NEW_RL_LINE}#g' "$DOCKERFILE"
    echo "Pinned iGibson ROS RL dependencies in $DOCKERFILE"
  elif grep -Fq "$pinned_rl_line_without_build_tools" "$DOCKERFILE"; then
    OLD_RL_LINE="$pinned_rl_line_without_build_tools" NEW_RL_LINE="$new_rl_line" perl -0pi -e 's#\Q$ENV{OLD_RL_LINE}\E#$ENV{NEW_RL_LINE}#g' "$DOCKERFILE"
    echo "Pinned iGibson ROS RL build tools in $DOCKERFILE"
  elif grep -Fq "$pinned_rl_line_without_pip" "$DOCKERFILE"; then
    OLD_RL_LINE="$pinned_rl_line_without_pip" NEW_RL_LINE="$new_rl_line" perl -0pi -e 's#\Q$ENV{OLD_RL_LINE}\E#$ENV{NEW_RL_LINE}#g' "$DOCKERFILE"
    echo "Pinned iGibson ROS RL pip version in $DOCKERFILE"
  elif grep -Fq "$new_rl_line" "$DOCKERFILE"; then
    echo "iGibson ROS RL dependencies already pinned in $DOCKERFILE"
  elif grep -Fq "stable-baselines3" "$DOCKERFILE" || grep -Fq "ray[default,rllib]" "$DOCKERFILE"; then
    echo "iGibson ROS RL dependency line has changed upstream; review $DOCKERFILE before building." >&2
    exit 1
  else
    echo "iGibson ROS RL dependency line not present in $DOCKERFILE; leaving unchanged."
  fi
}

patch_miniconda_installer

echo "Building iGibson ROS Docker image from $DOCKER_DIR"
cd "$DOCKER_DIR"
./build.sh
