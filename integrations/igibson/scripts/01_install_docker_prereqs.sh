#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "20.04" ]]; then
  echo "This script is written for Ubuntu 20.04; detected ${PRETTY_NAME}." >&2
  exit 1
fi

echo "Installing Docker Engine and NVIDIA Container Toolkit for Ubuntu 20.04"

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y ca-certificates curl gnupg lsb-release

"${SUDO[@]}" install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | "${SUDO[@]}" gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
"${SUDO[@]}" chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${VERSION_CODENAME} stable" |
  "${SUDO[@]}" tee /etc/apt/sources.list.d/docker.list >/dev/null

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |
  "${SUDO[@]}" gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list |
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' |
  "${SUDO[@]}" tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y nvidia-container-toolkit
"${SUDO[@]}" nvidia-ctk runtime configure --runtime=docker
"${SUDO[@]}" systemctl restart docker

if [[ "$(id -u)" -ne 0 ]]; then
  "${SUDO[@]}" usermod -aG docker "$USER"
  echo "Added $USER to the docker group. Run 'newgrp docker' or log out and back in before using Docker without sudo."
fi

echo "Docker prerequisites installed. Re-run integrations/igibson/scripts/00_check_host.sh to verify GPU access."
