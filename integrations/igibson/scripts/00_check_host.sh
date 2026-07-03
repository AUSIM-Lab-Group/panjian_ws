#!/usr/bin/env bash
set -Eeuo pipefail

CUDA_TEST_IMAGE="${CUDA_TEST_IMAGE:-nvidia/cuda:12.2.0-base-ubuntu20.04}"

section() {
  printf '\n== %s ==\n' "$1"
}

run_optional() {
  local label="$1"
  shift
  section "$label"
  if "$@"; then
    return 0
  fi
  local status=$?
  printf 'WARN: command failed with exit code %s\n' "$status" >&2
  return 0
}

section "OS"
if command -v lsb_release >/dev/null 2>&1; then
  lsb_release -a
else
  cat /etc/os-release
fi
uname -a

section "Display"
printf 'DISPLAY=%s\n' "${DISPLAY:-}"
printf 'XDG_SESSION_TYPE=%s\n' "${XDG_SESSION_TYPE:-}"
if command -v xhost >/dev/null 2>&1; then
  xhost >/dev/null && echo "xhost: available" || echo "xhost: available but current X access query failed"
else
  echo "xhost: missing"
fi

run_optional "NVIDIA GPU" nvidia-smi

section "Docker CLI"
if command -v docker >/dev/null 2>&1; then
  docker --version
  docker info --format 'Docker server: {{.ServerVersion}}' 2>/dev/null || echo "Docker server is not reachable"
else
  echo "docker: missing"
fi

section "NVIDIA Container Toolkit"
if command -v nvidia-ctk >/dev/null 2>&1; then
  nvidia-ctk --version
elif command -v nvidia-container-toolkit >/dev/null 2>&1; then
  nvidia-container-toolkit --version
else
  echo "nvidia-container-toolkit: missing"
fi

section "Docker GPU Smoke Test"
if command -v docker >/dev/null 2>&1; then
  if docker info >/dev/null 2>&1; then
    docker run --rm --gpus all "$CUDA_TEST_IMAGE" nvidia-smi
  else
    echo "Skipped: Docker server is not reachable for this user"
  fi
else
  echo "Skipped: docker is not installed"
fi
