#!/usr/bin/env bash
set -Eeuo pipefail

VENDOR_ROOT="${IGIBSON_VENDOR_ROOT:-/home/lxr20/lxr/igibson_vendor}"
REPO_DIR="${VENDOR_ROOT}/iGibson"
IGIBSON_REPO="${IGIBSON_REPO:-https://github.com/StanfordVL/iGibson.git}"
CLONE_DEPTH="${IGIBSON_CLONE_DEPTH:-1}"
CLONE_RETRIES="${IGIBSON_CLONE_RETRIES:-3}"

mkdir -p "$VENDOR_ROOT"

if [[ -d "$REPO_DIR/.git" ]]; then
  echo "Reusing existing iGibson checkout: $REPO_DIR"
  git -C "$REPO_DIR" fetch --tags --prune
  git -C "$REPO_DIR" submodule update --init --recursive
else
  echo "Cloning iGibson into $REPO_DIR"
  for attempt in $(seq 1 "$CLONE_RETRIES"); do
    if git clone "$IGIBSON_REPO" "$REPO_DIR" --recursive --depth "$CLONE_DEPTH" --shallow-submodules; then
      break
    fi
    rm -rf "$REPO_DIR"
    if [[ "$attempt" -eq "$CLONE_RETRIES" ]]; then
      echo "Failed to clone iGibson after $CLONE_RETRIES attempts." >&2
      exit 1
    fi
    echo "Clone attempt $attempt failed; retrying in 5 seconds..." >&2
    sleep 5
  done
fi

echo "iGibson vendor checkout is ready at $REPO_DIR"
