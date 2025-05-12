#!/bin/bash
CONFIG_FILE=".colcon_build_config"

# Colcon workspaceのルートで実行してください
selected_packages=$(colcon list --names-only | fzf --multi --prompt="Select packages: ")

if [ -z "$selected_packages" ]; then
  echo "No packages selected."
  exit 1
fi

# === 1. 読み込み ===
if [ -f "$CONFIG_FILE" ]; then
  source "$CONFIG_FILE"
fi

# === 2. parallel-workers の入力（保存済みでなければ）===
read -rp "Enter number of parallel workers (default = ${PARALLEL_WORKERS}): " PARALLEL_WORKERS_input

# if PARALLEL_WORKERS_input is empty, use default
if [ -z "$PARALLEL_WORKERS_input" ]; then
    PARALLEL_WORKERS_input=$PARALLEL_WORKERS
else
    echo "PARALLEL_WORKERS=$PARALLEL_WORKERS_input" > "$CONFIG_FILE"
fi

echo "parallel-workers $PARALLEL_WORKERS_input"
echo "Building selected packages:"
echo "$selected_packages"


colcon build --symlink-install --packages-up-to $selected_packages --parallel-workers $PARALLEL_WORKERS_input
