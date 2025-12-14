#!/usr/bin/env bash

# ===== CONFIG =====
OLD_PATH="/home/shady/Documents/unige_robotics_msc/0x03_second_year_3rd_semester/experimental/ros2_ws"

# Detect current ros2_ws absolute path
ROS2_WS_PATH="$(cd "$(dirname "$0")" && pwd)"

echo "Replacing:"
echo "  OLD: $OLD_PATH"
echo "  NEW: $ROS2_WS_PATH"
echo

# Safety check
if [[ ! -d "$ROS2_WS_PATH/src" ]]; then
  echo "❌ Error: This script must be placed inside ros2_ws"
  exit 1
fi

echo "🔍 Scanning files (excluding build/, install/, log/, .venv/)..."
echo

find . \
  -type d \( -name build -o -name install -o -name log -o -name .venv \) -prune -false \
  -o -type f -print \
| while read -r file; do
    if grep -q "$OLD_PATH" "$file"; then
      echo "✔ Updating $file"
      sed -i "s|$OLD_PATH|$ROS2_WS_PATH|g" "$file"
    fi
  done

echo
echo "✅ All paths updated successfully."
