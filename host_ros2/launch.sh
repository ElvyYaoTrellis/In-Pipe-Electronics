#!/bin/bash

# ROS 2 Humble launcher (Joystick + RTSP streaming + IMU viz)
# - Starts joy_node (ROS2)
# - Starts host_udp.py
# - Starts IMU_host.py

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST_SCRIPT="$SCRIPT_DIR/host_esp_base.py"
IMU_SCRIPT="$SCRIPT_DIR/IMU_host.py"   # <-- adjust if your file is elsewhere

# Source ROS 2 Humble safely even under `set -u`
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +u
source /opt/ros/humble/setup.bash
set -u

echo "[INFO] Killing existing processes (best-effort)..."
pkill -f "ros2 run joy joy_node" 2>/dev/null || true
pkill -f "host_esp_base.py" 2>/dev/null || true
pkill -f "python3 .*host_udp.py" 2>/dev/null || true
pkill -f "python3 .*IMU_host.py" 2>/dev/null || true   # <-- added
sleep 1

JOY_CMD="source /opt/ros/humble/setup.bash; ros2 run joy joy_node"
HOST_CMD="source /opt/ros/humble/setup.bash; cd \"$SCRIPT_DIR\"; python3 \"$HOST_SCRIPT\""
IMU_CMD="source /opt/ros/humble/setup.bash; cd \"$SCRIPT_DIR\"; python3 \"$IMU_SCRIPT\""  # <-- added

run_in_terminal_or_bg() {
  local title="$1"; shift
  local cmd="$*"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -lc "$cmd; exec bash"
    return 0
  fi

  echo "[WARN] gnome-terminal not found; running '$title' in background."
  bash -lc "$cmd" &
  echo $!
}

echo "[INFO] Starting joy_node..."
JOY_PID=$(run_in_terminal_or_bg "joy_node" "$JOY_CMD") || true

echo "[INFO] Starting joystick+stream app..."
HOST_PID=$(run_in_terminal_or_bg "joy_stream" "$HOST_CMD") || true

# echo "[INFO] Starting IMU host visualizer..."
# IMU_PID=$(run_in_terminal_or_bg "imu_host" "$IMU_CMD") || true

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[INFO] Background PIDs: joy=${JOY_PID:-n/a} host=${HOST_PID:-n/a} imu=${IMU_PID:-n/a}"
  echo "[INFO] Press Ctrl+C here to stop."

  trap 'echo "[INFO] Stopping..."; kill ${JOY_PID:-} 2>/dev/null || true; kill ${HOST_PID:-} 2>/dev/null || true; kill ${IMU_PID:-} 2>/dev/null || true; exit 0' INT TERM
  while true; do sleep 1; done
fi
