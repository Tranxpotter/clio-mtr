#!/usr/bin/env bash
# Post-Start Initialization Script
# Runs after container is started to initialize workspace and services
set -euo pipefail

WORKSPACE_PATH="${1:-}"
if [ -z "${WORKSPACE_PATH}" ]; then
  echo "Usage: $0 <containerWorkspaceFolder>"
  exit 1
fi

# Running as root inside a user namespace can lead to strange permission errors
# The devcontainer normally invokes this script as the non-root user,
# so only use sudo when absolutely necessary.
if [ "$(id -u)" -eq 0 ]; then
  echo "[warning] post-start.sh running as root; you may see permission errors."
  echo "          try invoking without sudo unless you really need it."
fi

mkdir -p "${WORKSPACE_PATH}"
cd "${WORKSPACE_PATH}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/display-utils.sh"

echo "============================================================================="
echo "DevContainer Post-Start Script"
print_mode_summary
echo "============================================================================="

# Helper to mark git repos as safe when ownership differs (fixes 'dubious ownership' errors)
ensure_git_safe_dir() {
  local repo="$1"
  if command -v git >/dev/null 2>&1 && [ -d "$repo/.git" ]; then
    git config --global --add safe.directory "$repo" || true
  fi
}

# Initialize display services early so noVNC/X11 is available immediately
if [ "${DISPLAY_MODE}" = "vnc" ]; then
  echo "Bringing up VNC services early..."
  ensure_vnc_stack
  vnc_healthcheck || {
    echo "WARNING: Early VNC health-check failed, attempting full recovery..."
    recover_vnc_stack
    vnc_healthcheck || echo "ERROR: noVNC still unavailable; check /tmp/intern-vnc/websockify.log and /tmp/intern-vnc/x11vnc.log"
  }
  echo "VNC services ready on DISPLAY=${DISPLAY_VNC}."
else
  ensure_x11_env
fi

# Source ROS 2 environment
set +u
source /opt/ros/humble/setup.bash
set -u

export CYCLONEDDS_URI="file://${WORKSPACE_PATH}/.devcontainer/cyclonedds.xml"

# Build Livox-SDK2 if not installed
if [ ! -f "/usr/local/lib/liblivox_lidar_sdk_shared.so" ]; then
  echo "Building Livox-SDK2..."
  cd "${WORKSPACE_PATH}"
  if [ -d "Livox-SDK2" ]; then
    rm -rf Livox-SDK2
  fi
  git clone https://github.com/Livox-SDK/Livox-SDK2.git
  ensure_git_safe_dir "${WORKSPACE_PATH}/Livox-SDK2"
  cd "${WORKSPACE_PATH}/Livox-SDK2"
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  # Install (use sudo if available)
  if command -v sudo >/dev/null 2>&1; then
    sudo make install
    sudo ldconfig
  else
    make install
    ldconfig || true
  fi
  echo "Livox-SDK2 built and installed."
fi

# Source workspace environment (includes ROS 2 + all packages)
set +u
source "${WORKSPACE_PATH}/install/setup.bash"
set -u
echo "Workspace environment sourced from ${WORKSPACE_PATH}/install/setup.bash."

# Clean stale ament_cmake_python directory conflicts before workspace build
echo "Cleaning potential symlink conflicts before workspace build..."

find "${WORKSPACE_PATH}/build" -type d -path "*/ament_cmake_python/*/*" 2>/dev/null | while IFS= read -r stale_dir; do
  echo "Removing stale ament_cmake_python dir: ${stale_dir}"
  rm -rf "${stale_dir}" 2>/dev/null || true
done

echo "Post-start initialization complete."