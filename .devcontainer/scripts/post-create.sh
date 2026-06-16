#!/usr/bin/env bash
# Post-Create Setup Script
# Runs once when container is first created to initialize environment
set -euo pipefail

WORKSPACE_PATH="${1:-}"

if [ -z "${WORKSPACE_PATH}" ]; then
  echo "Usage: $0 <containerWorkspaceFolder>"
  exit 1
fi

mkdir -p "${WORKSPACE_PATH}"
cd "${WORKSPACE_PATH}"

echo "============================================================================="
echo "DevContainer Post-Create Script"
echo "============================================================================="

# Persist GUI/VNC environment defaults for login shells
if command -v sudo >/dev/null 2>&1; then
  sudo tee /etc/profile.d/99-vnc-display.sh > /dev/null <<'EOF'
if [ "${DISPLAY_MODE:-vnc}" = "vnc" ]; then
  export DISPLAY="${DISPLAY_VNC:-:99}"
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
else
  export DISPLAY="${DISPLAY_X11:-${DISPLAY:-:0}}"
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
fi
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
# GPU-accelerated OpenGL via VirtualGL when available; fall back to software rendering.
# Use: vglrun <opengl-app>  (e.g. vglrun rviz2)
if [ -e /dev/dri/renderD128 ] && command -v vglrun >/dev/null 2>&1; then
  export VGL_DISPLAY=egl
else
  export LIBGL_ALWAYS_SOFTWARE=1
fi
EOF
  sudo chmod 0644 /etc/profile.d/99-vnc-display.sh
fi

# Sync devcontainer scripts to system locations so workspace edits take effect
# without requiring a full container rebuild.
if command -v sudo >/dev/null 2>&1; then
  sudo cp "${WORKSPACE_PATH}/.devcontainer/scripts/display-utils.sh" /usr/local/bin/display-utils.sh
  sudo chmod +x /usr/local/bin/display-utils.sh
  sudo cp "${WORKSPACE_PATH}/.devcontainer/scripts/entrypoint.sh" /usr/local/bin/entrypoint.sh
  sudo chmod +x /usr/local/bin/entrypoint.sh
  sudo cp "${WORKSPACE_PATH}/.devcontainer/scripts/post-start.sh" /usr/local/bin/post-start.sh
  sudo chmod +x /usr/local/bin/post-start.sh
  sudo cp "${WORKSPACE_PATH}/.devcontainer/supervisord.conf" /etc/supervisor/conf.d/supervisord.conf
  # Ensure intern-vnc log dir exists with correct permissions
  sudo mkdir -p /tmp/intern-vnc
  sudo chmod 1777 /tmp/intern-vnc
  echo "post-create: VNC system scripts synced from workspace."
fi

# Update bashrc with necessary environment variables
for rc in /home/intern/.bashrc /root/.bashrc; do
  if [ -f "$rc" ]; then
    grep -q "export QT_X11_NO_MITSHM=1" "$rc" 2>/dev/null || echo "export QT_X11_NO_MITSHM=1" >> "$rc"
    grep -q "DISPLAY_MODE:-vnc" "$rc" 2>/dev/null || cat >> "$rc" <<'EOF'
if [ "${DISPLAY_MODE:-vnc}" = "vnc" ]; then
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
fi
EOF
  fi
done

# Helper to mark git repos as safe when ownership differs
ensure_git_safe_dir() {
  local repo="$1"
  if command -v git >/dev/null 2>&1 && [ -d "$repo/.git" ]; then
    git config --global --add safe.directory "$repo" || true
  fi
}

# Add ROS 2 environment to bashrc
if ! grep -q "source /opt/ros/humble/setup.bash" /root/.bashrc 2>/dev/null; then
  if [ -w /root/.bashrc ]; then
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
  elif command -v sudo >/dev/null 2>&1; then
    echo "source /opt/ros/humble/setup.bash" | sudo tee -a /root/.bashrc > /dev/null
  fi
fi

# Fix apt cache directory ownership issues before running apt-get
sudo mkdir -p /var/lib/apt/lists/partial
sudo chown -R root:root /var/lib/apt/lists
sudo chmod -R 755 /var/lib/apt/lists

sudo apt-get update -qq

set +u
source /opt/ros/humble/setup.bash
set -u

rosdep update --rosdistro humble || true
echo "post-create: rosdep database updated."

echo "Post-create setup complete."