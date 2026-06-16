#!/usr/bin/env bash
# Display Manager Utilities
# Shared utilities for display mode handling (VNC/X11)
set -euo pipefail

DISPLAY_MODE="${DISPLAY_MODE:-vnc}"
DISPLAY_VNC="${DISPLAY_VNC:-:99}"
DISPLAY_X11="${DISPLAY_X11:-${DISPLAY:-:0}}"
VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
XAUTHORITY_FILE="${XAUTHORITY:-/tmp/.Xauthority}"

# Resolve X authority file with fallback
resolve_xauthority_file() {
  local candidate="${XAUTHORITY_FILE}"
  local fallback="${HOME:-/home/intern}/.Xauthority"

  if touch "${candidate}" >/dev/null 2>&1; then
    chmod 600 "${candidate}" >/dev/null 2>&1 || true
    XAUTHORITY_FILE="${candidate}"
    return 0
  fi

  mkdir -p "$(dirname "${fallback}")" >/dev/null 2>&1 || true
  touch "${fallback}" >/dev/null 2>&1 || true
  chmod 600 "${fallback}" >/dev/null 2>&1 || true
  XAUTHORITY_FILE="${fallback}"
}

# Get current display based on mode
resolve_display() {
  if [ "${DISPLAY_MODE}" = "vnc" ]; then
    echo "${DISPLAY_VNC}"
  else
    echo "${DISPLAY_X11}"
  fi
}

# Print display configuration summary
print_mode_summary() {
  echo "Display Mode: ${DISPLAY_MODE}"
  if [ "${DISPLAY_MODE}" = "vnc" ]; then
    echo "  DISPLAY=${DISPLAY_VNC}"
    echo "  VNC_PORT=${VNC_PORT}"
    echo "  NOVNC_PORT=${NOVNC_PORT}"
  else
    echo "  DISPLAY=${DISPLAY_X11}"
  fi
}

# Ensure X11 environment for passthrough mode
ensure_x11_env() {
  export DISPLAY="${DISPLAY_X11}"
  echo "X11 passthrough DISPLAY=${DISPLAY}"
}

# Ensure VNC services are running
ensure_vnc_stack() {
  if [ "${DISPLAY_MODE}" != "vnc" ]; then
    return 0
  fi
  
  echo "[display-utils] Ensuring VNC stack..."
  resolve_xauthority_file
  
  # Start Xvfb
  if ! pgrep -x "Xvfb" >/dev/null; then
    echo "  Starting Xvfb..."
    /usr/bin/Xvfb :99 -screen 0 1920x1080x24 +iglx -ac &
    sleep 2
  fi
  
  # Start fluxbox
  if ! pgrep -x "fluxbox" >/dev/null; then
    echo "  Starting fluxbox..."
    sleep 2 && /usr/bin/fluxbox -display :99 &
  fi
  
  # Start x11vnc
  if ! pgrep -x "x11vnc" >/dev/null; then
    echo "  Starting x11vnc..."
    /usr/bin/x11vnc -display :99 -forever -shared -rfbport 5900 -nopw -noxdamage -noxkb -nowf -nowcr -noshm &
  fi
  
  # Start noVNC
  if ! pgrep -x "websockify" >/dev/null; then
    echo "  Starting noVNC..."
    /usr/bin/websockify --web=/usr/share/novnc 6080 localhost:5900 &
  fi
}

# Recover VNC stack if services are down
recover_vnc_stack() {
  if [ "${DISPLAY_MODE}" != "vnc" ]; then
    echo "[recover] Not in VNC mode, skipping recovery."
    return 1
  fi
  
  echo "[recover] Recovering VNC stack..."
  
  # Kill any stale processes
  pkill -9 -f "Xvfb :99" 2>/dev/null || true
  pkill -9 -f "x11vnc.*:99" 2>/dev/null || true
  pkill -9 -f "websockify.*5900" 2>/dev/null || true
  
  # Restart services
  ensure_vnc_stack
  
  echo "[recover] VNC stack recovery initiated."
}

# Health check for VNC services
vnc_healthcheck() {
  if [ "${DISPLAY_MODE}" != "vnc" ]; then
    return 0
  fi
  
  local retries=5
  local wait_time=2
  
  echo "[healthcheck] Checking VNC services..."
  
  for i in $(seq 1 $retries); do
    if pgrep -x "x11vnc" >/dev/null && pgrep -x "websockify" >/dev/null; then
      echo "[healthcheck] VNC services are healthy."
      return 0
    fi
    
    echo "[healthcheck] Attempt $i/$retries - waiting ${wait_time}s..."
    sleep "$wait_time"
  done
  
  echo "[healthcheck] WARNING: VNC health check failed after $retries attempts."
  return 1
}