#!/usr/bin/env bash
# Unified Container Entry Point
# Handles both VNC and X11 passthrough modes
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/display-utils.sh"

# Ensure the per-user VNC log directory exists and is writable by intern
mkdir -p /tmp/intern-vnc 2>/dev/null || true
chmod 1777 /tmp/intern-vnc 2>/dev/null || true

echo "============================================================================="
echo "Container Entry Point"
print_mode_summary
echo "============================================================================="

if [ "${DISPLAY_MODE}" = "vnc" ]; then
    echo "Starting in VNC mode"
    
    # Prepare X11 socket directory
    mkdir -p /tmp/.X11-unix
    chmod 1777 /tmp/.X11-unix || true
    
    # Initialize display services early
    ensure_vnc_stack
    vnc_healthcheck || {
        echo "WARNING: Early VNC health-check failed, attempting full recovery..."
        recover_vnc_stack
        vnc_healthcheck || echo "ERROR: noVNC still unavailable; check /tmp/intern-vnc/websockify.log and /tmp/intern-vnc/x11vnc.log"
    }
    echo "VNC services ready on DISPLAY=${DISPLAY_VNC}."
    
    # Hand off to supervisord for managed autorestart of all VNC services
    exec /usr/bin/supervisord -n -c /etc/supervisor/conf.d/supervisord.conf
else
    echo "Starting in X11 passthrough mode"
    ensure_x11_env
    exec tail -f /dev/null
fi