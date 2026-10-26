#!/usr/bin/env bash
# Starts the virtual desktop (VNC on :1, noVNC on port 6080), then runs the command.
# Set HEADLESS=1 to skip the desktop, e.g. in CI.
set -e

if [[ "${HEADLESS:-0}" != "1" ]]; then
  rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
  vncserver :1 -geometry "${VNC_GEOMETRY:-1600x900}" -depth 24 \
    -SecurityTypes None -localhost yes --I-KNOW-THIS-IS-INSECURE \
    -xstartup /usr/bin/openbox-session > /tmp/vnc.log 2>&1
  websockify --web /usr/share/novnc 6080 localhost:5901 > /tmp/novnc.log 2>&1 &
fi

source /opt/ros/humble/setup.bash
exec "$@"
