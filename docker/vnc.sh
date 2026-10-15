#!/bin/bash
# Runs a command on a virtual desktop reachable from the browser at http://localhost:6080/vnc.html
Xvfb :1 -screen 0 ${VNC_RESOLUTION:-1600x900}x24 >/dev/null 2>&1 &
export DISPLAY=:1
sleep 2
fluxbox >/dev/null 2>&1 &
x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5900 >/dev/null 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 >/dev/null 2>&1 &
echo "Desktop available at http://localhost:6080/vnc.html"
exec "$@"
