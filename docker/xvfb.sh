#!/bin/bash
# Runs a command on a virtual display (headless rendering of the robot cameras)
Xvfb :99 -screen 0 1280x1024x24 >/dev/null 2>&1 &
export DISPLAY=:99
sleep 2
exec "$@"
