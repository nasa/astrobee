#!/bin/sh

# This script is designed to be run inside the container (Dockerfile "CMD") to start up the necessary daemons.

export DISPLAY=${DISPLAY:-:0} # Select screen 0 by default.
x11vnc -bg -forever -nopw -quiet -display WAIT$DISPLAY &
Xvfb $DISPLAY -screen 0 1024x768x16 &
sleep 1

xdpyinfo

# disable screen saver and power management
xset -dpms &
xset s noblank &
xset s off &

# start window manager
/usr/bin/startxfce4 --replace > $HOME/wm.log &

# CMD script should not exit
sleep infinity
