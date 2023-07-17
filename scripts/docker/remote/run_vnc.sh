#!/bin/sh
cd ${HOME}/astrobee/src

# Note: Explicitly specifying loopback interface (127.0.0.1) in -p is
# important for security. Otherwise the VNC server will be accessible
# from outward-facing network interfaces and you should expect a
# warning from network security admins when it is found in a port scan.
docker run -it --rm --name astrobee-vnc -p 127.0.0.1:5900:5900 astrobee/astrobee:latest-vnc-ubuntu20.04
