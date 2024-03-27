#!/bin/sh
cd ${HOME}/ros_ws/astrobee/src

docker run -ti --rm -p 9090:22/tcp --name astrobee-vnc astrobee/astrobee:latest-vnc-ubuntu20.04
