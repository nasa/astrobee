#!/bin/sh
cd ${HOME}/ros_ws/astrobee/src
docker exec -it --env "DISPLAY=:0" astrobee-vnc bash
