#!/bin/sh
cd ${HOME}/astrobee/src
docker exec -it --env "DISPLAY=:0" astrobee-vnc bash
