#!/bin/sh
set -e

cd ${HOME}/ros_ws/astrobee/src
docker build . -f ./scripts/docker/remote/astrobee_ssh.Dockerfile --build-arg UBUNTU_VERSION=20.04 --build-arg REMOTE=ghcr.io/nasa -t astrobee/astrobee:latest-vnc-ubuntu20.04
