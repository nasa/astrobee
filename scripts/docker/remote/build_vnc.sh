#!/bin/sh
cd ${HOME}/astrobee/src
docker build . -f ./scripts/docker/astrobee_vnc.Dockerfile --build-arg UBUNTU_VERSION=20.04 --build-arg REMOTE=ghcr.io/nasa -t astrobee/astrobee:latest-vnc-ubuntu20.04
