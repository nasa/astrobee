#!/bin/sh

# Find root dir
thisdir=$(dirname "$(readlink -f "$0")")
rootdir=$(realpath ${thisdir}/../../..)

# Note: Explicitly specifying loopback interface (127.0.0.1) in -p is
# important for security. Otherwise the SSH server will be accessible
# from outward-facing network interfaces and you should expect a
# warning from network security admins when it is found in a port scan.
docker run -ti --rm -p 9090:22/tcp --name astrobee-ssh astrobee/astrobee:latest-ssh-ubuntu20.04 bash
