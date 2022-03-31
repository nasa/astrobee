#!/bin/bash
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

read -r -d '' usage_string <<EOF
usage: build.sh [-h] [-x] [-b] [-f] [-r] [-d]
                [target1] [target2] ...

-h or --help: Print this help
-x or --xenial: Build Ubuntu 16.04 docker images
-b or --bionic: Build Ubuntu 18.04 docker images
-f or --focal: Build Ubuntu 20.04 docker images
-r or --remote: Build first target on top of a pre-built remote image
-d or --dry-run: Just print what commands would be run

Build specified docker image targets. Available targets:
- astrobee_base
- astrobee
- test_astrobee

Default if no targets are specified: astrobee_base astrobee

Targets are always built in the order listed above.
EOF

set -e

usage()
{
    echo "$usage_string"
}

os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`

build_astrobee_base="false"
build_astrobee="false"
build_test_astrobee="false"

remote="false"
dry_run="false"

while [ "$1" != "" ]; do
    case $1 in
        -h | --help )                   usage
                                        exit
                                        ;;
        -x | --xenial )                 os="xenial"
                                        ;;
        -b | --bionic )                 os="bionic"
                                        ;;
        -f | --focal )                  os="focal"
                                        ;;
	-r | --remote )                 remote="true"
					;;
	-d | --dry-run )                dry_run="true"
					;;
	astrobee_base )                 build_astrobee_base="true"
					;;
	astrobee )                      build_astrobee="true"
					;;
	test_astrobee )                 build_test_astrobee="true"
					;;
        * )                             usage
                                        exit 1
    esac
    shift
done

if [ "$dry_run" = "true" ]; then
    echo "Dry run"

    docker()
    {
	# dry run, do nothing
	{ : ; } 2>/dev/null
    }
fi

if [[ "$build_astrobee_base" == "false" \
	  && "$build_astrobee" == "false" \
	  && "$build_test_astrobee" == "false" ]]; then
   # if user didn't specify any targets, set defaults
   build_astrobee_base="true"
   build_astrobee="true"
fi

if [[ "$build_astrobee_base" == "true" \
	  && "$remote" == "true" ]]; then
    echo "Error: --remote doesn't make sense when first target is astrobee_base."
    echo "Run with -h for help."
    exit 1
fi

script_dir=$(dirname "$(readlink -f "$0")")
checkout_dir=$(dirname $(dirname "$script_dir"}))
echo "Astrobee checkout path: "${checkout_dir}/

UBUNTU_VERSION=16.04
ROS_VERSION=kinetic
PYTHON=''

if [ "$os" = "bionic" ]; then
    UBUNTU_VERSION=18.04
    ROS_VERSION=melodic
    PYTHON=''
elif [ "$os" = "focal" ]; then
    UBUNTU_VERSION=20.04
    ROS_VERSION=noetic
    PYTHON='3'
fi

remote_repo="ghcr.io/nasa"

echo "Building Ubuntu $UBUNTU_VERSION images"

if [ "$build_astrobee_base" = "true" ]; then
    echo ======================================================================
    echo "Building astrobee_base"
    set -x
    docker build ${checkout_dir}/ \
           -f ${checkout_dir}/scripts/docker/astrobee_base.Dockerfile \
           --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
           --build-arg ROS_VERSION=${ROS_VERSION} \
           --build-arg PYTHON=${PYTHON} \
           -t astrobee/astrobee:latest-base-ubuntu${UBUNTU_VERSION}
    { set +x; } 2>/dev/null
fi

if [ "$build_astrobee" = "true" ]; then
    echo ======================================================================
    echo "Building astrobee"
    remote_args=""
    if [ "$remote" = "true" ]; then
	echo "[Building on top of remote image]"
	remote_args="--build-arg REMOTE=${remote_repo}"
	remote="false"  # build subsequent targets on local image
    fi
    set -x
    docker build ${checkout_dir}/ \
           -f ${checkout_dir}/scripts/docker/astrobee.Dockerfile \
           --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
           --build-arg ROS_VERSION=${ROS_VERSION} \
	   $remote_args \
           -t astrobee/astrobee:latest-ubuntu${UBUNTU_VERSION}
    { set +x; } 2>/dev/null
fi

if [ "$build_test_astrobee" = "true" ]; then
    echo ======================================================================
    echo "Building test_astrobee"
    remote_args=""
    if [ "$remote" = "true" ]; then
	echo "[Building on top of remote image]"
	remote_args="--build-arg REMOTE=${remote_repo}"
	remote="false"  # build subsequent targets on local image
    fi
    set -x
    docker build ${checkout_dir}/ \
           -f ${checkout_dir}/scripts/docker/test_astrobee.Dockerfile \
           --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
	   $remote_args \
           -t astrobee/astrobee:test-ubuntu${UBUNTU_VERSION}
    { set +x; } 2>/dev/null
fi
