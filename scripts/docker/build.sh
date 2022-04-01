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
usage: build.sh [-h] [-x] [-b] [-f] [-r] [-o <owner>] [-d]
                -- [target1] [target2] ...

-h or --help: Print this help
-x or --xenial: Build Ubuntu 16.04 docker images
-b or --bionic: Build Ubuntu 18.04 docker images
-f or --focal: Build Ubuntu 20.04 docker images
-r or --remote: Build first target on top of a pre-built remote image
-o or --owner: Set ghcr.io owner for push action (default: nasa)
-d or --dry-run: Just print what commands would be run

Build specified docker image targets. Available targets:
- astrobee_base
- astrobee
- test_astrobee
- push_astrobee_base (push astrobee_base to ghcr.io)
- push_astrobee (push astrobee to ghcr.io)

Default if no targets are specified: astrobee_base astrobee

Targets are always built in the order listed above. You must
separately run 'docker login' to authenticate before using build.sh
to build the push_* targets.
EOF

set -e

usage()
{
    echo "$usage_string"
}

######################################################################
# Parse options 1 (validate and normalize with getopt)
######################################################################

shortopts="h,x,b,f,r,o:,d"
longopts="help,xenial,bionic,focal,remote,owner:,dry-run"
opts=$(getopt -a -n build.sh --options "$shortops" --longoptions "$longopts" -- "$@")
if [ $? -ne 0 ]; then
    echo
    usage
    exit 1
fi
# echo "opts: $opts"

eval set -- "$opts"

######################################################################
# Parse options 2 (extract variables)
######################################################################

os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`

build_astrobee_base="false"
build_astrobee="false"
build_test_astrobee="false"
push_astrobee_base="false"
push_astrobee="false"

remote="false"
owner="nasa"
dry_run="false"
revision="latest"

while [ "$1" != "" ]; do
    case $1 in
        --help )                   usage
                                   exit
                                   ;;
        --xenial )                 os="xenial"
                                   ;;
        --bionic )                 os="bionic"
                                   ;;
        --focal )                  os="focal"
                                   ;;
        --remote )                 remote="true"
                                   ;;
        --owner )                  owner=$2
                                   shift
                                   ;;
        --dry-run )                dry_run="true"
                                   ;;
        -- )                       shift
                                   break
                                   ;;
        * )                        usage
                                   exit 1
    esac
    shift
done

# remaining arguments are positional, i.e., targets

if [ "$#" -eq 0 ]; then
   # if user didn't specify any targets, set defaults
   build_astrobee_base="true"
   build_astrobee="true"
fi

# collect user-specified targets
while [ "$1" != "" ]; do
    case $1 in
        astrobee_base )            build_astrobee_base="true"
                                   ;;
        astrobee )                 build_astrobee="true"
                                   ;;
        test_astrobee )            build_test_astrobee="true"
                                   ;;
        push_astrobee_base )       push_astrobee_base="true"
                                   ;;
        push_astrobee )            push_astrobee="true"
                                   ;;
        * )                        echo "unknown target '$1'"
                                   usage
                                   exit 1
    esac
    shift
done

if [[ "$build_astrobee_base" == "true" \
          && "$remote" == "true" ]]; then
    echo "Error: --remote doesn't make sense when first target is astrobee_base."
    echo "Run with -h for help."
    exit 1
fi

######################################################################
# Set up version
######################################################################

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
echo "Building Ubuntu $UBUNTU_VERSION images"

######################################################################
# Dry run
######################################################################

if [ "$dry_run" = "true" ]; then
    echo "Dry run"

    docker()
    {
        # dry run, do nothing
        { : ; } 2>/dev/null
    }
fi

######################################################################
# Define actions
######################################################################

build () {
    stage=$1
    tag_revision=$2
    tag_stage=$3

    echo ======================================================================
    echo "Building ${stage}"

    remote_args=""
    if [ "$remote" = "true" ]; then
        echo "[Building on top of remote image]"
        # note: pull from ghcr.io/nasa even if we push to another owner
        remote_args="--build-arg REMOTE=ghcr.io/nasa"
        remote="false"  # build subsequent targets on local image
    fi

    set -x
    docker build . \
           -f ./scripts/docker/${stage}.Dockerfile \
           --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
           --build-arg ROS_VERSION=${ROS_VERSION} \
           --build-arg PYTHON=${PYTHON} \
           $remote_args \
           -t astrobee/astrobee:${tag_revision}${tag_stage}ubuntu${UBUNTU_VERSION}
    { set +x; } 2>/dev/null
}

push () {
    stage=$1
    tag_revision=$2
    tag_stage=$3

    tag_suffix="astrobee:${tag_revision}${tag_stage}ubuntu${UBUNTU_VERSION}"
    local_tag="astrobee/$tag_suffix"
    remote_tag="ghcr.io/${owner}/$tag_suffix"

    echo ======================================================================
    echo "Pushing ${stage}"

    set -x
    docker tag $local_tag $remote_tag
    docker push $remote_tag
    { set +x; } 2>/dev/null
}

######################################################################
# Run
######################################################################

script_dir=$(dirname "$(readlink -f "$0")")
checkout_dir=$(dirname $(dirname "$script_dir"}))
echo "Astrobee checkout path: "${checkout_dir}/

set -x
cd "${checkout_dir}"
{ set +x; } 2>/dev/null

if [ "$build_astrobee_base" = "true" ]; then
    build astrobee_base "${revision}-" "base-"
fi

if [ "$build_astrobee" = "true" ]; then
    build astrobee "${revision}-" ""
fi

if [ "$build_test_astrobee" = "true" ]; then
    build test_astrobee "" "test-"
fi

if [ "$push_astrobee_base" = "true" ]; then
    push astrobee_base "${revision}-" "base-"
fi

if [ "$push_astrobee" = "true" ]; then
    push astrobee "${revision}-" ""
fi
