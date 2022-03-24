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
usage: run.sh [-h] [-x] [-b] [-f] [-r] [-n] [-m]
              [-s] [-a "arg1 arg2"]
              -- [cmd [arg1] [arg2] ...]

-h or --help: Print this help
-x or --xenial: Use Ubuntu 16.04 docker image
-b or --bionic: Use Ubuntu 18.04 docker image
-f or --focal: Use Ubuntu 20.04 docker image
-r or --remote: Fetch pre-built remote docker image (vs. local image built by build.sh)
-n or --no-display: Don't set up X forwarding. (For headless environment.)
-m or --mount-directory: Mount the local checkout folder into the docker container.
-s or --sim: Run the Astrobee simulator
-a or --sim-args "arg1 arg2": Pass extra args to sim

Run the specified command and args within a docker container. If no
command is specified, the default is to provide an interactive
shell with 'bash'.

With --sim, run the standard command to start the Astrobee simulator.
Pass extra args to the simulator with --sim-args.

The -xbfr flags specify the type of docker container to run the command in.
EOF

docs_url="https://nasa.github.io/astrobee/html/install-docker.html"

usage()
{
    echo "$usage_string"
    echo
    echo "See also: $docs_url"
}

######################################################################
# Parse options 1 (validate and normalize with getopt)
######################################################################

shortopts="x,b,f,r,n,m,s,a:,h"
longopts="xenial,bionic,focal,remote,no-display,mount-directory,sim,sim-args:,help"
opts=$(getopt -a -n run.sh --options "$shortops" --longoptions "$longopts" -- "$@")
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

# variable defaults
os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`
sim_args="dds:=false robot:=sim_pub"
display="true"
mount="false"
sim="false"
tagrepo=astrobee

# extract variables from options
while [ "$1" != "" ]; do
    case $1 in
        --xenial )                 os="xenial"
                                   ;;
        --bionic )                 os="bionic"
                                   ;;
        --focal )                  os="focal"
                                   ;;
        --remote )                 tagrepo="ghcr.io/nasa"
                                   ;;
        --no-display )             display="false"
                                   ;;
        --mount-directory )        mount="true"
                                   ;;
        --sim )                    sim="true"
                                   ;;
        --sim-args )               sim_args+=" $2"
                                   shift
                                   ;;
        --help )                   usage
                                   exit
                                   ;;
        -- )                       shift
                                   break
                                   ;;
        * )                        usage
                                   exit 1
    esac
    shift
done

# collect remaining non-option arguments
user_cmd="$*"

######################################################################
# Set up command
######################################################################

if [ "$sim" = "true" ]; then
    if [ -z "$user_cmd" ]; then
        # run simulator
        cmd="roslaunch astrobee sim.launch $sim_args"
    else
        echo "Specify --sim or your own command, not both."
        echo
        usage
        exit 1
    fi
else
    if [ -z "$user_cmd" ]; then
        # run interactive shell
        cmd="bash"
    else
        # run the user's command
        cmd="$user_cmd"
    fi
fi
# echo "cmd: $cmd"

######################################################################
# Set up tag
######################################################################

if [ "$os" = "xenial" ]; then
    tag=astrobee:latest-ubuntu16.04
elif [ "$os" = "bionic" ]; then
    tag=astrobee:latest-ubuntu18.04
elif [ "$os" = "focal" ]; then
    tag=astrobee:latest-ubuntu20.04
fi

# check if local docker tag exists
if [ "$tagrepo" = "astrobee" ] && [[ "$(docker images -q $tagrepo/$tag 2> /dev/null)" == "" ]]; then
  echo "Tag $tagrepo/$tag not found locally, either build astrobee locally or use the --remote flag."
  echo
  usage
  exit 1
fi

######################################################################
# Set up display
######################################################################

if [ -z "$DISPLAY" ]; then
    echo "Detected DISPLAY is not set. Running with --no-display."
    display="false"
fi

display_args=""
if [ "$display" = "true" ]; then
    # setup XServer for Docker
    XSOCK=/tmp/.X11-unix
    XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    display_args+=" --volume=$XSOCK:$XSOCK:rw"
    display_args+=" --volume=$XAUTH:$XAUTH:rw"
    display_args+=" --env=\"XAUTHORITY=${XAUTH}\""
    display_args+=" --env=\"DISPLAY\""
    display_args+=" --gpus all"
fi

######################################################################
# Set up mount
######################################################################

script_dir=$(dirname "$(readlink -f "$0")")
src_dir=$(dirname $(dirname "$script_dir"))

mount_args=""
if [ "$mount" = "true" ]; then
    mount_args="--mount type=bind,source=${src_dir},target=/src/astrobee/src"
fi

######################################################################
# Run
######################################################################

cd $script_dir
docker run -it --rm --name astrobee \
       $display_args \
       $mount_args \
       $tagrepo/$tag \
       /astrobee_init.sh $cmd
