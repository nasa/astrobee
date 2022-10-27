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
usage: run.sh [-h] [-x] [-b] [-f] [-r] [-n] [-m] [-d]
              [-a "arg1" -a "arg2"]
              -- [cmd [arg1] [arg2] ...]

-h or --help: Print this help
-x or --xenial: Use Ubuntu 16.04 docker image
-b or --bionic: Use Ubuntu 18.04 docker image
-f or --focal: Use Ubuntu 20.04 docker image
-r or --remote: Fetch pre-built remote docker image (vs. local image built by build.sh)
-n or --no-display: Don't set up X forwarding. (For headless environment.)
-g or --gpu: Overwrite default gpu check. (For non nvidia gpu's.)
-m or --mount: Mount the local checkout folder into the docker container.
-d or --dry-run: Just what commands would be run
-i or --image run a different image tag
-a or --args "arg1" --args "arg2": Pass extra args to sim

Run the specified command and args within a docker container.

If no command is specified, the default is to start the Astrobee
simulator.  Pass extra args to the simulator with --args.

If you want an interactive shell, use 'bash' as the command.

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

shortopts="h,x,b,f,r,n,g:,m,d,i:,a:"
longopts="help,xenial,bionic,focal,remote,no-display,gpu:,mount,dry-run,image:,args:"
opts=$(getopt -a -n run.sh --options "$shortopts" --longoptions "$longopts" -- "$@")
if [ $? -ne 0 ]; then
    echo
    usage
    exit 1
fi
echo "opts: $opts"

eval set -- "$opts"

######################################################################
# Parse options 2 (extract variables)
######################################################################

# variable defaults
os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`
sim_args="dds:=false robot:=sim_pub"
display="true"
if [[ $(lshw -C display | grep vendor) =~ Nvidia ]]; then
  gpu="true"
else
  gpu="false"
fi
mount="false"
tagrepo="astrobee"
dry_run="false"
image=""

# extract variables from options
while [ "$1" != "" ]; do
    case $1 in
        -h | --help )                   usage
                                   exit
                                   ;;
        -x | --xenial )            os="xenial"
                                   ;;
        -b | --bionic )            os="bionic"
                                   ;;
        -f | --focal )             os="focal"
                                   ;;
        -r | --remote )            tagrepo="ghcr.io/nasa"
                                   ;;
        -n | --no-display )        display="false"
                                   ;;
        -g | --gpu)                gpu="$2"
                                   shift
                                   ;;
        -m | --mount )             mount="true"
                                   ;;
        -d | --dry-run )           dry_run="true"
                                   ;;
        -i | --image )             image="$2"
                                   shift
                                   ;;
        -a | --args )              sim_args+=" $2"
                                   shift
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
cmd="$*"
if [ -z "$cmd" ]; then
    cmd="roslaunch astrobee sim.launch $sim_args"
fi

# echo "cmd: $cmd"

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
# Set up tag
######################################################################

if [ "$os" = "xenial" ]; then
    tag=astrobee:latest-${image}ubuntu16.04
elif [ "$os" = "bionic" ]; then
    tag=astrobee:latest-${image}ubuntu18.04
elif [ "$os" = "focal" ]; then
    tag=astrobee:latest-${image}ubuntu20.04
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
    display_args+=" --volume="$XSOCK":"$XSOCK":rw"
    display_args+=" --volume="$XAUTH":"$XAUTH":rw"
    display_args+=" --env=XAUTHORITY="${XAUTH}
    display_args+=" --env=DISPLAY"
    if [ "$gpu" = "true" ]; then
        display_args+=" --gpus all"
    fi
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

set -x
cd $script_dir
docker run -it --rm --name astrobee \
       $display_args \
       $mount_args \
       $tagrepo/$tag \
       /astrobee_init.sh $cmd
