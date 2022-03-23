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

# This script is like run.sh, but it:
# (1) Doesn't bother with X display setup. Therefore, you can use it
#     to run a docker container from within a headless VM. (But
#     graphical apps of course won't work.)
# (2) Doesn't start the simulator. Instead, it starts an interactive
#     shell that you can use to run any command in the container.
# (3) Shares the currently checked out source folder into the docker
#     container. This allows you to do a quicker incremental compile and
#     test without rebuilding everything using astrobee.Dockerfile.

# short help
usage_string="$0 usage:  [-h] [-x <use ubuntu 16.04 image>]\
 [-b <use ubuntu 18.04 image>] [-f <use ubuntu 20.04 image>]\
 [-r <download remote image>] [-n]\
\
-n or --no-share: Suppress sharing currently checked out source folder into docker container."
#[-t make_target]
docs_url="https://nasa.github.io/astrobee/html/install-docker.html"

usage()
{
    echo "$usage_string"
    echo "see: $docs_url"
}
os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`
tagrepo=astrobee
script_folder=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
src_dir=${script_folder}/../..
bind_args="--mount type=bind,source=${src_dir},target=/src/astrobee/src"

while [ "$1" != "" ]; do
    case $1 in
        -x | --xenial )                 os="xenial"
                                        ;;
        -b | --bionic )                 os="bionic"
                                        ;;
        -f | --focal )                  os="focal"
                                        ;;
        -r | --remote )                 tagrepo=ghcr.io/nasa
                                        ;;
        -n | --no-share )               bind_args=""
                                        ;;
        -h | --help )                   usage
                                        exit
                                        ;;
        * )                             usage
                                        exit 1
    esac
    shift
done

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
  usage
  exit 1
fi

cd $script_folder

docker run -it --rm --name astrobee \
    $bind_args \
    $tagrepo/$tag \
    /astrobee_init.sh bash
