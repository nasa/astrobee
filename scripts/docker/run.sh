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

# short help
usage_string="$0 usage:  [-h] [-x <use ubuntu 16.04 image>]\
 [-b <use ubuntu 18.04 image>] [-f <use ubuntu 20.04 image>]\
 [-r <download remote image>]"
#[-t make_target]
docs_url="https://nasa.github.io/astrobee/html/install-docker.html"

usage()
{
    echo "$usage_string"
    echo "see: $docs_url"
}
os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`
args="dds:=false robot:=sim_pub"
useremotetag=0

while [ "$1" != "" ]; do
    case $1 in
        -x | --xenial )                 os="xenial"
                                        ;;
        -b | --bionic )                 os="bionic"
                                        ;;
        -f | --focal )                  os="focal"
                                        ;;
        -r | --remote )                 useremotetag=1
                                        ;;
        --args )                        args+=" $2"
                                        shift
                                        ;;
        -h | --help )                   usage
                                        exit
                                        ;;
        * )                             usage
                                        exit 1
    esac
    shift
done


rootdir=$(dirname "$(readlink -f "$0")")
cd $rootdir

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

if [ "$os" = "xenial" ]; then
localtag=astrobee/astrobee:latest-ubuntu16.04
remotetag=ghcr.io/nasa/astrobee:latest-ubuntu16.04
elif [ "$os" = "bionic" ]; then
localtag=astrobee/astrobee:latest-ubuntu18.04
remotetag=ghcr.io/nasa/astrobee:latest-ubuntu18.04
elif [ "$os" = "focal" ]; then
localtag=astrobee/astrobee:latest-ubuntu20.04
remotetag=ghcr.io/nasa/astrobee:latest-ubuntu20.04
fi

# check if docker tag exists
if [ $useremotetag -eq 1 ]; then
  tag=$remotetag
elif [[ "$(docker images -q $localtag 2> /dev/null)" == "" ]]; then
  echo "No local tag found, either build astrobee locally or use the --remote flag."
  usage
  exit 1
else
  tag=$localtag
fi

docker run -it --rm --name astrobee \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY" \
        --gpus all \
      $tag \
    /astrobee_init.sh roslaunch astrobee sim.launch $args
