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
set -e

# short help
usage_string="$scriptname [-h] [-x <use ubuntu 16 installation>]
    [-b <use ubuntu 18 installation>] [-f <use ubuntu 20 installation>]"
#[-t make_target]

usage()
{
    echo "usage: sysinfo_page [[[-a file ] [-i]] | [-h]]"
}

os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`

while [ "$1" != "" ]; do
    case $1 in
        -x | --xenial )                 os="xenial"
                                        ;;
        -b | --bionic )                 os="bionic"
                                        ;;
        -f | --focal )                  os="focal"
                                        ;;
        -h | --help )                   usage
                                        exit
                                        ;;
        * )                             usage
                                        exit 1
    esac
    shift
done


thisdir=$(dirname "$(readlink -f "$0")")
rootdir=${thisdir}/../..
echo "Astrobee path: "${rootdir}/

UBUNTU_VERSION=ubuntu16.04
ROS_VERSION=kinetic
PYTHON=''

if [ "$os" = "bionic" ]; then
  UBUNTU_VERSION=ubuntu18.04
  ROS_VERSION=bionic
  PYTHON=''

elif [ "$os" = "focal" ]; then
  UBUNTU_VERSION=ubuntu20.04
  ROS_VERSION=noetic
  PYTHON='3'
fi

echo "Building $UBUNTU_VERSION image"
docker build ${rootdir}/ \
            -f ${rootdir}/scripts/docker/astrobee_base.Dockerfile \
            --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
            --build-arg ROS_VERSION=${ROS_VERSION} \
            --build-arg PYTHON=${PYTHON} \
            -t astrobee/astrobee:base-latest-${UBUNTU_VERSION}
docker build ${rootdir}/ \
            -f ${rootdir}/scripts/docker/astrobee.Dockerfile \
            --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
            -t astrobee/astrobee:latest-${UBUNTU_VERSION}