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

DIST=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`

while [ "$1" != "" ]; do
    case $1 in
        -x | --xenial )                 os="xenial"
                                        ;;
        -b | --bionic )                 os="bionic"
                                        ;;
        -f | --focal )                  os="focal"
                                        ;;
        -h | --help )           		usage
                                		exit
                                		;;
        * )                     		usage
                                		exit 1
    esac
    shift
done


thisdir=$(dirname "$(readlink -f "$0")")
rootdir=${thisdir}/../..
echo "Astrobee path: "${rootdir}/
if [ "$os" = "xenial" ]; then
    docker build ${rootdir}/ \
                -f ${rootdir}/scripts/docker/astrobee_base_xenial.Dockerfile \
                -t astrobee/astrobee:base-latest-xenial
    docker build ${rootdir}/ \
                -f ${rootdir}/scripts/docker/astrobee_xenial.Dockerfile \
                -t astrobee/astrobee:latest-xenial
elif [ "$os" = "bionic" ]; then
    docker build ${rootdir}/ \
                -f ${rootdir}/scripts/docker/astrobee_base_bionic.Dockerfile \
                -t astrobee/astrobee:base-latest-bionic
    docker build ${rootdir}/ \
                -f ${rootdir}/scripts/docker/astrobee_bionic.Dockerfile \
                -t astrobee/astrobee:latest-bionic
elif [ "$os" = "focal" ]; then
    docker build ${rootdir}/ \
                -f ${rootdir}/scripts/docker/astrobee_base_focal.Dockerfile \
                -t astrobee/astrobee:base-latest-focal
    docker build ${rootdir}/ \
                -f ${rootdir}/scripts/docker/astrobee_focal.Dockerfile \
                -t astrobee/astrobee:latest-focal
fi

