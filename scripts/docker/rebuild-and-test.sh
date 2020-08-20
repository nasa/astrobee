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
usage_string="$scriptname [-h] [-n <use ubuntu 18 installation>]"
#[-t make_target]

usage()
{
    echo "usage: sysinfo_page [[[-a file ] [-i]] | [-h]]"
}
ubuntu18=0

while [ "$1" != "" ]; do
    case $1 in
        -a | --astrobee_source_dir )   shift
                                		astrobee_source=$1
                                		;;
        -n | --ubuntu18 )               ubuntu18=1
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
echo "Astrobee path: "${astrobee_source:-${rootdir}/astrobee/}
if [ $ubuntu18 == 0 ]; then
    docker build ${astrobee_source:-${rootdir}} \
                 -f ${astrobee_source:-${rootdir}/}scripts/docker/rebuild_test_astrobee_kinetic.Dockerfile \
                 -t astrobee/astrobee:test
else
    docker build ${astrobee_source:-${rootdir}} \
                 -f ${astrobee_source:-${rootdir}/}scripts/docker/rebuild_test_astrobee_melodic.Dockerfile \
                 -t astrobee/astrobee:test
fi

