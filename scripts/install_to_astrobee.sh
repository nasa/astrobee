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
#
# usage: install_to_astrobee armhf_dir [ config_version=p4 ]

shopt -s extglob

# Check to see if there are arguments
if [ $# -eq 0 ]; then
  echo "Please supply the armhf folder as an argument."
  exit 1
fi

source $(dirname "$0")/deploy/constants.sh

# find which robot is available (or if the specified one is available, if any specified
robot_index=-1
for i in "${!robot_names[@]}"
do
  if [ $# -gt 1 ] && [ $2 != ${robot_names[$i]} ]; then
    continue;
  fi
  if ping -w 1 -q -c 1 "${mlp_ips[$i]}" > /dev/null
  then
    robot_index=$i
    break;
  fi
done

if [ $robot_index -lt 0 ]; then
  echo "No robot online."
  exit 1
fi

echo "Installing to ${robot_names[$robot_index]}..."

# Remove accidental trailing slashes
self_path=$(dirname "$0")
target=${1%/}
dirname=$(basename ${target})
master_ip=${llp_ips[${robot_index}]}
config_ver=${2:-p4}

if [ -z ${FREEFLYER_MASTER} ]; then
  # See if we have an IP on the Astrobee network
  ip_addr=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep '${subnet}')
  if [ -n "$ip_addr" ]; then
    master_ip=$ip_addr
  fi
fi


FREEFLYER_TARGETS=${FREEFLYER_TARGETS=llp mlp}
FREEFLYER_MASTER=${FREEFLYER_MASTER:=http://${master_ip}:11311/}
FREEFLYER_LLP_FIXES=${FREEFLYER_LLP_FIXES=rpath}
FREEFLYER_INSTALL_DIR=/opt/astrobee

if [[ ${FREEFLYER_TARGETS,,} =~ 'dock' ]]; then
  echo "Copying files to Dock..."
  if ! rsync -azh --delete --info=progress2 $target/ astrobee@${dock_ip}:${FREEFLYER_INSTALL_DIR}
  then
    exit 1
  fi
fi

if [[ ${FREEFLYER_TARGETS,,} =~ 'mlp' ]]; then
  echo "Copying files to MLP..."
  if ! rsync -azh --delete --info=progress2 $target/ astrobee@${mlp_ips[${robot_index}]}:${FREEFLYER_INSTALL_DIR}
  then
    exit 1
  fi
fi

if [[ ${FREEFLYER_TARGETS,,} =~ 'llp' ]]; then
  # Install to LLP
  echo "Copying files to LLP..."
  if ! rsync -azh --delete --info=progress2 $target/ astrobee@${llp_ips[${robot_index}]}:${FREEFLYER_INSTALL_DIR}
  then
    exit 1
  fi

  # fixes=''

  # fixes=${fixes//+([[:space:]])/ }

  # if [ -n "${fixes}" ]; then
  #   ssh -t ubuntu@${subnet}.${llp} "${fixes}"
  # fi
fi

if [ -n "$ip_addr" ]; then
  echo '*** NOTE ***'
  echo 'Be sure to export the following before running anything from this machine: '
  echo "  export ROS_MASTER_URI=${FREEFLYER_MASTER}"
  echo "  export ROS_HOSTNAME=${ip_addr}"
  echo "  export ROS_SSH_UNKNOWN=1"
fi
