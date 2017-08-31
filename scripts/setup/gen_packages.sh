#!/bin/sh -e
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

# Generate the packages.lst file used by the install_desktop_packages.sh
# script. This script needs to know the location of the "platform" directory
# where the rootfs *.conf files are located.

platform_dir=${1:-../../submodules/platform}
platform_dir=$(realpath "$platform_dir")

if [ ! -d "$platform_dir/rootfs" ]; then
  echo "$platform_dir is not a valid platform directory. aborting."
  exit 1
fi

conf_files="$platform_dir/rootfs/ros_16_04.conf $platform_dir/rootfs/dev_16_04.conf"

fgrep "packages=" ${conf_files} | cut -f 2 -d "=" | tr ' ' '\n' >packages_base.lst

