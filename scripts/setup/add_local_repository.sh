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

#
# Script to setup the source repository for the astrobee custom debians
#

scriptdir=$(dirname "$(readlink -f "$0")")
rootdir=`cd $scriptdir/../../..; pwd`

arssrc=/etc/apt/sources.list.d/astrobee-latest.list

# Add these packages to the apt sources
sudo touch $arssrc
# The correct source for the repo is now configure on the fly in install_desktop_16.04_package.sh
#sudo /bin/bash -c "echo \"deb [arch=amd64] http://127.0.0.1:8765/software xenial main\" > $arssrc" || exit 1
#sudo /bin/bash -c "echo \"deb-src http://127.0.0.1:8765/software xenial main\" >> $arssrc" || exit 1

sudo apt-key add $scriptdir/../../submodules/platform/rootfs/multistrap/keys/astrobee.key || exit 1
