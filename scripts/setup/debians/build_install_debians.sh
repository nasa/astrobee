#/bin/bash -e
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
# Install the dependencies needed for the debians. Build and install flight
# software debians.

DEBIAN_LOC=`pwd`

sudo apt-get install -y devscripts equivs libproj-dev

cd ${DEBIAN_LOC}/alvar
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_alvar.sh
sudo dpkg -i libalvar2_2.0-3_amd64.deb
sudo dpkg -i libalvar-dev_2.0-3_amd64.deb

cd ${DEBIAN_LOC}/dlib
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_dlib.sh
sudo dpkg -i libdbowdlib1_0.1-2_amd64.deb
sudo dpkg -i libdbowdlib-dev_0.1-2_amd64.deb

cd ${DEBIAN_LOC}/dbow2
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_dbow2.sh
sudo dpkg -i libdbow21_0.1-4_amd64.deb
sudo dpkg -i libdbow2-dev_0.1-4_amd64.deb

cd ${DEBIAN_LOC}/decomputil
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_decomputil.sh
sudo dpkg -i libdecomputil0_0.1-1_amd64.deb
sudo dpkg -i libdecomputil-dev_0.1-1_amd64.deb

cd ${DEBIAN_LOC}/jps3d
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_jps3d.sh
sudo dpkg -i libjps3d0_0.1-1_amd64.deb
sudo dpkg -i libjps3d-dev_0.1-1_amd64.deb

cd ${DEBIAN_LOC}/openmvg
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_openmvg.sh
sudo dpkg -i libopenmvg1_1.1-1_amd64.deb
sudo dpkg -i libopenmvg-dev_1.1-1_amd64.deb
