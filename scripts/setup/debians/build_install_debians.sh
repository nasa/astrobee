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

# delete old debians
rm *_amd64.deb

cd ${DEBIAN_LOC}/alvar
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_alvar.sh || exit 1
sudo dpkg -i libalvar*_amd64.deb || exit 1

cd ${DEBIAN_LOC}/dlib
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_dlib.sh || exit 1
sudo dpkg -i libdbowdlib*_amd64.deb || exit 1

cd ${DEBIAN_LOC}/dbow2
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_dbow2.sh || exit 1
sudo dpkg -i libdbow*_amd64.deb || exit 1

cd ${DEBIAN_LOC}/decomputil
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_decomputil.sh || exit 1
sudo dpkg -i libdecomputil*_amd64.deb || exit 1

cd ${DEBIAN_LOC}/openmvg
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_openmvg.sh || exit 1
sudo dpkg -i libopenmvg*_amd64.deb || exit 1

cd ${DEBIAN_LOC}/jps3d
sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control
cd ${DEBIAN_LOC}
./build_jps3d.sh || exit 1
sudo dpkg -i libjps3d*_amd64.deb || exit 1  