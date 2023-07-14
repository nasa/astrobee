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

# This package depends on rti which is a proprietary package and therefore
# cannot be shared. It is not necessary for running the astrobee simulation.
# This debian build is only useful for NASA maintenence of this repo.

PACKAGE_NAME=libsoracore
ORIG_TAR=libsoracore_2.0.orig.tar.gz
DEB_DIR=soracore
DIST=$(grep -oP "(?<=VERSION_CODENAME=).*" /etc/os-release)

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git clone https://github.com/marinagmoreira/soraCore.git --branch catkin_build $PACKAGE_NAME 2>&1 || exit 1
cd $PACKAGE_NAME
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$1 -DCMAKE_SHARED_LINKER_FLAGS="-Wl,-allow-multiple-definition" -DMIRO_BUILD_WITH_QT5=true .. || exit 1
make || exit 1
make install || exit 1
cd ../..
