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

# CMake 3.16.3 is broken for the armhf setup we use.
# In order to compile our custom debians for the robot we need to patch 
# CMake using this script.
# The patched debians are already in our repositories. You do not need to 
# run this script again unless an additional change to CMake is needed.
# This issue is fixed in CMake 3.19.0.

PACKAGE_NAME=cmake-3.16.3
CONTROL_FILES=cmake_3.16.3-1ubuntu1.debian.tar.xz
ORIG_TAR=cmake_3.16.3.orig.tar.gz
DEB_DIR=cmake

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
wget http://ports.ubuntu.com/pool/main/c/cmake/$ORIG_TAR &&
wget http://ports.ubuntu.com/pool/main/c/cmake/$CONTROL_FILES &&
tar -xzvf $ORIG_TAR &&
tar -xvf $CONTROL_FILES -C $PACKAGE_NAME &&
cd $PACKAGE_NAME &&
cp -r ../$DEB_DIR/* debian &&
debuild -us -uc &&
cd ..
