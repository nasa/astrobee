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

PACKAGE_NAME=decomputil

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git clone --quiet https://github.com/bcoltinnasa/DecompUtil.git $PACKAGE_NAME 2>&1 || exit 1
cd $PACKAGE_NAME
git checkout --quiet 5d652b8d144c8075b272094fb95b90ab9ff0e48e 2>&1
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$1 -DUSE_ROS=FALSE -DBUILD_SHARED_LIBS=TRUE .. || exit 1
make || exit 1
make install || exit 1
cd ../..
