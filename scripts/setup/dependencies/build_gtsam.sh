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

PACKAGE_NAME=gtsam

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git clone --quiet https://github.com/borglab/gtsam.git $PACKAGE_NAME --branch 4.1rc 2>&1 || exit 1
cd $PACKAGE_NAME
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$1 \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_WITH_TBB=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_WRAP=OFF \
      -DGTSAM_BUILD_DOCS=OFF \
      -DGTSAM_INSTALL_CPPUNITLITE=OFF \
      -DGTSAM_BUILD_UNSTABLE=OFF \
      -DCMAKE_BUILD_TYPE=Release .. || exit 1
make || exit 1
make install || exit 1
cd ../..
