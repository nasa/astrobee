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

PACKAGE_NAME=opencv

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git clone https://github.com/opencv/opencv.git --branch 4.2.0 $PACKAGE_NAME/opencv 2>&1 || exit 1
git clone https://github.com/opencv/opencv_contrib.git --branch 4.2.0 $PACKAGE_NAME/contrib 2>&1 || exit 1
cd $PACKAGE_NAME/opencv
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$1 -DCMAKE_BUILD_TYPE=RELEASE \
	-DINSTALL_C_EXAMPLES=ON \
	-DINSTALL_PYTHON_EXAMPLES=ON \
	-DOPENCV_GENERATE_PKGCONFIG=ON \
	-DOPENCV_EXTRA_MODULES_PATH="$(pwd)/../../contrib/modules" \
	-DBUILD_EXAMPLES=ON \
	-DOPENCV_ENABLED_NONFREE=YES \
	-DENABLE_PRECOMPILED_HEADERS=OFF \
  	-DCMAKE_SHARED_LINKER_FLAGS_RELEASE="$(LDFLAGS)" \
	-DBUILD_SHARED_LIBS=ON -DBUILD_DOCS=ON \
	-DWITH_V4L=OFF \
	-DWITH_LIBV4L=OFF \
	-DWITH_CUDA=OFF .. || exit 1
make -j$(nproc)|| exit 1
make install || exit 1
cd ../../..
