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

# This image is split into two, because we need to copy over the toolchain
# folder and rootfs folder. Because docker can only access files in the
# build context, and we don't want it to scan the entire computer by
# chosing a bigger build context, this is the easiest solution. This also
# allows the user to place the folders freely.

# Base docker image copies over the toolchain and rootfs, resulting from
# the NASA_INSTALL.md instructions. Also installs minimum dependencies
echo "Build context for base: "${ARMHF_TOOLCHAIN}
docker build $ARMHF_TOOLCHAIN -f scripts/docker/cross_compile/astrobee_base_toolchain.Dockerfile -t astrobee/astrobee:base-toolchain

echo "Build context for base: "${ARMHF_CHROOT_DIR}
docker build $ARMHF_CHROOT_DIR -f scripts/docker/cross_compile/astrobee_base_rootfs.Dockerfile -t astrobee/astrobee:base-cross

# Cross-compiles the code
thisdir=$(dirname "$(readlink -f "$0")")
echo "Build context for cross: "${thisdir}
docker build . -f scripts/docker/cross_compile/astrobee_cross.Dockerfile -t astrobee/astrobee:cross