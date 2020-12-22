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

# Check that the paths are defined
DIR=$(dirname "$(readlink -f "$0")")

echo "Build context for toolchain base: "${ARMHF_TOOLCHAIN}
if [ -z "${ARMHF_TOOLCHAIN}" ]; then echo ARMHF_TOOLCHAIN is not set, please check armhf instructions; exit -1; fi
echo "Build context for armhf base: "${ARMHF_CHROOT_DIR}
if [ -z "${ARMHF_CHROOT_DIR}" ]; then echo ARMHF_CHROOT_DIR is not set, please check armhf instructions; exit -1; fi
echo "Build context for cross: "${DIR}/../../..

if [ -z "${INSTALL_PATH}" ]
then
  if [ -d "${DIR}/../../../../astrobee_install/armhf/" ] # Checks if this folder is defined otherwise defaults to standard
  then export INSTALL_PATH=${DIR}/../../../../astrobee_install/armhf/
  else export INSTALL_PATH=$HOME/astrobee_build/armhf
  fi
fi
echo "Saving the cross-compiled code in: "${INSTALL_PATH}

# Setup the build context
docker build $ARMHF_TOOLCHAIN -f scripts/docker/cross_compile/astrobee_base_toolchain.Dockerfile -t astrobee/astrobee:base-toolchain
docker build $ARMHF_CHROOT_DIR -f scripts/docker/cross_compile/astrobee_base_rootfs.Dockerfile -t astrobee/astrobee:base-cross

# Cross-compiles the code
docker build ${DIR}/../../.. -f scripts/docker/cross_compile/astrobee_cross.Dockerfile -t astrobee/astrobee:cross

# Save the code
docker run --rm --entrypoint tar astrobee/astrobee:cross cC /opt/astrobee . | tar xvC ${INSTALL_PATH}
