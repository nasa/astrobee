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

PACKAGE_NAME=android-platform-system-core
CONTROL_FILES=android-platform-system-core_8.1.0+r23-5ubuntu2.debian.tar.xz
ORIG_TAR=android-platform-system-core_8.1.0+r23.orig.tar.gz
URL=http://ports.ubuntu.com/pool/universe/a/android-platform-system-core
DEB_DIR=fastboot

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
wget $URL/$ORIG_TAR $URL/$CONTROL_FILES &&
mkdir $PACKAGE_NAME && tar -xzvf $ORIG_TAR -C $PACKAGE_NAME &&
tar -xvf $CONTROL_FILES -C $PACKAGE_NAME &&
cd $PACKAGE_NAME &&
cp -r ../$DEB_DIR/* debian &&
debuild -us -uc &&
cd ..
