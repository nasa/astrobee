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

PACKAGE_NAME=libar_track_alvar_msgs
ORIG_TAR=ros-noetic-ar-track-alvar-msgs_0.7.1.orig.tar.gz
DEB_DIR=ar_track_alvar_msgs
DIST=$(grep -oP "(?<=VERSION_CODENAME=).*" /etc/os-release)

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git clone --quiet -b noetic-devel https://github.com/ros-perception/ar_track_alvar.git $PACKAGE_NAME 2>&1 || exit 1
cd $PACKAGE_NAME/$DEB_DIR
git archive --prefix=$PACKAGE_NAME/ --output=../$ORIG_TAR --format tar.gz HEAD || exit 1
cp -r ../../ar-track-alvar-msgs debian
debuild -us -uc || exit 1
cd ../..

mv $PACKAGE_NAME/*ar-track-alvar*{.deb,.debian.tar.xz,.orig.tar.gz,.dsc} .