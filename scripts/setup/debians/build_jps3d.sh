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

PACKAGE_NAME=libjps3d
ORIG_TAR=libjps3d_0.1.orig.tar.gz
DEB_DIR=jps3d
DIST=$(grep -oP "(?<=VERSION_CODENAME=).*" /etc/os-release)

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
# originally from sikang... but he deleted his history
git clone --quiet https://github.com/bcoltin/jps3d.git $PACKAGE_NAME 2>&1 || exit 1
cd $PACKAGE_NAME
git checkout --quiet 85e6b22171478684119a119c45dbf6b42653d828 2>&1
git archive --prefix=$PACKAGE_NAME/ --output=../$ORIG_TAR --format tar.gz HEAD || exit 1
cp -r ../$DEB_DIR debian
dch -l"+$DIST" -D"$DIST" "Set distribution '$DIST' for local build"
debuild -us -uc || exit 1
cd ..
