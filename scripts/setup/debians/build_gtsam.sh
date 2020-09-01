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

PACKAGE_NAME=libgtsam
ORIG_TAR=libgtsam_4.0.2.orig.tar.gz
DEB_DIR=gtsam

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git clone --quiet https://github.com/borglab/gtsam.git $PACKAGE_NAME --branch 4.0.2 2>&1 || exit 1
cd $PACKAGE_NAME
git archive --prefix=$PACKAGE_NAME/ --output=../$ORIG_TAR --format tar.gz HEAD || exit 1
rm -rf debian # Remove outdated debian included by gtsam
cp -r ../$DEB_DIR debian
debuild -us -uc || exit 1
cd ..
