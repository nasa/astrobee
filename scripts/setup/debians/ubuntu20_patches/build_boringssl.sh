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

PACKAGE_NAME=android-platform-external-boringssl
CONTROL_FILES=android-platform-external-boringssl_8.1.0+r23-2build1.debian.tar.xz
ORIG_TAR=android-platform-external-boringssl_8.1.0+r23.orig.tar.gz
URL=http://ports.ubuntu.com/pool/universe/a/android-platform-external-boringssl/
DEB_DIR=boringssl

# Set LANG so perl doesn't complain all the time on chroot
[[ -z $LANG ]] && export LANG=C

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
  rm -f *boringssl*{.deb,.ddeb,.build,.buildinfo,.changes,.dsc,.gz,.xz}
fi
wget $URL/$ORIG_TAR $URL/$CONTROL_FILES &&
mkdir $PACKAGE_NAME && tar -xzvf $ORIG_TAR -C $PACKAGE_NAME &&
tar -xvf $CONTROL_FILES -C $PACKAGE_NAME && rm $CONTROL_FILES &&
cd $PACKAGE_NAME &&
cp -r ../$DEB_DIR/* debian &&
debuild -us -uc &&
cd ..
