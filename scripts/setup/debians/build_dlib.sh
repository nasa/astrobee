#!/bin/bash

PACKAGE_NAME=libdbowdlib
ORIG_TAR=libdbowdlib_0.1.orig.tar.gz
DEB_DIR=dlib

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git clone --quiet https://github.com/dorian3d/DLib.git $PACKAGE_NAME --branch v1.1-free 2>&1 || exit 1
cd $PACKAGE_NAME
git archive --prefix=$PACKAGE_NAME/ --output=../$ORIG_TAR --format tar.gz HEAD || exit 1
cp -r ../$DEB_DIR debian
debuild -us -uc || exit 1
cd ..
