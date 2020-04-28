#!/bin/bash

PACKAGE_NAME=libdbowdlib
ORIG_TAR=libdbowdlib_0.1.orig.tar.gz
DEB_DIR=dlib

cd $PACKAGE_NAME
git archive --prefix=$PACKAGE_NAME/ --output=../$ORIG_TAR --format tar.gz HEAD || exit 1
cp -r ../$DEB_DIR debian
debuild -us -uc || exit 1
cd ..
