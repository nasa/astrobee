#!/bin/bash

PACKAGE_NAME=libdeepdive
ORIG_TAR=libdeepdive_0.1.orig.tar.gz

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git -c advice.detachedHead=false clone https://github.com/asymingt/libdeepdive.git $PACKAGE_NAME --branch release-0.1.1 || exit 1
cd $PACKAGE_NAME
git archive --prefix=$PACKAGE_NAME/ --output=../$ORIG_TAR --format tar.gz HEAD || exit 1
debuild -us -uc || exit 1
cd ..
