#!/bin/bash

PACKAGE_NAME=libdeepdive
ORIG_TAR=libdeepdive_0.1.orig.tar.gz

cd $PACKAGE_NAME
git archive --prefix=$PACKAGE_NAME/ --output=../$ORIG_TAR --format tar.gz HEAD || exit 1
debuild -us -uc || exit 1
cd ..
