#!/bin/bash

PACKAGE_NAME=dbowdlib


if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
git clone https://github.com/ana-GT/DLib.git $PACKAGE_NAME --branch v1.1-free-opencv4 2>&1 || exit 1
cd $PACKAGE_NAME
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$1 .. || exit 1
make -j$(nproc) || exit 1
make install || exit 1
cd ../..