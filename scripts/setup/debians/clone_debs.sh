#!/bin/bash

# This script will clone all needed libs. Then, the user needs to add OpenCV linking to the correct folders. It is advised to install OpenCV+Contrib to ~/.local

git clone https://github.com/astanin/mirror-alvar.git libalvar || exit 1

git clone https://github.com/dorian3d/DBoW2.git libdbow2 --branch v1.1-free || exit 1

git clone https://github.com/bcoltinnasa/DecompUtil.git libdecomputil || exit 1
cd libdecomputil
git checkout 5d652b8d144c8075b272094fb95b90ab9ff0e48e
cd ..

git -c advice.detachedHead=false clone https://github.com/asymingt/libdeepdive.git libdeepdive --branch release-0.1.1

git clone https://github.com/dorian3d/DLib.git libdbowdlib --branch v1.1-free || exit 1

git clone https://github.com/openMVG/openMVG.git libopenmvg --branch v1.1 || exit 1

git clone https://github.com/Pedro-Roque/jps3d.git libjps3d


