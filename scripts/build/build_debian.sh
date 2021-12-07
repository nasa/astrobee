#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -n "$(git status --porcelain)" ]; then 
  echo "You should not build Debians for a dirty source tree!"
  echo "Make sure all your changes are committed AND pushed to the server..."
  exit -1
fi

EXTRA_FLAGS="-b -a armhf"
if [[ $* == *--config* ]]; then
  EXTRA_FLAGS="-A"
fi

pushd $DIR/../..
export CMAKE_TOOLCHAIN_FILE=${DIR}/ubuntu_cross.cmake
DEB_BUILD_OPTIONS="parallel=20" debuild -e ARMHF_CHROOT_DIR -e ARMHF_TOOLCHAIN -e CMAKE_TOOLCHAIN_FILE -e CMAKE_PREFIX_PATH -us -uc $EXTRA_FLAGS 
popd > /dev/null
