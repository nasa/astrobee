#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -n "$(git status --porcelain)" ]; then 
  echo "You should not build Debians for a dirty source tree!"
  echo "Make sure all your changes are committed AND pushed to the server..."
  exit -1
fi

pushd $DIR/../..
debuild -e ARMHF_CHROOT_DIR -e ARMHF_TOOLCHAIN -us -uc -b -a armhf -j`nproc`
popd > /dev/null
