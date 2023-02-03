#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIST=$(. /etc/os-release && echo $UBUNTU_CODENAME)

if [ -n "$(git status --porcelain)" ]; then 
  echo "You should not build Debians for a dirty source tree!"
  echo "Make sure all your changes are committed AND pushed to the server..."
  exit -1
fi

EXTRA_FLAGS="-b -aarmhf"

# In some cases we may want to build for amd64 (e.g. astrobee-comms for users)
if [[ $* == *--native* ]]; then
  EXTRA_FLAGS="-b"
fi

if [[ $* == *--config* ]]; then
  EXTRA_FLAGS="-A"
fi

pushd $DIR/../..
DEBEMAIL="astrobee-fsw@nx.arc.nasa.gov" DEBFULLNAME="Astrobee Flight Software" dch -l"+$DIST" -D"$DIST" "Set distribution '$DIST' for local build"
debuild -e ARMHF_CHROOT_DIR -e ARMHF_TOOLCHAIN -e CMAKE_TOOLCHAIN_FILE -e CMAKE_PREFIX_PATH -e ROS_DISTRO -e ROS_PYTHON_VERSION -us -uc $EXTRA_FLAGS
git checkout debian/changelog
popd > /dev/null
