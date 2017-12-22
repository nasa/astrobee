#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

pushd $DIR/../..
debuild -e ARMHF_CHROOT_DIR -e ARMHF_TOOLCHAIN -us -uc -b -a armhf -j`nproc`
popd > /dev/null
