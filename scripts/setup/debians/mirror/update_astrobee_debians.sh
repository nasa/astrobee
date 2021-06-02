#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
APTLY="$DIR/aptly/aptly -config=$DIR/aptly.conf"

DEBLOC=${ASTROBEE_DEBIAN_DIR:-/home/p-free-flyer/free-flyer/FSW/ars_debs/dists/xenial/main}
BIONIC_DEBLOC=${ASTROBEE_BIONIC_DEBIAN_DIR:-/home/p-free-flyer/free-flyer/FSW/ars_debs/dists/bionic}

# add our debians
$APTLY repo add astrobee $DEBLOC/binary-armhf/*.deb
$APTLY repo add astrobee $DEBLOC/binary-amd64/*.deb
$APTLY repo add astrobee $DEBLOC/source/*.dsc
$APTLY repo add bionic-astrobee $BIONIC_DEBLOC/*.deb
$APTLY repo add bionic-astrobee $BIONIC_DEBLOC/*.dsc
