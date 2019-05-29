#!/bin/bash

mkdir astrobee-repo-version
mkdir -p astrobee-repo-version/DEBIAN
mkdir -p astrobee-repo-version/etc/
echo $1 > astrobee-repo-version/etc/astrobee-release
VERSION=`date +%Y%m%d%H%m`
read -r -d '' RELEASE << EOM
SNAPSHOT="$1"
TIMESTAMP="$VERSION"
TIMESTAMP_UNIX="`date +%s`"
EOM
echo "$RELEASE" > astrobee-repo-version/etc/astrobee-release
read -r -d '' CONTROL << EOM
Package: astrobee-repo-version
Version: $VERSION
Maintainer: Brian Coltin <nospam@nospam.org>
Architecture: all
Description: Astrobee debian repo version. Updates with each snapshot.
EOM
echo "$CONTROL" > astrobee-repo-version/DEBIAN/control
cd astrobee-repo-version
tar czf ../data.tar.gz etc
cd DEBIAN
tar czf ../../control.tar.gz *
cd ../..
echo 2.0 > debian-binary
ar r astrobee-repo-version-$VERSION.deb debian-binary control.tar.gz data.tar.gz
rm -r astrobee-repo-version debian-binary control.tar.gz data.tar.gz
