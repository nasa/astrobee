#!/bin/bash

if [ $# -ne 1 ]
  then
    echo "Usage: $0 VERSION"
    exit 0
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd $DIR/../../..

dch -c debian/changelog -v $1

sed -i -e "s/set(ASTROBEE_VERSION .*)/set(ASTROBEE_VERSION $1)/g" CMakeLists.txt

sed -i -e "s/\# Astrobee Robot Software v1/\# Astrobee Robot Software v1\n\n\#\# Release $1\n\nINSERT DESCRIPTION HERE/g" RELEASE.md

sed -i -e "s/^PROJECT_NUMBER.*/PROJECT_NUMBER         = $1/g" freeflyer.doxyfile

$EDITOR RELEASE.md
