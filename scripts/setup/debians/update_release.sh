#!/bin/bash

if [ $# -ne 1 ]
  then
    echo "Usage: $0 VERSION"
    exit 0
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

MINOR=`awk -F . '{print $2}' <<< $1`
MAJOR=`awk -F . '{print $1}' <<< $1`

cd $DIR/../../..

dch -c debian/changelog -v $1

sed -i -e "s/set(ASTROBEE_VERSION_MAJOR .*)/set(ASTROBEE_VERSION_MAJOR $MAJOR)/g" -e "s/set(ASTROBEE_VERSION_MINOR .*)/set(ASTROBEE_VERSION_MINOR $MINOR)/g" CMakeLists.txt

sed -i -e "s/\# Astrobee Robot Software v1/\# Astrobee Robot Software v1\n\n\#\# Release $1\n\nINSERT DESCRIPTION HERE/g" RELEASE.md

$EDITOR RELEASE.md
