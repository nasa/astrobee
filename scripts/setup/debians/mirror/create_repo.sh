#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
APTLY="$DIR/aptly/aptly -config=$DIR/aptly.conf"

parse_packages() {
  local filename="$*"
  var=`fgrep "packages=" $filename | cut -f 2 -d "="`
  # remove leading whitespace characters
  var="${var#"${var%%[![:space:]]*}"}"
  # remove trailing whitespace characters
  var="${var%"${var##*[![:space:]]}"}"   
  packages=`echo -n "$var"`
  packages=${packages//[[:space:]]/ | }
  echo -n $packages
  #echo -n ${packages// / | }
}

# needs latest conf files from repo
packages=$(parse_packages $DIR/astrobee_conf/*.conf)

# add keys to keyring
wget -O - http://archive.ubuntu.com/ubuntu/ubuntu/ubuntu/project/ubuntu-archive-keyring.gpg | gpg --no-default-keyring --keyring trustedkeys.gpg --import
wget -O - http://packages.ros.org/ros.key | gpg --no-default-keyring --keyring trustedkeys.gpg --import

$APTLY mirror create -architectures=armhf xenial-main http://ports.ubuntu.com/ xenial main universe
$APTLY mirror create -architectures=armhf xenial-updates http://ports.ubuntu.com/ xenial-updates main universe
$APTLY mirror create -architectures=armhf xenial-security http://ports.ubuntu.com/ xenial-security main universe
$APTLY mirror create -architectures=armhf ros http://packages.ros.org/ros/ubuntu xenial main

$APTLY repo create astrobee
