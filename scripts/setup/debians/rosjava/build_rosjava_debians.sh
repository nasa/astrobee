#!/bin/bash -e
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Build and install flight software custom dependencies.
set -e

debian_loc=$(dirname "$(readlink -f "$0")")
build_list=()
install_debians=false
install_deps=false

die () {
  printf "${c_red}${c_bold}[ FATAL ]${c_reset} ${c_bold}%b${c_reset}\\n" "$1" >&2
  exit "${2:-1}"
}

cleanup() {
  rm -rf ros-noetic-rosjava-build-tools ros-noetic-rosjava-bootstrap \
    ros-noetic-genjava ros-noetic-rosjava-messages
}

# delete old files (-f avoids 'no such file' warning on first run)
rm -f *.deb *.debian.tar.xz *.orig.tar.gz *.dsc *.build *.buildinfo *.changes *.ddeb

# Process arguments and take action
while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--install)
      install_debians=true
      ;;
    -d|--install-with-deps)
      install_debians=true
      install_deps=true
      ;;
  esac
  shift
done

# Add public debians to build list
build_list+=( build-tools bootstrap genjava messages )

echo "Building debians"
trap 'cleanup' EXIT

for pkg in ${build_list[@]}
do
  # Dependencies
  if $install_deps ; then
    cd ${debian_loc}/$pkg || die
    sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control ||
      die "Failed to install dependencies for $pkg"
  fi

  # Building
  cd "$debian_loc"
  ./build_${pkg}.sh || die "Failed to build $pkg"

  # Installing
  if $install_debians ; then
    sudo dpkg -i *${pkg}*.deb || die "Failed to install $pkg"
  fi
done
