#/bin/bash -e
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
dist=$(. /etc/os-release && echo $UBUNTU_CODENAME)
build_list=()
install_debians=false
install_deps=false

die () {
  printf "${c_red}${c_bold}[ FATAL ]${c_reset} ${c_bold}%b${c_reset}\\n" "$1" >&2
  exit "${2:-1}"
}

# Set LANG so perl doesn't complain all the time on chroot
[[ -z $LANG ]] && export LANG=C

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
      # Packages to find and build dependencies
      sudo apt-get install -y devscripts equivs libproj-dev || die
      # Dependencies for jps3d
      if [[ $dist == "bionic" ]]; then
        sudo apt-get install -y libvtk6.3 libboost-filesystem1.62.0 libboost-system1.62.0 || die
      fi
      if [[ $dist == "focal" ]]; then
        sudo apt-get install -y libvtk7.1p libboost-filesystem1.71.0 libboost-system1.71.0 || die
      fi
      ;;
  esac
  shift
done

# Build opencv if ubuntu 18 or 20
[[ "$dist" =~ ^bionic|focal$ ]] && build_list+=( opencv )

# Add public debians to build list
build_list+=( alvar dlib dbow2 gtsam decomputil jps3d openmvg )
# If restricted rti-dev debian is present, add miro and soracore as well
dpkg-query -W -f='${Status}\n' rti-dev 2>&1 | grep -q "install ok installed" &&
echo "Package rti-dev exists. Including miro and soracore to build list..." &&
build_list+=( miro soracore )

echo "Building debians, this may take a while..."

export DEBEMAIL="nospam@nospam.org"
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