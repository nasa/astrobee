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
# Install the dependencies needed for the debians. Build and install flight
# software debians.
set -e

debian_loc=$(dirname "$(readlink -f "$0")")
dist=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`
build_list=()

sudo apt-get install -y devscripts equivs libproj-dev
# delete old files (-f avoids 'no such file' warning on first run)
rm -f *.deb *.debian.tar.xz *.orig.tar.gz *.dsc *.build *.buildinfo *.changes *.ddeb

case $dist in
  xenial)
    echo "Ubuntu 16 detected"
    ;;
  bionic|focal)
    build_list+=( opencv )
    ;;&
  bionic)
    echo "Ubuntu 18 detected"
     # jps3d deps
    sudo apt-get install -y libvtk6.3 libboost-filesystem1.62.0 libboost-system1.62.0
    ;;
  focal)
    echo "Ubuntu 20 detected"
    #jps3d deps
    sudo apt-get install -y libvtk7.1p libboost-filesystem1.71.0 libboost-system1.71.0
    ;;
  *)
    echo "No supported distribution detected"
    exit 1
esac

# Add public debians to build list
build_list+=( alvar dlib dbow2 gtsam decomputil jps3d openmvg )
# If restricted rti-dev debian is present, add miro and soracore as well
dpkg-query -W -f='${Status}\n' rti-dev 2>&1 | grep -q "install ok installed" &&
echo "Package rti-dev exists. Including miro and soracore to build list..." &&
build_list+=( miro soracore )

export DEBEMAIL="nospam@nospam.org"
for pkg in ${build_list[@]}
do
  cd ${debian_loc}/$pkg &&
  sudo mk-build-deps -i -r -t "apt-get --no-install-recommends -y" control &&
  cd ${debian_loc} &&
  ./build_${pkg}.sh &&
  sudo dpkg -i *${pkg}*.deb || exit 1
done
