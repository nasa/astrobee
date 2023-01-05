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
export LANG=C

debian_loc=$(dirname "$(readlink -f "$0")")
dist=$(. /etc/os-release && echo $UBUNTU_CODENAME)
build_list=()

# delete old files (-f avoids 'no such file' warning on first run)
rm -f *.deb *.debian.tar.xz *.orig.tar.gz *.dsc *.build *.buildinfo *.changes *.ddeb

# Build opencv if ubuntu 18 or 20
[[ "$dist" =~ ^bionic|focal$ ]] && build_list+=( opencv )

# Add public debians to build list
build_list+=( alvar dlib dbow2 gtsam decomputil jps3d openmvg )
# If restricted rti-dev debian is present, add miro and soracore as well
dpkg-query -W -f='${Status}\n' rti-dev 2>&1 | grep -q "install ok installed" &&
echo "Package rti-dev exists. Including miro and soracore to build list..." &&
build_list+=( miro soracore )

export DEBEMAIL="nospam@nospam.org"
for pkg in ${build_list[@]}
do
  cd ${debian_loc} &&
  ./build_${pkg}.sh &&
  sudo dpkg -i *${pkg}*.deb || exit 1
done
