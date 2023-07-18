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
# Install the dependencies needed for the debians. Build and install flight
# software debians.
set -e
default_folder="$HOME/.local"

# Get the location of the debian build scripts
if [ $# -eq 0 ]
  then
    dest_folder=$default_folder
    echo "Installing dependencies to $dest_folder"
  else
    dest_folder=$1
    echo "Installing dependencies to $dest_folder"
fi

# Check if ~/.local exists
if [ -d "$dest_folder" ] 
then
    echo "$dest_folder exists."
else
    mkdir $dest_folder
fi

debian_loc=$(dirname "$(readlink -f "$0")")
dist=$(. /etc/os-release && echo $UBUNTU_CODENAME)
build_list=()

sudo apt-get install -y devscripts equivs libproj-dev
# delete old files (-f avoids 'no such file' warning on first run)
rm -f *.deb *.debian.tar.xz *.orig.tar.gz *.dsc *.build *.buildinfo *.changes *.ddeb

case $dist in
  bionic|focal)
    build_list+=( opencv )
    ;;&
  jammy)
    # build_list+=( opencv )
    echo "Ubuntu 22 detected"
    sudo apt-get install -y libvtk7.1p
    ;;
  focal)
    echo "Ubuntu 20 detected"
    #jps3d deps
    sudo apt-get install -y libvtk7.1p libboost-filesystem1.71.0 libboost-system1.71.0
    ;;
  *)
    echo "No supported distribution detected. Source installation possible only for Ubuntu 20 and 22."
    exit 1
esac

# Update PATH
update_shrc=0
if [[ ":$PATH:" == *":$HOME/.local/bin:"* ]]; then
  echo "PATH is set."
else
  echo "Setting PATH in $HOME/.zshrc"
  if [[ "$SHELL" == *"zsh"* ]]; then
    sed -i '3 i\export PATH=$PATH:$HOME/.local/bin' $HOME/.zshrc
  else
    sed -i '4 i\export PATH=$PATH:$HOME/.local/bin' $HOME/.bashrc
  fi
  update_shrc=1
fi

# Update LD Library Path
if [[ ":$LD_LIBRARY_PATH:" == *":$HOME/.local/lib:"* ]]; then
  echo "LD_LIBRARY_PATH is set."
else
  echo "Setting LD_LIBRARY_PATH."
  if [[ "$SHELL" == *"zsh"* ]]; then
    sed -i '4 i\export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/lib' $HOME/.zshrc
  else
    sed -i '5 i\export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/lib' $HOME/.bashrc
  fi
  update_shrc=1
fi


if [[ ":$CPATH:" == *":$HOME/.local:"* ]]; then
  echo "CPATH is set."
else
  echo "Setting CPATH."
  if [[ "$SHELL" == *"zsh"* ]]; then
    sed -i '5 i\export CPATH=$CPATH:$HOME/.local' $HOME/.zshrc
    exit 0
  else
    sed -i '6 i\export CPATH=$CPATH:$HOME/.local' $HOME/.bashrc
    exit 0
  fi
  update_shrc=1
fi

if [ $update_shrc -eq 1 ]; then
  if [[ "$SHELL" == *"zsh"* ]]; then
    echo "source $HOME/.zshrc and run this script again."
  else
    echo "source $HOME/.bashrc and run this script again."
  fi  
  exit 0
fi

# Add public debians to build list
build_list+=( dlib dbow2 gtsam decomputil jps3d openmvg )
# If restricted rti-dev debian is present, add miro and soracore as well
dpkg-query -W -f='${Status}\n' rti-dev 2>&1 | grep -q "install ok installed" &&
echo "Package rti-dev exists. Including miro and soracore to build list..." &&
build_list+=( miro soracore )
echo "Building the following packages: ${build_list[@]} in $debian_loc"

for pkg in ${build_list[@]}
do
  echo "Installing $pkg"
  cd ${debian_loc} &&
  ./build_${pkg}.sh $dest_folder
done
