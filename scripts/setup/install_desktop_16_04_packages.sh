#!/bin/bash
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

# Install a set of packages to build an Ubuntu 16.04 distro with
# all the development tools required for Astrobee work.
#
# The script uses the package list files for the list of
# packages to install.

scriptdir=$(dirname "$(readlink -f "$0")")

pkg_files=${1:-$scriptdir/packages_base.lst $scriptdir/packages_desktop.lst}
echo $pkg_files

pkgs=''
for i in $pkg_files; do
  [ -r "$i" ] || continue

  # this is a non-portable bash-ism.
  pkgs="$pkgs $(<"$i")"
done

if ! sudo apt-get install -m -y $pkgs; then
  filter_pkgs="libroyale-dev rti-dev libsoracore-dev libmiro-dev libroyale1 rti libmiro0 libsoracore1"
  for p in $filter_pkgs; do
    pkgs=${pkgs//$p/}
  done
  echo "$pkgs"
  sudo apt-get install -m -y $pkgs || {
    echo "Couldn't install a necessary package."
    exit 1
  }
fi
