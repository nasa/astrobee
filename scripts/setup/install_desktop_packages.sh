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
arssrc=/etc/apt/sources.list.d/astrobee-latest.list
export DIST=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`

pkg_files=${1:-$scriptdir/packages_base_"${DIST}".lst $scriptdir/packages_desktop_${DIST}.lst}
echo "$pkg_files ${DIST}"

pkgs=''
for i in $pkg_files; do
  [ -r "$i" ] || continue

  # this is a non-portable bash-ism.
  pkgs="$pkgs $(<"$i")"
done

# if this file exists, we are using the astrobee repository
# need to set up tunnel through m (NASA users only)
if [ -e $arssrc ];
then
  username=${NDC_USERNAME:+${NDC_USERNAME}@}

  # Add these packages to the apt sources (we remove them later, so apt update succeeds)

  NO_TUNNEL=${NO_TUNNEL:-0} # Override this with 1 before invoking if the tunnel is not desired
 
  if [ "${NO_TUNNEL}" -eq 1 ]; then
      echo "Getting the custom Debian without tunnel"
      sudo /bin/bash -c "echo \"deb [arch=amd64] http://astrobee.ndc.nasa.gov/software ${DIST} main\" > $arssrc" || exit 1
      sudo /bin/bash -c "echo \"deb-src http://astrobee.ndc.nasa.gov/software ${DIST} main\" >> $arssrc" || exit 1
  else
      echo "Tunnelling to get the custom Debian"
      sudo /bin/bash -c "echo \"deb [arch=amd64] http://127.0.0.1:8765/software ${DIST} main\" > $arssrc" || exit 1
      sudo /bin/bash -c "echo \"deb-src http://127.0.0.1:8765/software ${DIST} main\" >> $arssrc" || exit 1
      ssh -N -L 8765:astrobee.ndc.nasa.gov:80 ${username}m.ndc.nasa.gov &
  fi
  
  trap "kill $! 2> /dev/null; sudo truncate -s 0 $arssrc; wait $!" 0 HUP QUIT ILL ABRT FPE SEGV PIPE TERM INT
  sleep 1
fi

sudo apt-get update || exit 1

if ! sudo apt-get install -m -y $pkgs; then
  filter_pkgs="libroyale-dev rti-dev libsoracore-dev libmiro-dev libroyale1 rti libmiro0 libsoracore1"
  for p in $filter_pkgs; do
    pkgs=${pkgs//$p/}
  done
  sudo apt-get install -m -y $pkgs || {
    echo "Couldn't install a necessary package."
    exit 1
  }
fi
