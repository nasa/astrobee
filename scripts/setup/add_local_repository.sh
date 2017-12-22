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
#
#
# Script to install the prepared debian packages to support the ARS build.
#
# This script works in two steps:
#   1) Rsync debian packages from volar
#   2) Add the local debian packages to the sources
#
# Step 1) will be removed when the debian packages can be hosted on a public
# server not requiring authentification.
#
# The script use a default location for the rsynched packages:
#   ars_repo_root/.astrobee_repo
# This default location can be overridden by setting the following
# environment variable before running the script:
#   ARS_DEB_DIR
# Note: if using ARS_DEB_DIR, make sure that it is consistent between
# successive invocations of the script (unless you know what you are doing)
#
# If your username (when invoking the script) differs from your NDC username
# (typical from a VM), the variable NDC_USERNAME cat be set and will be honored
# Alternatively a correct ssh config can be setup so that the NDC username is
# used for the rsync.
#

scriptdir=$(dirname "$(readlink -f "$0")")
rootdir=`cd $scriptdir/../../..; pwd`

debdir=${ARS_DEB_DIR:-$rootdir/.astrobee_repo}
mkdir -p $debdir
arssrc=/etc/apt/sources.list.d/astrobee-latest.list

username=${NDC_USERNAME:+${NDC_USERNAME}@}

# Get the debian packages from the server
echo "Attempting to ssh to volar... did you set up your .ssh/config?"
rsync -avz ${username}volar.ndc.nasa.gov:/home/p-free-flyer/free-flyer/FSW/ars_debs/ $debdir/ || exit 1

# Add these packages to the apt sources
sudo /bin/bash -c "echo \"deb [arch=amd64] file://$debdir xenial main\" > $arssrc" || exit 1
sudo /bin/bash -c "echo \"deb-src file://$debdir xenial main\" >> $arssrc" || exit 1
sudo apt-key add $scriptdir/../../submodules/platform/rootfs/keys/astrobee.key || exit 1
sudo apt-get update || exit 1
