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

# To run rviz setup a desktop environment, this is using gdm - GNOME Display Manager, 
# to launch the gui desktop uncomment the following line in the Vagrantfile: vb.gui = true

main(){
  installAptPackages

  echoMessage
}

installAptPackages(){
  sudo apt-get -y install gdm mesa-utils xterm

  # install pyqtgraph from .deb on web site, not available in standard repos
  (cd /tmp && wget http://www.pyqtgraph.org/downloads/python-pyqtgraph_0.9.10-1_all.deb)
  (cd /tmp && sudo dpkg -i python-pyqtgraph_0.9.10-1_all.deb)
}

echoMessage(){
  echo "To run rviz setup a desktop environment, this is using gdm - GNOME Display Manager, 
  to launch the gui desktop uncomment the following line in the Vagrantfile: vb.gui = true"
}


main $@
