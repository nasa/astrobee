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

CURRENT_LOC=$(dirname "$(readlink -f "$0")")
echo ${CURRENT_LOC}
# cd /usr/local/lib
PACKAGE_NAME=LuaJIT-2.0.5

if [ -d $PACKAGE_NAME ]; then
  sudo rm -rf $PACKAGE_NAME
fi
wget https://luajit.org/download/LuaJIT-2.0.5.tar.gz
tar xvzf LuaJIT-2.0.5.tar.gz
cd $PACKAGE_NAME
make && sudo make install
# cd ${CURRENT_LOC}
