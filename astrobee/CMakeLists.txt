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

project(astrobee)

catkin_package(
    CATKIN_DEPENDS roscpp
)

install(DIRECTORY config/ DESTINATION config)
# resources will be installed separately
# install(DIRECTORY resources/ DESTINATION resources)

install_launch_files()

install(PROGRAMS scripts/check_env.sh
                 scripts/print_version.sh
                 scripts/cpu_print_version.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
