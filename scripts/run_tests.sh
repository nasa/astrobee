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

# This script is intended to be run from within your docker
# container. It closely mimics test_astrobee.Dockerfile and the
# testing phase of the GitHub CI.

script_folder=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
catkin_ws=$( cd -- "${script_folder}/../.." &> /dev/null && pwd )

set -e
set -x

# The package argument is optional. Default is to test all packages.
package=$1

cd ${catkin_ws}

catkin build --no-status --force-color ${package} --make-args tests
{ catkin build --no-status --force-color ${package} --make-args test -j1 || true; }

set +x
{ source devel/setup.sh || true; }
set -x

catkin run_tests --no-status --force-color ${package}
catkin_test_results build/${package}
