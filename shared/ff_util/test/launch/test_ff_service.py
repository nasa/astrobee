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

import os
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from utilities.utilities import *


# This function specifies the processes to be run for the test.
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    test_ff_service = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "test_ff_service",
            ]
        ),
        output="screen",
    )

    return launch.LaunchDescription(
        [
            SetEnvironmentVariable(name="ASTROBEE_ROBOT", value="sim"),
            SetEnvironmentVariable(name="ASTROBEE_WORLD", value="iss"),
            SetEnvironmentVariable(
                name="ASTROBEE_CONFIG_DIR",
                value=os.getenv("ASTROBEE_CONFIG_DIR", get_path("config")),
            ),
            SetEnvironmentVariable(
                name="ASTROBEE_RESOURCE_DIR",
                value=os.getenv("ASTROBEE_RESOURCE_DIR", get_path("resources")),
            ),
            SetEnvironmentVariable(
                name="ROSCONSOLE_CONFIG_FILE",
                value=os.getenv(
                    "ROSCONSOLE_CONFIG_FILE", get_path("resources/logging.config")
                ),
            ),
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package containing test executables",
                default_value="/src/astrobee/build/ff_util"
            ),
            test_ff_service,
            # Tell launch when to start the test
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "test_ff_service": test_ff_service,
    }


class TestTerminatingProcessStops(unittest.TestCase):
    def test_proc_terminates(self, proc_info, test_ff_service):
        proc_info.assertWaitForShutdown(process=test_ff_service, timeout=30)


# These tests are run after the processes in generate_test_description() have shutdown.
@launch_testing.post_shutdown_test()
class TestGTestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
