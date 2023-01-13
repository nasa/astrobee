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

import ament_index_python
import launch
import launch_testing
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from utilities.utilities import *


def generate_test_description():
    test_ff_timer = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration('test_binary_dir'),
                'test_ff_timer',
            ]
        ),
        output='screen',
    )

    return launch.LaunchDescription([
        SetEnvironmentVariable(name='ASTROBEE_CONFIG_DIR',    value=os.getenv('ASTROBEE_CONFIG_DIR',    get_path('config'))),
        SetEnvironmentVariable(name='ASTROBEE_RESOURCE_DIR',  value=os.getenv('ASTROBEE_RESOURCE_DIR',  get_path('resources'))),
        SetEnvironmentVariable(name='ROSCONSOLE_CONFIG_FILE', value=os.getenv('ROSCONSOLE_CONFIG_FILE', get_path('resources/logging.config'))),
        launch.actions.DeclareLaunchArgument(
            name='test_binary_dir',
            description='Binary directory of package containing test executables',
        ),
        test_ff_timer,
        # TimerAction(period=2.0, actions=[test_ff_timer]),
        launch_testing.actions.ReadyToTest(),
    ]), {
        'test_ff_timer': test_ff_timer,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, test_ff_timer):
        self.proc_info.assertWaitForShutdown(test_ff_timer, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes
    def test_gtest_pass(self, proc_info, test_ff_timer):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_ff_timer
        )
