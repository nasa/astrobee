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

import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            # SetEnvironmentVariable(name='SVGA_VGPU10', value='0', condition=IfCondition(LaunchConfiguration('vmware'))),
            # IncludeLaunchDescription(
            #   PythonLaunchDescriptionSource([os.path.join(
            #      get_package_share_directory('astrobee_gazebo'), 'launch/start_simulation.launch.py')]),
            #   launch_arguments = {'world':   LaunchConfiguration('world', default="iss"),                   # Execution context
            #                       'gui':     LaunchConfiguration('sviz',     default="false"),      # Robot namespace
            #                       'speed':   LaunchConfiguration('speed', default="1"),   # Output for logging
            #                       'debug':   LaunchConfiguration('debug',  default="false"),      # Debug a node set
            #                       'physics': LaunchConfiguration('physics',    default="ode"), # SIM IP address
            #                       }.items(),
            # ),
        ]
    )
