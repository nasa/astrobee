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

from utilities.utilities import *


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="iss"),     # Robot namespace
        DeclareLaunchArgument("sviz",    default_value="false"),    # Robot pose
        DeclareLaunchArgument("vmware",  default_value="true"),    # Robot description
        DeclareLaunchArgument("speed",   default_value="1"),       # Robot description
        DeclareLaunchArgument("debug",   default_value="false"),   # Robot description
        DeclareLaunchArgument("physics", default_value="ode"),     # Robot description

        SetEnvironmentVariable(name='SVGA_VGPU10', value='0', condition=IfCondition(LaunchConfiguration('vmware'))),

        IncludeLaunchDescription(
            get_launch_file('launch/start_simulation.launch.py', 'astrobee_gazebo'),
            launch_arguments = {'world':   LaunchConfiguration('world'),         # Execution context
                                'gui':     LaunchConfiguration('sviz',),      # Robot namespace
                                'speed':   LaunchConfiguration('speed'),   # Output for logging
                                'debug':   LaunchConfiguration('debug'),      # Debug a node set
                                'physics': LaunchConfiguration('physics'), # SIM IP address
                                }.items(),
        ),
    ])
