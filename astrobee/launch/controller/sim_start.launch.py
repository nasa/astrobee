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

        # Update the environment variables relating to absolute paths
        SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",
                               value=os.getenv("ASTROBEE_CONFIG_DIR", "/home/astrobee/native/config"),
                               condition=LaunchConfigurationNotEquals("llp", "local")),
        SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",
                               value=os.getenv("ASTROBEE_RESOURCE_DIR", "home/astrobee/native/resources"),
                               condition=LaunchConfigurationNotEquals("llp", "local")),
        SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE",
                               value=os.getenv("ROSCONSOLE_CONFIG_FILE", "/home/astrobee/native/resources/logging.config"),
                               condition=LaunchConfigurationNotEquals("llp", "local")),

        SetEnvironmentVariable(name="DISPLAY", value=":0",
                               condition=LaunchConfigurationNotEquals("sim", "local")),
        SetEnvironmentVariable(name="ROS_IP", value=LaunchConfiguration("sim"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),


        SetEnvironmentVariable(name='SVGA_VGPU10', value='0', condition=IfCondition(LaunchConfiguration('vmware'))),
        # IncludeLaunchDescription(
        #     get_launch_file('launch/start_simulation.launch.py', 'astrobee_gazebo'),
        #     launch_arguments = {'world':   LaunchConfiguration('world', default="iss"),                   # Execution context
        #                         'gui':     LaunchConfiguration('sviz',     default="false"),      # Robot namespace
        #                         'speed':   LaunchConfiguration('speed', default="1"),   # Output for logging
        #                         'debug':   LaunchConfiguration('debug',  default="false"),      # Debug a node set
        #                         'physics': LaunchConfiguration('physics',    default="ode"), # SIM IP address
        #                         }.items(),
        # ),
    ])
