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
    world_file = [get_path("worlds", "astrobee_gazebo"), "/", LaunchConfiguration('world'), ".world"]

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("speed",   default_value="1"),
        DeclareLaunchArgument("debug",   default_value="false"),
        DeclareLaunchArgument("physics",   default_value="ode"),


        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value="/usr/share/gazebo-11"),

#   <param name="/simulation_speed" value="$(arg speed)" />
# TODO(@mgouveia): Not sure what to do about the speed, I think I'll have to pass it to
# the plugin through sdf robot description since I can't set the parameter here

        IncludeLaunchDescription(
            get_launch_file( 'launch/gzserver.launch.py', 'gazebo_ros'),
            launch_arguments = {
                                'world':   world_file,      # Execution context
                                'verbose': LaunchConfiguration('debug'),   # Debug a node set
                                'physics': LaunchConfiguration('physics'), # SIM IP address
                                }.items(),
        ),
        IncludeLaunchDescription(
            get_launch_file( 'launch/gzclient.launch.py', 'gazebo_ros'),
            launch_arguments = {'verbose':   LaunchConfiguration('debug'),      # Debug a node set
                                }.items(),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
    ])
