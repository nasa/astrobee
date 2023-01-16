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
    """Generate launch description for synthetic."""

    #     <!-- Connect and update environment variables if required -->
    #     <machine unless="$(eval arg('sim')=='local')" name="sim_server" default="true"
    #              address="$(arg sim)" user="astrobee" password="astrobee" timeout="10"/>
    if LaunchConfigurationNotEquals("bag", ""):
        return LaunchDescription([
            DeclareLaunchArgument("robot_description"),     # Robot description
            #   <group if="$(eval arg('bag')=='')" >
            SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",
                                   value="/home/astrobee/native/config",
                                   condition=LaunchConfigurationNotEquals("sim", "local")),
            SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",
                                   value=os.getenv("ASTROBEE_RESOURCE_DIR", get_path("/resources")),
                                   condition=LaunchConfigurationNotEquals("sim", "local")),
            SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE",
                                   value=os.getenv("ROSCONSOLE_CONFIG_FILE", get_path("/resources/logging.config")),
                                   condition=LaunchConfigurationNotEquals("sim", "local")),
            SetEnvironmentVariable(name="DISPLAY", value=":0",
                                   condition=LaunchConfigurationNotEquals("sim", "local")),
            SetEnvironmentVariable(name="ROS_IP", value=LaunchConfiguration("sim"),
                                   condition=LaunchConfigurationNotEquals("sim", "local")),

            IncludeLaunchDescription(
                get_launch_file("launch/spawn_astrobee.launch.py", "astrobee_gazebo"),
                launch_arguments={
                    "ns"  : LaunchConfiguration("ns"),
                    "pose": LaunchConfiguration("pose"),
                    "robot_description": LaunchConfiguration("robot_description"),
                }.items(),
            ),
        ])
    return LaunchDescription([
        IncludeLaunchDescription(
            get_launch_file("/launch/controller/bagreplay.launch"),
            launch_arguments={"bag": LaunchConfiguration("bag")}.items()
        ),
    ])
