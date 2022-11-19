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
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():

    return LaunchDescription(
        [
            #   <group if="$(eval arg('bag')=='')" >
            SetEnvironmentVariable(
                name="ASTROBEE_CONFIG_DIR",
                value="/home/astrobee/native/config",
                condition=LaunchConfigurationNotEquals("sim", "local"),
            ),
            SetEnvironmentVariable(
                name="ASTROBEE_RESOURCE_DIR",
                value=os.getenv(
                    "ASTROBEE_RESOURCE_DIR",
                    get_package_share_directory("astrobee", "/resources"),
                ),
            ),
            SetEnvironmentVariable(
                name="ROSCONSOLE_CONFIG_FILE",
                value=os.getenv(
                    "ROSCONSOLE_CONFIG_FILE",
                    get_package_share_directory(
                        "astrobee", "/resources/logging.config"
                    ),
                ),
            ),
            # Declare our global logging format
            SetEnvironmentVariable(
                name="RCUTILS_CONSOLE_OUTPUT_FORMAT",
                value="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("astrobee"),
                            "launch/controller/rqt.spawn_astrobee.py",
                        )
                    ]
                ),
                launch_arguments={
                    "ns": LaunchConfiguration("ns"),
                    "pose": LaunchConfiguration("pose"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("astrobee"),
                            "/launch/controller/bagreplay.launch",
                        )
                    ]
                ),
                condition=LaunchConfigurationNotEquals("bag", ""),
                launch_arguments={"bag": LaunchConfiguration("bag")}.items(),
            ),
        ]
    )


# <launch>
#   <arg name="world" />      <!-- World name                -->
#   <arg name="ns" />         <!-- Robot namespace           -->
#   <arg name="sim" />        <!-- SIM IP address            -->
#   <arg name="pose" />       <!-- Initial pose (sim only)   -->
#   <arg name="bag" />        <!-- Bag to replay             -->

#     <!-- Connect and update environment variables if required -->
#     <machine unless="$(eval arg('sim')=='local')" name="sim_server" default="true"
#              address="$(arg sim)" user="astrobee" password="astrobee" timeout="10"/>

#     <!-- Update the environment variables relating to absolute paths -->
#     <env unless="$(eval arg('sim')=='local')"
#          name="ASTROBEE_CONFIG_DIR" value="/home/astrobee/native/config" />
#     <env unless="$(eval arg('sim')=='local')"
#          name="ASTROBEE_RESOURCE_DIR" value="home/astrobee/native/resources" />
#     <env unless="$(eval arg('sim')=='local')"
#          name="ROSCONSOLE_CONFIG_FILE" value="/home/astrobee/native/resources/logging.config"/>
#     <env unless="$(eval arg('sim')=='local')"
#          name="DISPLAY" value=":0"/>
#     <env unless="$(eval arg('sim')=='local')"
#          name="ROS_IP" value="$(arg sim)"/>


#   </group>

#  </launch>
