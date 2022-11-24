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
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="rqt_gui",
                namespace="",
                executable="rqt_gui",
                name="rqt_gui",
                arguments=[
                    "--perspective-file",
                    os.path.join(
                        get_package_share_directory("astrobee"),
                        "resources/gui.perspective",
                    ),
                ],
                respawn=True,
            )
        ]
    )


# <launch>
#   <node pkg="diagnostic_aggregator" type="aggregator_node" name="diagnostic_aggregator" >
#     <rosparam command="load" file="$(find astrobee)/resources/diagnostics.yaml" />
#   </node>
# </launch>
