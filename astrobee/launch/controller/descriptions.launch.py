# <!-- Copyright (c) 2017, United States Government, as represented by the     -->
# <!-- Administrator of the National Aeronautics and Space Administration.     -->
# <!--                                                                         -->
# <!-- All rights reserved.                                                    -->
# <!--                                                                         -->
# <!-- The Astrobee platform is licensed under the Apache License, Version 2.0 -->
# <!-- (the "License"); you may not use this file except in compliance with    -->
# <!-- the License. You may obtain a copy of the License at                    -->
# <!--                                                                         -->
# <!--     http://www.apache.org/licenses/LICENSE-2.0                          -->
# <!--                                                                         -->
# <!-- Unless required by applicable law or agreed to in writing, software     -->
# <!-- distributed under the License is distributed on an "AS IS" BASIS,       -->
# <!-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         -->
# <!-- implied. See the License for the specific language governing            -->
# <!-- permissions and limitations under the License.                          -->


import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            # We must publish global transforms in case no robot has been spawned
            # Node(
            # package='framestore',
            # namespace='',
            # executable='global_transforms',
            # name='global_transforms',
            # parameters=[{
            #     # Description is based on the world being simulated / launched
            #     'world': LaunchConfiguration('world'),
            #     '/granite/robot_description':       os.path.join(get_package_share_directory('astrobee_granite'), '/urdf/model.urdf'),
            #     '/iss/robot_description':           os.path.join(get_package_share_directory('astrobee_iss'), '/urdf/model.urdf'),
            #     '/dock/robot_description':          os.path.join(get_package_share_directory('astrobee_dock'), '/urdf/model.urdf')      ,
            #     '/handrail_8_5/robot_description':  os.path.join(get_package_share_directory('handrail_8_5'), '/urdf/model.urdf'),
            #     '/handrail_21_5/robot_description': os.path.join(get_package_share_directory('handrail_21_5'), '/urdf/model.urdf'),
            #     '/handrail_30/robot_description':   os.path.join(get_package_share_directory('handrail_30'), '/urdf/model.urdf'),
            #     '/handrail_41_5/robot_description': os.path.join(get_package_share_directory('handrail_41_5'), '/urdf/model.urdf'),
            #      }]
            # )
        ]
    )
