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

"""
TODO I think this needs to be X Y Z and not pose....
"""

from utilities.utilities import *

# def read_pose(context, *args, **kwargs):
#     pose_arg = LaunchConfiguration("pose")
#     pose = str(pose_arg.perform(context)).split(" ")
#     print("Got pose: ", pose, " for ", LaunchConfiguration("ns").perform(context))
#     return ["-x", pose[0], "-y", pose[1], "-z", pose[2],
#             "-R", pose[3], "-P", pose[4], "-Y", pose[5]]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ns",    default_value=""),     # Robot namespace
        DeclareLaunchArgument("pose"),                        # Robot pose
        DeclareLaunchArgument("robot_description"),           # Robot description

        Node(
            package='astrobee_gazebo',
            executable='spawn_entity.py',
            name='spawn_astrobee',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "bsharp", "-timeout", "30.0",
                        "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"), "-z", LaunchConfiguration("z"),
                        "-R", LaunchConfiguration("R"), "-P", LaunchConfiguration("P"), "-Y", LaunchConfiguration("Y")],
            condition=LaunchConfigurationEquals("ns", "")
        ),
        Node(
            package='astrobee_gazebo',
            executable='spawn_entity.py',
            name='spawn_astrobee',
            output='screen',
            namespace=LaunchConfiguration("ns"),
            arguments=["-topic", "robot_description", "-entity", LaunchConfiguration("ns"), "-timeout", "30.0", "-robot_namespace", LaunchConfiguration("ns"),
                       "-pose", LaunchConfiguration("pose")],
            condition=LaunchConfigurationNotEquals("ns", "")
        )

    ])
