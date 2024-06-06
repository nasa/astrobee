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


def read_pose(context, *args, **kwargs):
    pose_arg = LaunchConfiguration("pose")
    pose = str(pose_arg.perform(context)).split(" ")

    return [
        DeclareLaunchArgument('x', default_value=pose[0]),
        DeclareLaunchArgument('y', default_value=pose[1]),
        DeclareLaunchArgument('z', default_value=pose[2]),
        DeclareLaunchArgument('R', default_value=pose[3]),
        DeclareLaunchArgument('P', default_value=pose[4]),
        DeclareLaunchArgument('Y', default_value=pose[5]),
    ]

def launch_setup(context, *args, **kwargs):
  ns = str( (LaunchConfiguration('ns').perform(context)) )
  
  topic = "/robot_description"
  entity = "bsharp"

  if ns:
    topic = "/" + ns + topic     
    entity = ns
    
  spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_astrobee',
            output='screen',
            arguments=["-topic", topic, "-entity", entity, "-timeout", "30.0",
                        "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"), "-z", LaunchConfiguration("z"),
                        "-R", LaunchConfiguration("R"), "-P", LaunchConfiguration("P"), "-Y", LaunchConfiguration("Y")],
            condition=LaunchConfigurationEquals("ns", "")
        )
  
  return [spawn_entity]

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument("ns",    default_value=""),     # Robot namespace
        DeclareLaunchArgument("pose"),                        # Robot pose
        OpaqueFunction(function=read_pose),
        DeclareLaunchArgument("robot_description"),           # Robot description
        OpaqueFunction(function=launch_setup)
    ])
