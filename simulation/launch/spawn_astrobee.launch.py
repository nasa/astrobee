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
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # robot_name = 'rrbot_description'
    # world_file_name = 'empty.world'

    # world = os.path.join(get_package_share_directory(
    #     robot_name), 'worlds', world_file_name)

    # urdf = os.path.join(get_package_share_directory(
    #     robot_name), 'urdf', 'my_robot.urdf')

    # xml = open(urdf, 'r').read()

    # xml = xml.replace('"', '\\"')

    # spawn_args = '{name: \"bsharp\", xml: \"' + xml + '\" }'
    return LaunchDescription([
        DeclareLaunchArgument("ns",    default_value=""),     # Robot namespace
        DeclareLaunchArgument("pose"),                        # Robot pose
        DeclareLaunchArgument("robot_description"),           # Robot description

        # Use the namespace forward-slash to signify that this model has no name
        # Node(
        #     package="astrobee_gazebo",
        #     namespace="",
        #     executable="spawn_model",
        #     name="spawn_astrobee",
        #     respawn="false",
        #     output='screen',
        #     arguments=['-param', 'robot_description', '-model', 'bsharp', '-urdf', '-pose', LaunchConfiguration("pose")],
        #     parameters=[{'respawn': "false",
        #                  'robot_description': LaunchConfiguration("robot_description")}],
        #     condition=LaunchConfigurationEquals("ns", "")
        # ),

        # # Spawn with an actual string namespace
        # Node(
        #     package="astrobee_gazebo",
        #     namespace=LaunchConfiguration("ns"),
        #     executable="spawn_astrobee",
        #     name="spawn_astrobee",
        #     respawn="false",
        #     output='screen',
        #     arguments=[['-param', 'robot_description', '-urdf -model ', LaunchConfiguration("ns"), ' -pose ', LaunchConfiguration("pose")]],
        #     parameters=[{'respawn': "false",
        #                  'robot_description': LaunchConfiguration("robot_description")}],
        #     condition=LaunchConfigurationNotEquals("ns", "")
        # ),
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', world,
        #          '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'),

        # ExecuteProcess(
        #     cmd=['ros2', 'param', 'set', '/gazebo',
        #          'use_sim_time', use_sim_time],
        #     output='screen'),

        # ExecuteProcess(
        #     cmd=['ros2', 'service', 'call', '/spawn_entity',
        #          'gazebo_msgs/SpawnEntity', spawn_args],
        #     output='screen'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_astrobee',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "bsharp", "-timeout", "30.0"],
            condition=LaunchConfigurationEquals("ns", "")
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_astrobee',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", LaunchConfiguration("ns"), "-timeout", "30.0"],
            condition=LaunchConfigurationNotEquals("ns", "")
        )

    ])
