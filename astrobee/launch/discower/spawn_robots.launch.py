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
    robot_description = Command(['xacro ', get_path('urdf/model.urdf.xacro', 'discower_description'),
                                ' world:="',      LaunchConfiguration('world'),
                                '" top_aft:="',   LaunchConfiguration('top_aft'),
                                '" bot_aft:="',   LaunchConfiguration('bot_aft'),
                                '" bot_front:="', LaunchConfiguration('bot_front'),
                                '" ns:="_',       LaunchConfiguration('ns'),
                                '" prefix:="',    LaunchConfiguration('ns'), '/"' ])
    return LaunchDescription([
        # Make sure all environment variables are set for controller
        # Override the robot and world environment variables all the time. The
        # environment variables are the default if they are set. So in this
        # case we are overriding the environment variables with themselves.
        # Ros launch arguments override the environment variable which is what
        # this will do.
        SetEnvironmentVariable(name="ASTROBEE_ROBOT", value=os.getenv("ASTROBEE_ROBOT", LaunchConfiguration("robot"))),
        SetEnvironmentVariable(name="ASTROBEE_WORLD", value=os.getenv("ASTROBEE_WORLD", LaunchConfiguration("world"))),
        SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",    value=os.getenv("ASTROBEE_CONFIG_DIR",    get_path("config"))),
        SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",  value=os.getenv("ASTROBEE_RESOURCE_DIR",  get_path("resources"))),
        SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE", value=os.getenv("ROSCONSOLE_CONFIG_FILE", get_path("resources/logging.config"))),
        # Declare our global logging format
        SetEnvironmentVariable(name="RCUTILS_CONSOLE_OUTPUT_FORMAT",
            value="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"),

        # Context options (NB: THESE ARE OVERRIDDEN BY ENVIRONMENT VARIABLES)   -->
        # Set world and world correctly; environment variable over rule default -->
        DeclareLaunchArgument("robot", default_value=os.getenv("ASTROBEE_ROBOT", "p4d")),
        DeclareLaunchArgument("world", default_value=os.getenv("ASTROBEE_WORLD", "granite")),

        DeclareLaunchArgument("ns",    default_value=""),     # Robot namespace
        DeclareLaunchArgument("spurn", default_value=""),     # Prevent a specific node
        DeclareLaunchArgument("nodes", default_value=""),     # Launch specific nodes
        DeclareLaunchArgument("extra", default_value=""),     # Inject an additional node
        DeclareLaunchArgument("debug", default_value=""),     # Debug node group
        DeclareLaunchArgument("dds",   default_value="true"), # Enable DDS
        
        # Remaining options
        DeclareLaunchArgument("output",  default_value="log"),    # Output to screen or log
        DeclareLaunchArgument("gtloc",   default_value="false"),  # Use Ground Truth Localizer
        DeclareLaunchArgument("drivers", default_value="true"),   # Should we launch drivers?
        DeclareLaunchArgument("sim",     default_value="local"),  # SIM IP address
        DeclareLaunchArgument("llp",     default_value="local"),  # LLP IP address
        DeclareLaunchArgument("mlp",     default_value="local"),  # MLP IP address

        # Payload options
        DeclareLaunchArgument("top_aft",   default_value="perching_arm"), # Payload bays
        DeclareLaunchArgument("bot_aft",   default_value="empty"),        # Payload bays
        DeclareLaunchArgument("bot_front", default_value="empty"),        # Payload bays

        # Simulation options only -->
        DeclareLaunchArgument("pose",      default_value="0 0 0 0 0 0"),  # Initial pose (sim only)

        # Path to the bag file
        DeclareLaunchArgument("bag", default_value=""),

        # Set the TF prefix, create a robot description and joint state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration('ns'),
            output="screen",
            parameters=[{'robot_description': ParameterValue(robot_description) }],
        ),

        # If we need to load synthetic drivers (we are not running on a real robot)
        IncludeLaunchDescription(
            get_launch_file("launch/spawn_discower.launch.py", "astrobee_gazebo"),
            launch_arguments={
                "ns"  : LaunchConfiguration("ns"),
                "pose": LaunchConfiguration("pose")
            }.items(),
        ),
        GroupAction(
            actions=[PushRosNamespace(LaunchConfiguration('ns')),
                # LLP
                IncludeLaunchDescription(
                    get_launch_file("launch/robot/LLP.launch.py"),
                    launch_arguments={
                        "drivers": LaunchConfiguration("drivers"),  # Don't start driver nodes
                        "spurn"  : LaunchConfiguration("spurn"),    # Prevent node
                        "nodes"  : LaunchConfiguration("nodes"),    # Launch node group
                        "extra"  : LaunchConfiguration("extra"),    # Inject extra nodes
                        "debug"  : LaunchConfiguration("debug"),    # Debug a node set
                        "output" : LaunchConfiguration("output"), 
                        "gtloc"  : LaunchConfiguration("gtloc"),    # Use Ground Truth Localizer
                    }.items(),
                    condition=LaunchConfigurationNotEquals("llp", "disabled")
                ),

                # MLP
                IncludeLaunchDescription(
                    get_launch_file("launch/robot/MLP.launch.py"),
                    launch_arguments={
                        "drivers": LaunchConfiguration("drivers"), # Don't start driver nodes
                        "spurn"  : LaunchConfiguration("spurn"),   # Prevent node
                        "nodes"  : LaunchConfiguration("nodes"),   # Launch node group
                        "extra"  : LaunchConfiguration("extra"),   # Inject extra nodes
                        "debug"  : LaunchConfiguration("debug"),   # Debug a node set
                        "output" : LaunchConfiguration("output"), 
                        "gtloc"  : LaunchConfiguration("gtloc"),   # Use Ground Truth Localizer
                    }.items(),
                    condition=LaunchConfigurationNotEquals("mlp", "disabled")
                ),
            ]),
        ],
    )
