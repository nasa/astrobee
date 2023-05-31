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
    print("asdf")
    robot_description = Command(['xacro ', get_path('urdf/model.urdf.xacro', 'description'),
                        ' world:="',      LaunchConfiguration('world'),
                        '" top_aft:="',   LaunchConfiguration('top_aft'),
                        '" bot_aft:="',   LaunchConfiguration('bot_aft'),
                        '" bot_front:="', LaunchConfiguration('bot_front'), 
                        '" ns:="_',       LaunchConfiguration('ns'),
                        '" prefix:="',    LaunchConfiguration('ns'), '/"' ])
    # robot_description = Command(['xacro ', get_path('urdf/model_carriage.urdf.xacro')])
    print("asdf")

    return LaunchDescription([
        # Context options (NB: THESE ARE OVERRIDDEN BY ENVIRONMENT VARIABLES)   -->
        # Set world and world correctly; environment variable over rule default -->
        DeclareLaunchArgument("robot", default_value=os.getenv("ASTROBEE_ROBOT", "bsharp")),
        DeclareLaunchArgument("world", default_value=os.getenv("ASTROBEE_WORLD", "granite")),

        DeclareLaunchArgument("ns",     default_value=""),     # Robot namespace
        DeclareLaunchArgument("spurn",  default_value=""),     # Prevent a specific node
        DeclareLaunchArgument("nodes",  default_value=""),     # Launch specific nodes
        DeclareLaunchArgument("extra",  default_value=""),     # Inject an additional node
        DeclareLaunchArgument("debug",  default_value=""),     # Debug node group
        DeclareLaunchArgument("dds",    default_value="true"), # Enable DDS
        
        # General options
        DeclareLaunchArgument("vive",   default_value="false"),
        DeclareLaunchArgument("gviz",   default_value="false"),
        DeclareLaunchArgument("rviz",   default_value="false"),
        DeclareLaunchArgument("sviz",   default_value="false"),
        DeclareLaunchArgument("rqt",    default_value="false"),
        DeclareLaunchArgument("gds",    default_value="false"),
        DeclareLaunchArgument("agent1", default_value="Bsharp"),
        DeclareLaunchArgument("agent2", default_value="Bumble"),
        DeclareLaunchArgument("agent3", default_value="Honey"),
        DeclareLaunchArgument("stats",  default_value="false"),

        # Remaining options
        DeclareLaunchArgument("output",  default_value="log"),    # Output to screen or log
        DeclareLaunchArgument("gtloc",   default_value="false"),  # Use Ground Truth Localizer
        DeclareLaunchArgument("drivers", default_value="true"),   # Should we launch drivers?
        DeclareLaunchArgument("llp",     default_value="10.42.0.37"),  # LLP IP address
        DeclareLaunchArgument("mlp",     default_value="10.42.0.38"),  # MLP IP address
        DeclareLaunchArgument("rec",     default_value=""),
        DeclareLaunchArgument("vmware",  default_value="true"),
        DeclareLaunchArgument("speed",   default_value="1"),
        DeclareLaunchArgument("physics", default_value="ode"),
        DeclareLaunchArgument("sim",     default_value="local"),


        # Simulation options only -->
        DeclareLaunchArgument("pose", default_value="0 0 -0.15 0 0 0"),  # Initial pose (sim only)

        # Path to the bag file
        DeclareLaunchArgument("bag", default_value=""),

        # Make sure all environment variables are set for controller
        # Override the robot and world environment variables all the time. The
        # environment variables are the default if they are set. So in this
        # case we are overriding the environment variables with themselves.
        # Roslaunch arguments override the environment variable which is what
        # this will do.
        SetEnvironmentVariable(
            name='ASTROBEE_ROBOT',
            value=LaunchConfiguration('robot'),
        ),
        SetEnvironmentVariable(
            name='ASTROBEE_WORLD',
            value=LaunchConfiguration('world'),
        ),
        SetEnvironmentVariable(
            name='ASTROBEE_CONFIG_DIR',
            value=os.getenv("ASTROBEE_CONFIG_DIR",get_path("config")),
        ),
        SetEnvironmentVariable(
            name='ASTROBEE_RESOURCE_DIR',
            value=os.getenv("ASTROBEE_RESOURCE_DIR",get_path("resources")),
        ),
        SetEnvironmentVariable(
            name='ROSCONSOLE_CONFIG_FILE',
            value=os.getenv("ROSCONSOLE_CONFIG_FILE",get_path("resources/logging.config")),
        ),

        SetEnvironmentVariable(
            name='ROSCONSOLE_FORMAT',
            value="[${severity}] [${time}] : (${logger}) ${message}"
        ),

        # Start ground controller services
        IncludeLaunchDescription(
            get_launch_file("launch/controller/descriptions.launch.py"),
            launch_arguments={
                "world": LaunchConfiguration("world"),
            }.items(),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/stats.launch.py"),
            condition=IfCondition(LaunchConfiguration("stats")),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/vive.launch.py"),
            condition=IfCondition(LaunchConfiguration("vive")),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/gviz.launch.py"),
            condition=IfCondition(LaunchConfiguration("gviz")),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/gds.launch.py"),
            launch_arguments={
                "world" : LaunchConfiguration("world"),
                "agent1": LaunchConfiguration("agent1"),
                "agent2": LaunchConfiguration("agent2"),
                "agent3": LaunchConfiguration("agent3"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("gds")),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/rqt.launch.py"),
            condition=IfCondition(LaunchConfiguration("rqt")),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/rviz.launch.py"),
            launch_arguments={
                "world" : LaunchConfiguration("world"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        # Launch a recorder for this robot
        IncludeLaunchDescription(
            get_launch_file("launch/controller/bagrecord.launch.py"),
            condition=LaunchConfigurationNotEquals("rec", ""),
            launch_arguments={"bag": LaunchConfiguration("rec")}.items(),
        ),

        # Start the simulator
        IncludeLaunchDescription(
            get_launch_file("launch/controller/sim_start.launch.py"),
            launch_arguments={
                "world"  : LaunchConfiguration("world"),
                "sviz"   : LaunchConfiguration("sviz"),
                "vmware" : LaunchConfiguration("vmware"),
                "speed"  : LaunchConfiguration("speed"),
                "debug"  : LaunchConfiguration("debug"),
                "physics": LaunchConfiguration("physics"),
            }.items(),
        ),

        # # Auto-inert platform #1 at a desired initial location
        # IncludeLaunchDescription(
        #     get_launch_file("launch/spawn.launch.py"),
        #     launch_arguments={
        #         "robot" : LaunchConfiguration("robot"),      # Type of robot
        #         "world" : LaunchConfiguration("world"),      # Execution context
        #         "ns"    : LaunchConfiguration("ns"),         # Robot namespace
        #         "output": LaunchConfiguration("output"),     # Output for logging
        #         "pose"  : LaunchConfiguration("pose"),       # Initial robot pose
        #         "spurn" : LaunchConfiguration("spurn"),      # Prevent node
        #         "nodes" : LaunchConfiguration("nodes"),      # Launch node group
        #         "extra" : LaunchConfiguration("extra"),      # Inject extra nodes
        #         "debug" : LaunchConfiguration("debug"),      # Debug a node set
        #         "sim"   : LaunchConfiguration("sim"),        # SIM IP address
        #         "llp"   : LaunchConfiguration("llp"),        # LLP IP address
        #         "mlp"   : LaunchConfiguration("mlp"),        # MLP IP address
        #         "dds"   : LaunchConfiguration("dds"),        # Enable DDS
        #         "gtloc" : LaunchConfiguration("gtloc"),      # Use Ground Truth Localizer
        #     }.items(),
        # ),

        # # bsharp
        # IncludeLaunchDescription(
        #     get_launch_file("launch/astrobee.launch.py"),
        #     launch_arguments={
        #         "robot"  : LaunchConfiguration("robot"),
        #         "world"  : LaunchConfiguration("world"),
        #         "ns"     : LaunchConfiguration("ns"),
        #         "gviz"   : LaunchConfiguration("gviz"),
        #         "rviz"   : LaunchConfiguration("rviz"),
        #         "sviz"   : LaunchConfiguration("sviz"),
        #         "output" : LaunchConfiguration("output"), 
        #         "drivers": "true",                          # Now start driver nodes
        #         "spurn"  : LaunchConfiguration("spurn"),    # Prevent node
        #         "nodes"  : LaunchConfiguration("nodes"),    # Launch node group
        #         "extra"  : LaunchConfiguration("extra"),    # Inject extra nodes
        #         "llp"    : LaunchConfiguration("llp"),
        #         "mlp"    : LaunchConfiguration("mlp"),
        #         "dds"    : LaunchConfiguration("dds"),
        #     }.items(),
        # ),
    ])
