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
    print("Getting astrobee...")
    return LaunchDescription([
        launch_arg("robot",  description="Robot name"),
        launch_arg("world",  description="World name"),
        launch_arg("ns",     description="Robot namespace prefix"),
        launch_arg("output", description="Where nodes should log"),
        launch_arg("spurn",  description="Prevent a specific node"),
        launch_arg("nodes",  description="Launch specific nodes"),
        launch_arg("extra",  description="Inject additional node"),
        launch_arg("debug",  description="Debug node group"),
        launch_arg("agent1", description="GDS Agent1"),
        launch_arg("agent2", description="GDS Agent2"),
        launch_arg("agent3", description="GDS Agent3"),
        launch_arg("sim",    description="SIM IP address"),
        launch_arg("llp",    description="LLP IP address"),
        launch_arg("mlp",    description="MLP IP address"),
        launch_arg("dds",    description="Enable DDS"),
        launch_arg("gtloc",  description="Use Ground Truth Localizer"),

        # Robot args
        launch_arg("default_robot", description="Insert default robot"),
        launch_arg("pose", default_value="9.816 -9.806 4.293 0 0 0"),
        launch_arg("honey", description="Insert honey robot"),
        launch_arg("bumble", description="Insert bumble robot"),
        launch_arg("queen", description="Insert queen robot"),
        launch_arg("honey_pose", description="Overwrite honey's pose"),
        launch_arg("bumble_pose", description="Overwrite bumble's pose"),
        launch_arg("queen_pose", description="Overwrite bumble's pose"),
        # Launch files
        IncludeLaunchDescription(
            get_launch_file("launch/controller/descriptions.launch.py"),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/gds.launch.py"),
            condition=IfCondition(LaunchConfiguration("gds")),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "agent1": LaunchConfiguration("agent1"),
                "agent2": LaunchConfiguration("agent2"),
                "agent3": LaunchConfiguration("agent3"),
            }.items(),
        ),
        # Auto-inert platform #1 at a desired initial location
        IncludeLaunchDescription(
            get_launch_file("launch/spawn.launch.py"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),      # Type of robot
                "world" : LaunchConfiguration("world"),      # Execution context
                "ns"    : LaunchConfiguration("ns"),         # Robot namespace
                "output": LaunchConfiguration("output"),     # Output for logging
                "pose"  : LaunchConfiguration("pose"),       # Initial robot pose
                "spurn" : LaunchConfiguration("spurn"),      # Prevent node
                "nodes" : LaunchConfiguration("nodes"),      # Launch node group
                "extra" : LaunchConfiguration("extra"),      # Inject extra nodes
                "debug" : LaunchConfiguration("debug"),      # Debug a node set
                "sim"   : LaunchConfiguration("sim"),        # SIM IP address
                "llp"   : LaunchConfiguration("llp"),        # LLP IP address
                "mlp"   : LaunchConfiguration("mlp"),        # MLP IP address
                "dds"   : LaunchConfiguration("dds"),        # Enable DDS
                "gtloc" : LaunchConfiguration("gtloc"),      # Use Ground Truth Localizer
            }.items(),
            condition=IfCondition(LaunchConfiguration("default_robot")),
        ),
        # Auto-insert honey at a canned location
        IncludeLaunchDescription(
            get_launch_file("launch/spawn.launch.py"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),      # Type of robot
                "world" : LaunchConfiguration("world"),      # Execution context
                "ns"    : "honey",                           # Robot namespace
                "output": LaunchConfiguration("output"),     # Output for logging
                "pose"  : LaunchConfiguration("honey_pose"), # Initial robot pose
                "spurn" : LaunchConfiguration("spurn"),      # Prevent node
                "nodes" : LaunchConfiguration("nodes"),      # Launch node group
                "extra" : LaunchConfiguration("extra"),      # Inject extra nodes
                "debug" : LaunchConfiguration("debug"),      # Debug a node set
                "sim"   : LaunchConfiguration("sim"),        # SIM IP address
                "llp"   : LaunchConfiguration("llp"),        # LLP IP address
                "mlp"   : LaunchConfiguration("mlp"),        # MLP IP address
                "dds"   : LaunchConfiguration("dds"),        # Enable DDS
                "gtloc" : LaunchConfiguration("gtloc"),      # Use Ground Truth Localizer
            }.items(),
            condition=IfCondition(LaunchConfiguration("honey")),
        ),
        # Auto-insert bumble at a canned location
        IncludeLaunchDescription(
            get_launch_file("launch/spawn.launch.py"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),       # Type of robot
                "world" : LaunchConfiguration("world"),       # Execution context
                "ns"    : "bumble",                           # Robot namespace
                "output": LaunchConfiguration("output"),      # Output for logging
                "pose"  : LaunchConfiguration("bumble_pose"), # Initial robot pose
                "spurn" : LaunchConfiguration("spurn"),       # Prevent node
                "nodes" : LaunchConfiguration("nodes"),       # Launch node group
                "extra" : LaunchConfiguration("extra"),       # Inject extra nodes
                "debug" : LaunchConfiguration("debug"),       # Debug a node set
                "sim"   : LaunchConfiguration("sim"),         # SIM IP address
                "llp"   : LaunchConfiguration("llp"),         # LLP IP address
                "mlp"   : LaunchConfiguration("mlp"),         # MLP IP address
                "dds"   : LaunchConfiguration("dds"),         # Enable DDS
                "gtloc" : LaunchConfiguration("gtloc"),       # Use Ground Truth Localizer
            }.items(),
            condition=IfCondition(LaunchConfiguration("bumble")),
        ),
        # Auto-insert queen at a canned location
        IncludeLaunchDescription(
            get_launch_file("launch/spawn.launch.py"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),       # Type of robot
                "world" : LaunchConfiguration("world"),       # Execution context
                "ns"    : "queen",                            # Robot namespace
                "output": LaunchConfiguration("output"),      # Output for logging
                "pose"  : LaunchConfiguration("queen_pose"),  # Initial robot pose
                "spurn" : LaunchConfiguration("spurn"),       # Prevent node
                "nodes" : LaunchConfiguration("nodes"),       # Launch node group
                "extra" : LaunchConfiguration("extra"),       # Inject extra nodes
                "debug" : LaunchConfiguration("debug"),       # Debug a node set
                "sim"   : LaunchConfiguration("sim"),         # SIM IP address
                "llp"   : LaunchConfiguration("llp"),         # LLP IP address
                "mlp"   : LaunchConfiguration("mlp"),         # MLP IP address
                "dds"   : LaunchConfiguration("dds"),         # Enable DDS
                "gtloc" : LaunchConfiguration("gtloc"),       # Use Ground Truth Localizer
            }.items(),
            condition=IfCondition(LaunchConfiguration("queen")),
        ),
    ])