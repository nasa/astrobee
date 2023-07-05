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

    return LaunchDescription([
        launch_arg("robot",  description="Robot name"),
        launch_arg("world",  description="World name"),
        launch_arg("ns",     description="Robot namespace prefix"),
        launch_arg("output", description="Where nodes should log"),
        launch_arg("spurn",  description="Prevent a specific node"),
        launch_arg("nodes",  description="Launch specific nodes"),
        launch_arg("extra",  description="Inject additional node"),
        launch_arg("debug",  description="Debug node group"),
        launch_arg("sim",    description="SIM IP address"),
        launch_arg("llp",    description="LLP IP address"),
        launch_arg("mlp",    description="MLP IP address"),
        launch_arg("dds",    description="Enable DDS"),
        launch_arg("gtloc",  description="Use Ground Truth Localizer"),

        # Robot args
        launch_arg("orion", description="Insert Orion robot"),
        launch_arg("apollo", description="Insert Apollo robot"),
        launch_arg("leo", description="Insert Leo robot"),
        launch_arg("orion_pose", description="Orion's pose"),
        launch_arg("apollo_pose", description="Apollo's pose"),
        launch_arg("leo_pose", description="Leo's pose"),
        # Launch files
        IncludeLaunchDescription(
            get_launch_file("launch/controller/descriptions.launch.py"),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        # Insert Orion
        IncludeLaunchDescription(
            get_launch_file("launch/discower/spawn_robots.launch.py"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),      # Type of robot
                "world" : LaunchConfiguration("world"),      # Execution context
                "ns"    : "orion",                           # Robot namespace
                "output": LaunchConfiguration("output"),     # Output for logging
                "pose"  : LaunchConfiguration("orion_pose"), # Initial robot pose
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
            condition=IfCondition(LaunchConfiguration("orion")),
        ),
        # Insert Apollo
        IncludeLaunchDescription(
            get_launch_file("launch/discower/spawn_robots.launch.py"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),       # Type of robot
                "world" : LaunchConfiguration("world"),       # Execution context
                "ns"    : "apollo",                           # Robot namespace
                "output": LaunchConfiguration("output"),      # Output for logging
                "pose"  : LaunchConfiguration("apollo_pose"), # Initial robot pose
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
            condition=IfCondition(LaunchConfiguration("apollo")),
        ),
        # Insert Leo
        IncludeLaunchDescription(
            get_launch_file("launch/discower/spawn_robots.launch.py"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),       # Type of robot
                "world" : LaunchConfiguration("world"),       # Execution context
                "ns"    : "leo",                              # Robot namespace
                "output": LaunchConfiguration("output"),      # Output for logging
                "pose"  : LaunchConfiguration("leo_pose"),    # Initial robot pose
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
            condition=IfCondition(LaunchConfiguration("leo")),
        ),
    ])