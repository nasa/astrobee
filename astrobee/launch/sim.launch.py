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
        launch_arg("robot",  default_value=os.getenv("ASTROBEE_ROBOT", "sim"),description="Robot name"),
        launch_arg("world",  default_value="discower",
                            description="World name"),

        launch_arg("ns",     default_value="",    description="Robot namespace prefix"),
        launch_arg("output", default_value="log", description="Where nodes should log"),
        launch_arg("spurn",  default_value="",    description="Prevent a specific node"),
        launch_arg("nodes",  default_value="",    description="Launch specific nodes"),
        launch_arg("extra",  default_value="",    description="Inject additional node"),
        launch_arg("debug",  default_value="",    description="Debug node group"),

        # Physics engine options: ode (gazebo default), bullet, simbody, dart
        launch_arg("physics", default_value="ode",   description="Choose physics engine"),
        launch_arg("sim",     default_value="local", description="SIM IP address"),
        launch_arg("llp",     default_value="local", description="LLP IP address"),
        launch_arg("mlp",     default_value="local", description="MLP IP address"),
        launch_arg("rec",     default_value="",      description="Record local data "),
        launch_arg("dds",     default_value="true",  description="Enable DDS"),
        launch_arg("gtloc",   default_value="true", description="Use Ground Truth Localizer"),
        launch_arg("perch",   default_value="false", description="Start in the perch position"),
        # General options
        launch_arg("gviz",   default_value="false",  description="Start GNC visualizer"),
        launch_arg("rviz",   default_value="false",  description="Start Rviz visualization"),
        launch_arg("sviz",   default_value="false",  description="Start Gazebo visualization"),
        launch_arg("rqt",    default_value="false",  description="Start user interface"),
        launch_arg("gds",    default_value="false",  description="Start GDS"),
        launch_arg("agent1", default_value="Queen",  description="GDS Agent1"),
        launch_arg("agent2", default_value="Bumble", description="GDS Agent2"),
        launch_arg("agent3", default_value="Honey",  description="GDS Agent3"),
        launch_arg("vmware", default_value="true",   description="Enable vmware"),
        launch_arg("speed",  default_value="1",      description="Speed multiplier"),
        launch_arg("sdebug", default_value="false",  description="Debug simulator "),
        launch_arg("stats",  default_value="false",  description="Message statistics"),
        
        # Debug-specific options
        # Default starting pose based on scenario
        # Perch mode assumes ISS world
        launch_arg("default_robot", default_value="false", description="Insert default robot "),
        launch_arg("pose",  default_value="9.92 -9.54 4.50 0 0 0",),
        launch_arg("pose", default_value="9.92 -9.54 4.50 0 0 0",
                           condition=IfCondition(LaunchConfiguration("perch"))),
        # Default is using JPM Berth 1, for Berth 2 use: '9.817 -10.312 4.293 1 0 0 0'
        launch_arg("pose", default_value="9.816 -9.806 4.293 0 0 0",
                           condition=LaunchConfigurationEquals("world", "iss")),
        launch_arg("pose", default_value="0 0 -0.7 0 0 0",
                           condition=LaunchConfigurationEquals("world", "granite")),
        
        # Multi-robot simulation
        launch_arg("honey", default_value="true", description="Insert honey robot"),
        launch_arg("bumble", default_value="false", description="Insert bumble robot"),
        launch_arg("queen", default_value="false", description="Insert queen robot"),
        launch_arg("honey_pose",  default_value="11 -8 4.8 0 0 0", description="Overwrite honey's pose"),
        launch_arg("bumble_pose",  default_value="11 -9 4.8 0 0 0", description="Overwrite bumble's pose"),
        launch_arg("queen_pose",  default_value="11 -10 4.8 0 0 0", description="Overwrite queen's pose"),

        launch_arg("orion", default_value="true", description="Insert Orion robot"),
        launch_arg("apollo", default_value="true", description="Insert Apollo robot"),
        launch_arg("leo", default_value="true", description="Insert Leo robot"),
        launch_arg("orion_pose",  default_value="0.5 0.0 0 0 0 0", description="Overwrite orion's pose"),
        launch_arg("apollo_pose",  default_value="1 1 0 0 0 0", description="Overwrite apollo's pose"),
        launch_arg("leo_pose",  default_value="0.5 1 0 0 0 0", description="Overwrite leo's pose"),
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
        # Always launch on the local machine
        #   <group>
        #     <machine name ="local" address="localhost" default="true"/>
        # Start the descriptions (ISS, dock, granite) for visualization purposes
        # Start ground controller services
        IncludeLaunchDescription(
            get_launch_file("launch/controller/stats.launch.py"),
            condition=IfCondition(LaunchConfiguration("stats")),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/gviz.launch.py"),
            condition=IfCondition(LaunchConfiguration("gviz")),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/rqt.launch.py"),
            condition=IfCondition(LaunchConfiguration("rqt")),
        ),
        IncludeLaunchDescription(
            get_launch_file("launch/controller/rviz.launch.py"),
            condition=IfCondition(LaunchConfiguration("rviz")),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        # Launch a recorder for this robot
        IncludeLaunchDescription(
            get_launch_file("launch/controller/bagrecord.launch.py"),
            condition=LaunchConfigurationNotEquals("rec", ""),
            launch_arguments={"bag": LaunchConfiguration("rec")}.items(),
        ),
        #   <!-- Allow the simulator to be optionally launched remotely-->
        #   <!-- Connect and update environment variables if required -->
        #   <machine unless="$(eval arg('sim')=='local')" name="sim_server" default="true"
        #            address="$(arg sim)" user="astrobee" password="astrobee" timeout="10"/>
        # Update the environment variables relating to absolute paths
        SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",
                               value=os.getenv("ASTROBEE_CONFIG_DIR", "/home/astrobee/native/config"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),
        SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",
                               value=os.getenv("ASTROBEE_RESOURCE_DIR", "/home/astrobee/native/resources"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),
        SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE",
                               value=os.getenv("ROSCONSOLE_CONFIG_FILE", "/home/astrobee/native/resources/logging.config"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),

        SetEnvironmentVariable(name="DISPLAY", value=":0",
                               condition=LaunchConfigurationNotEquals("sim", "local")),
        SetEnvironmentVariable(name="ROS_IP", value=LaunchConfiguration("sim"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),
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
        
        # Spawn Astrobee's if the world is ISS or Granite
        # IncludeLaunchDescription(
        #     get_launch_file("launch/spawn_astrobee.launch.py"),
        #     launch_arguments={
        #         "robot" : LaunchConfiguration("robot"),                         # Type of robot
        #         "world" : LaunchConfiguration("world"),                         # Execution context
        #         "ns"    : LaunchConfiguration("ns"),                            # Robot namespace
        #         "output": LaunchConfiguration("output"),                        # Output for logging
        #         "spurn" : LaunchConfiguration("spurn"),                         # Prevent node
        #         "nodes" : LaunchConfiguration("nodes"),                         # Launch node group
        #         "extra" : LaunchConfiguration("extra"),                         # Inject extra nodes
        #         "debug" : LaunchConfiguration("debug"),                         # Debug a node set
        #         "sim"   : LaunchConfiguration("sim"),                           # SIM IP address
        #         "llp"   : LaunchConfiguration("llp"),                           # LLP IP address
        #         "mlp"   : LaunchConfiguration("mlp"),                           # MLP IP address
        #         "dds"   : LaunchConfiguration("dds"),                           # Enable DDS
        #         "gtloc" : LaunchConfiguration("gtloc"),                         # Use Ground Truth Localizer
        #         "agent1": LaunchConfiguration("agent1"),                        # GDS Agent 1
        #         "agent2": LaunchConfiguration("agent2"),                        # GDS Agent 2
        #         "agent3": LaunchConfiguration("agent3"),                        # GDS Agent 3
        #         "default_robot" : LaunchConfiguration("default_robot"),         # Use default robot
        #         "pose" : LaunchConfiguration("pose"),                           # Default robot pose
        #         "honey" : LaunchConfiguration("honey"),                         # Default robot pose
        #         "honey_pose" : LaunchConfiguration("honey_pose"),               # Default robot pose
        #         "bumble" : LaunchConfiguration("bumble"),                       # Default robot pose
        #         "bumble_pose" : LaunchConfiguration("bumble_pose"),             # Default robot pose
        #         "queen" : LaunchConfiguration("queen"),                        # Default robot pose
        #         "queen_pose" : LaunchConfiguration("queen_pose"),              # Default robot pose
        #     }.items(),
        #     condition=LaunchConfigurationNotEquals("world", "discower"),
        # ),
        IncludeLaunchDescription(
            get_launch_file("launch/discower/spawn_discower.launch.py"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),                         # Type of robot
                "world" : LaunchConfiguration("world"),                         # Execution context
                "ns"    : LaunchConfiguration("ns"),                            # Robot namespace
                "output": LaunchConfiguration("output"),                        # Output for logging
                "spurn" : LaunchConfiguration("spurn"),                         # Prevent node
                "nodes" : LaunchConfiguration("nodes"),                         # Launch node group
                "extra" : LaunchConfiguration("extra"),                         # Inject extra nodes
                "debug" : LaunchConfiguration("debug"),                         # Debug a node set
                "sim"   : LaunchConfiguration("sim"),                           # SIM IP address
                "llp"   : LaunchConfiguration("llp"),                           # LLP IP address
                "mlp"   : LaunchConfiguration("mlp"),                           # MLP IP address
                "dds"   : LaunchConfiguration("dds"),                           # Enable DDS
                "gtloc" : LaunchConfiguration("gtloc"),                         # Use Ground Truth Localizer
                "agent1": LaunchConfiguration("agent1"),                        # GDS Agent 1
                "agent2": LaunchConfiguration("agent2"),                        # GDS Agent 2
                "agent3": LaunchConfiguration("agent3"),                        # GDS Agent 3
                "orion" : LaunchConfiguration("orion"),                         # Default robot pose
                "orion_pose" : LaunchConfiguration("orion_pose"),               # Default robot pose
                "apollo" : LaunchConfiguration("apollo"),                       # Default robot pose
                "apollo_pose" : LaunchConfiguration("apollo_pose"),             # Default robot pose
                "leo" : LaunchConfiguration("leo"),                             # Default robot pose
                "leo_pose" : LaunchConfiguration("leo_pose"),                   # Default robot pose
            }.items(),
        ),
        ]
    )
