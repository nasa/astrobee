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
        DeclareLaunchArgument("robot", default_value=os.getenv("ASTROBEE_ROBOT", "sim")),
        DeclareLaunchArgument("world", default_value=os.getenv("ASTROBEE_WORLD", "iss")),
        DeclareLaunchArgument("pose",  default_value="9.92 -9.54 4.50 0 0 0",
                                       condition=LaunchConfigurationEquals("world", "iss")),
        DeclareLaunchArgument("pose",  default_value="9.92 -9.54 4.50 0 0 0",
                                       condition=LaunchConfigurationEquals("world", "granite")),

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
        IncludeLaunchDescription(
            get_launch_file("launch/astrobee.launch.py"),
            launch_arguments={
                "robot"  : LaunchConfiguration("robot"),                   # Type of robot
                "world"  : LaunchConfiguration("world"),                   # Execution context
                "ns"     : LaunchConfiguration("ns",     default=""),      # Robot namespace
                "output" : LaunchConfiguration("output", default="log"),   # Output for logging
                "pose"   : LaunchConfiguration("pose"),                    # Initial robot pose
                "drivers": "false",                                        # Don't start driver nodes
                "spurn"  : LaunchConfiguration("spurn", default=""),       # Prevent node
                "nodes"  : LaunchConfiguration("nodes", default=""),       # Launch node group
                "extra"  : LaunchConfiguration("extra", default=""),       # Inject extra nodes
                "debug"  : LaunchConfiguration("debug", default=""),       # Debug a node set
                "sim"    : LaunchConfiguration("sim",   default="local"),  # SIM IP address
                "llp"    : LaunchConfiguration("llp",   default="local"),  # LLP IP address
                "mlp"    : LaunchConfiguration("mlp",   default="local"),  # MLP IP address
                "dds"    : LaunchConfiguration("dds",   default="false"),  # Enable DDS
                "gtloc"  : LaunchConfiguration("gtloc", default="false"),  # Use Ground Truth Localizer
            }.items(),
        ),
    ])
