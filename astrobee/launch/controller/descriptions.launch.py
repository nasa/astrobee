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


from utilities.utilities import *


def generate_launch_description():
    [granite_urdf,       granite_robot_description]       = get_urdf('urdf/model.urdf', 'astrobee_granite')
    [iss_urdf,           iss_robot_description]           = get_urdf('urdf/model.urdf', 'astrobee_iss')
    [dock_urdf,          dock_robot_description]          = get_urdf('urdf/model.urdf', 'astrobee_dock')
    [handrail_8_5_urdf,  handrail_8_5_robot_description]  = get_urdf('urdf/model.urdf', 'astrobee_handrail_8_5')
    [handrail_21_5_urdf, handrail_21_5_robot_description] = get_urdf('urdf/model.urdf', 'astrobee_handrail_21_5')
    [handrail_30_urdf,   handrail_30_robot_description]   = get_urdf('urdf/model.urdf', 'astrobee_handrail_30')
    [handrail_41_5_urdf, handrail_41_5_robot_description] = get_urdf('urdf/model.urdf', 'astrobee_handrail_41_5')

    return LaunchDescription([

        DeclareLaunchArgument("world"),

        # Granite robot description
        Node(
            package="robot_state_publisher",
            namespace="granite",
            executable="robot_state_publisher",
            name="astrobee_state_publisher",
            parameters=[{'robot_description': ParameterValue(granite_robot_description) }],
            arguments=[granite_urdf],
            condition=LaunchConfigurationEquals("world", "granite")
        ),
        # Granite robot description
        Node(
            package="robot_state_publisher",
            namespace="iss",
            executable="robot_state_publisher",
            name="astrobee_state_publisher",
            parameters=[{'robot_description': ParameterValue(iss_robot_description) }],
            arguments=[iss_urdf],
            condition=LaunchConfigurationEquals("world", "iss")
        ),
        # Granite robot description
        Node(
            package="robot_state_publisher",
            namespace="dock",
            executable="robot_state_publisher",
            name="astrobee_state_publisher",
            parameters=[{'robot_description': ParameterValue(dock_robot_description) }],
            arguments=[dock_urdf]
        ),
        # Granite robot description
        Node(
            package="robot_state_publisher",
            namespace="handrail_8_5",
            executable="robot_state_publisher",
            name="astrobee_state_publisher",
            parameters=[{'robot_description': ParameterValue(handrail_8_5_robot_description) }],
            arguments=[handrail_8_5_urdf]
        ),
        # Granite robot description
        Node(
            package="robot_state_publisher",
            namespace="handrail_21_5",
            executable="robot_state_publisher",
            name="astrobee_state_publisher",
            parameters=[{'robot_description': ParameterValue(handrail_21_5_robot_description) }],
            arguments=[handrail_21_5_urdf]
        ),
        # Granite robot description
        Node(
            package="robot_state_publisher",
            namespace="handrail_30",
            executable="robot_state_publisher",
            name="astrobee_state_publisher",
            parameters=[{'robot_description': ParameterValue(handrail_30_robot_description) }],
            arguments=[handrail_30_urdf]
        ),
        # Granite robot description
        Node(
            package="robot_state_publisher",
            namespace="handrail_41_5",
            executable="robot_state_publisher",
            name="astrobee_state_publisher",
            parameters=[{'robot_description': ParameterValue(handrail_41_5_robot_description) }],
            arguments=[handrail_41_5_urdf]
        ),
        # We must publish global transforms in case no robot has been spawned
        Node(
        package='framestore',
        namespace='',
        executable='global_transforms',
        name='global_transforms',
        )
    ])
