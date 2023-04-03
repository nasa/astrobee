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
        # Update the environment variables relating to absolute paths
        SetEnvironmentVariable(name="ASTROBEE_ROBOT",         condition=LaunchConfigurationNotEquals("mlp", "local"),
                               value=os.getenv("ASTROBEE_ROBOT", LaunchConfiguration("robot"))),
        SetEnvironmentVariable(name="ASTROBEE_WORLD",         condition=LaunchConfigurationNotEquals("mlp", "local"),
                               value=os.getenv("ASTROBEE_WORLD", LaunchConfiguration("world"))),
        SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",    condition=LaunchConfigurationNotEquals("mlp", "local"),
                               value=os.getenv("ASTROBEE_CONFIG_DIR", "/opt/astrobee/config")),
        SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",  condition=LaunchConfigurationNotEquals("mlp", "local"),
                               value=os.getenv("ASTROBEE_RESOURCE_DIR", "/res")),
        SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE", condition=LaunchConfigurationNotEquals("mlp", "local"),
                               value=os.getenv("ROSCONSOLE_CONFIG_FILE", "/res/logging.config")),
        
        SetEnvironmentVariable(name="ROS_HOSTNAME", condition=LaunchConfigurationNotEquals("mlp", "local"),
                               value=LaunchConfiguration("mlp")),


        DeclareLaunchArgument("drivers",),                    # Start platform drivers
        DeclareLaunchArgument("spurn", default_value=""),     # Prevent a specific node
        DeclareLaunchArgument("nodes", default_value=""),     # Launch specific nodes
        DeclareLaunchArgument("extra", default_value=""),     # Inject an additional node
        DeclareLaunchArgument("debug", default_value=""),     # Debug node group
        DeclareLaunchArgument("dds",   default_value="true"), # Enable DDS

        DeclareLaunchArgument("output",  default_value="log"),    # Output to screen or log
        DeclareLaunchArgument("gtloc",   default_value="false"),  # Use Ground Truth Localizer

        ComposableNodeContainer(
        name='mlp_localization',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("gtloc")),
        composable_node_descriptions=[
            ComposableNode(
                 package='localization_manager',
                 plugin='localization_manager::LocalizationManagerComponent',
                 name='localization_manager',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ComposableNode(
                package='ground_truth_localizer',
                plugin='ground_truth_localizer::GroundTruthLocalizerComponent',
                name='ground_truth_localizer',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            # ComposableNode(
            #     package='image_sampler',
            #     plugin='image_sampler::ImageSampler',
            #     name='image_sampler',
            #     remappings=[('/image', '/burgerimage')],
            #     parameters=[{'history': 'keep_last'}],
            #     extra_arguments=[{'use_intra_process_comms': True}])
            ]
        ),
        ComposableNodeContainer(
        name='mlp_localization',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=UnlessCondition(LaunchConfiguration("gtloc")),
        composable_node_descriptions=[
            ComposableNode(
                package='localization_manager',
                plugin='localization_manager::LocalizationManagerNodelet',
                name='localization_manager',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            # ComposableNode(
            #     package='image_sampler',
            #     plugin='image_sampler::ImageSampler',
            #     name='image_sampler',
            #     remappings=[('/image', '/burgerimage')],
            #     parameters=[{'history': 'keep_last'}],
            #     extra_arguments=[{'use_intra_process_comms': True}])
            ]
        ),
        ComposableNodeContainer(
        name='mlp_graph_localization',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=UnlessCondition(LaunchConfiguration("gtloc")),
        composable_node_descriptions=[
            # ComposableNode(
            #     package='graph_loc',
            #     plugin='graph_localizer::GraphLocalizerNodelet',
            #     name='graph_loc',
            #     extra_arguments=[{'use_intra_process_comms': True}])
            ]
        ),
        ComposableNodeContainer(
        name='mlp_vision',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
            # ComposableNode(
            #     package='depth_odometry_nodelet',
            #     plugin='depth_odometry_nodelet::DepthOdometryNodelet',
            #     name='depth_odometry_nodelet',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='is_camera',
            #     plugin='is_camera::camera',
            #     name='nav_cam',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='is_camera',
            #     plugin='is_camera::camera',
            #     name='calibration_nav_cam',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='localization_node',
            #     plugin='localization_node::LocalizationNodelet',
            #     name='localization_node',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='lk_optical_flow',
            #     plugin='lk_optical_flow::LKOpticalFlowNodelet',
            #     name='optical_flow_nodelet',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='is_camera',
            #     plugin='debayer::DebayerNodelet',
            #     name='nav_cam_debayer',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='is_camera',
            #     plugin='is_camera::camera',
            #     name='dock_cam',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='is_camera',
            #     plugin='is_camera::camera',
            #     name='calibration_dock_cam',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='marker_tracking',
            #     plugin='marker_tracking_node::MarkerTrackingNodelet',
            #     name='marker_tracking',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='is_camera',
            #     plugin='debayer::DebayerNodelet',
            #     name='dock_cam_debayer',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_depth_cam',
        namespace='',
        package='rclcpp_components',
        condition=UnlessCondition(LaunchConfiguration("drivers")),
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     package='handrail_detect',
            #     plugin='handrail_detect::HandrailDetect',
            #     name='handrail_detect',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='planner_qp',
            #     plugin='planner_qp::Planner',
            #     name='planner_qp',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_depth_cam',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
            # ComposableNode(
            #     package='handrail_detect',
            #     plugin='handrail_detect::HandrailDetect',
            #     name='handrail_detect',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='planner_qp',
            #     plugin='planner_qp::Planner',
            #     name='planner_qp',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='pico_driver',
            #     plugin='pico_driver::PicoDriverNodelet',
            #     name='pico_driver',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='pico_driver',
            #     plugin='pico_proxy::PicoProxyNodelet',
            #     name='pico_proxy_haz_cam_extended',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_mapper',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mapper',
                plugin='mapper::MapperComponent',
                name='mapper',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_management',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     package='executive',
            #     plugin='executive::Executive',
            #     name='executive',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='access_control',
            #     plugin='access_control::AccessControl',
            #     name='access_control',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_recording',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     package='data_bagger',
            #     plugin='data_bagger::DataBagger',
            #     name='data_bagger',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_monitors',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
            # ComposableNode(
            #     package='sys_monitor',
            #     plugin='sys_monitor::SysMonitor',
            #     name='sys_monitor',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='cpu_mem_monitor',
            #     plugin='cpu_mem_monitor::CpuMemMonitor',
            #     name='mlp_cpu_mem_monitor',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            # ComposableNode(
            #     package='disk_monitor',
            #     plugin='disk_monitor::DiskMonitor',
            #     name='mlp_disk_monitor',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_monitors',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=UnlessCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
            # ComposableNode(
            #     package='sys_monitor',
            #     plugin='sys_monitor::SysMonitor',
            #     name='sys_monitor',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_communications',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     package='dds_ros_bridge',
            #     plugin='dds_ros_bridge::DdsRosBridge',
            #     name='dds_ros_bridge',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_multibridge',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     package='dds_ros_bridge',
            #     plugin='dds_ros_bridge::AstrobeeAstrobeeBridge',
            #     name='astrobee_astrobee_bridge',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_serial',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
            # ComposableNode(
            #     package='perching_arm',
            #     plugin='perching_arm::PerchingArmNode',
            #     name='perching_arm',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_mobility',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='choreographer',
                plugin='choreographer::ChoreographerComponent',
                name='choreographer',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ComposableNode(
                package='planner_trapezoidal',
                plugin='planner_trapezoidal::PlannerTrapezoidalComponent',
                name='planner_trapezoidal',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ComposableNode(
                package='framestore',
                plugin='mobility::FrameStore',
                name='framestore',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_arm',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='arm',
                plugin='arm::ArmComponent',
                name='arm'),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_dock',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='dock',
                plugin='dock::DockComponent',
                name='dock'),
                #extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_perch',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='perch',
                plugin='perch::PerchComponent',
                name='perch'),
                #extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_vive',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
            # ComposableNode(
            #     package='vive',
            #     plugin='vive::ViveNodelet',
            #     name='vive',
            #     extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='mlp_states',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='states',
                plugin='states::StatesComponent',
                name='states',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ]
        ),
    ])

