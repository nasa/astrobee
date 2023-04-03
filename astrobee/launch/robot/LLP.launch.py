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
        SetEnvironmentVariable(name="ASTROBEE_ROBOT",         condition=LaunchConfigurationNotEquals("llp", "local"),
                               value=os.getenv("ASTROBEE_ROBOT", LaunchConfiguration("robot"))),
        SetEnvironmentVariable(name="ASTROBEE_WORLD",         condition=LaunchConfigurationNotEquals("llp", "local"),
                               value=os.getenv("ASTROBEE_WORLD", LaunchConfiguration("world"))),
        SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",    condition=LaunchConfigurationNotEquals("llp", "local"),
                               value=os.getenv("ASTROBEE_CONFIG_DIR", "/opt/astrobee/config")),
        SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",  condition=LaunchConfigurationNotEquals("llp", "local"),
                               value=os.getenv("ASTROBEE_RESOURCE_DIR", "/res")),
        SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE", condition=LaunchConfigurationNotEquals("llp", "local"),
                               value=os.getenv("ROSCONSOLE_CONFIG_FILE", "/res/logging.config")),


        SetEnvironmentVariable(name="ROS_HOSTNAME", condition=LaunchConfigurationNotEquals("llp", "local"),
                               value=LaunchConfiguration("llp")),


        DeclareLaunchArgument("drivers",),                    # Start platform drivers
        DeclareLaunchArgument("spurn", default_value=""),     # Prevent a specific node
        DeclareLaunchArgument("nodes", default_value=""),     # Launch specific nodes
        DeclareLaunchArgument("extra", default_value=""),     # Inject an additional node
        DeclareLaunchArgument("debug", default_value=""),     # Debug node group

        DeclareLaunchArgument("output",  default_value="log"),    # Output to screen or log
        DeclareLaunchArgument("gtloc",   default_value="false"),  # Use Ground Truth Localizer



        ComposableNodeContainer(
        name='llp_gnc',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ctl',
                plugin='ctl::CtlComponent',
                name='ctl',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ComposableNode(
                package='fam',
                plugin='fam::FamComponent',
                name='fam',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='llp_imu_aug',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("gtloc")),
        composable_node_descriptions=[
        #     ComposableNode(
        #         package='imu_augmentor',
        #         plugin='imu_augmentor::ImuAugmentorNodelet',
        #         name='imu_aug',
        #         condition=IfCondition(LaunchConfiguration("gtloc")),
        #         extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='llp_monitors',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
        #     ComposableNode(
        #         package='cpu_mem_monitor',
        #         plugin='cpu_mem_monitor::CpuMemMonitor',
        #         name='llp_cpu_mem_monitor',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ComposableNode(
        #         package='disk_monitor',
        #         plugin='disk_monitor::DiskMonitor',
        #         name='llp_disk_monitor',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='llp_i2c',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
        #     ComposableNode(
        #         package='eps_driver',
        #         plugin='eps_driver::EpsDriverNode',
        #         name='eps_driver',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ComposableNode(
        #         package='laser',
        #         plugin='laser::LaserNodelet',
        #         name='laser',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ComposableNode(
        #         package='flashlight',
        #         plugin='flashlight::FlashlightNodelet',
        #         name='flashlight_front',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ComposableNode(
        #         package='flashlight',
        #         plugin='flashlight::FlashlightNodelet',
        #         name='flashlight_aft',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='llp_serial',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
        #     ComposableNode(
        #         package='speed_cam',
        #         plugin='speed_cam::SpeedCamNode',
        #         name='speed_cam',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='llp_pmc',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
        #     ComposableNode(
        #         package='pmc_actuator',
        #         plugin='pmc_actuator::PmcActuatorNodelet',
        #         name='pmc_actuator',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='llp_imu',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
        #     ComposableNode(
        #         package='epson_imu',
        #         plugin='epson_imu::EpsonImuNodelet',
        #         name='localization_manager',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ComposableNode(
        #         package='calibration_imu',
        #         plugin='epson_imu::EpsonImuNodelet',
        #         name='localization_manager',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
            ]
        ),
        ComposableNodeContainer(
        name='llp_lights',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("drivers")),
        composable_node_descriptions=[
        #     ComposableNode(
        #         package='signal_lights',
        #         plugin='signal_lights::SignalLightsNodelet',
        #         name='signal_lights',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='light_flow',
                plugin='light_flow::LightFlowComponent',
                name='light_flow',
                extra_arguments=[{'use_intra_process_comms': False, 'use_sim_time': True}]),
            ]
        ),
    ])
