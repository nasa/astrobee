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
        # ComposableNodeContainer(
        # name='llp_gnc',
        # namespace='',
        # package='rclcpp_components',
        # executable='component_container',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='ctl',
        #         plugin='ctl::ctl',
        #         name='ctl',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ComposableNode(
        #         package='fam',
        #         plugin='fam::fam',
        #         name='fam',
        #         extra_arguments=[{'use_intra_process_comms': True}])
        #     ]
        # ),
        # ComposableNodeContainer(
        # name='llp_imu_aug',
        # namespace='',
        # package='rclcpp_components',
        # executable='component_container',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='imu_augmentor',
        #         plugin='imu_augmentor::ImuAugmentorNodelet',
        #         name='imu_aug',
        #         condition=IfCondition(LaunchConfiguration("gtloc")),
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ]
        # ),
        # ComposableNodeContainer(
        # name='llp_monitors',
        # namespace='',
        # package='rclcpp_components',
        # executable='component_container',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='localization_manager',
        #         plugin='localization_manager::LocalizationManagerNodelet',
        #         name='localization_manager',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ]
        # ),
        # ComposableNodeContainer(
        # name='llp_i2c',
        # namespace='',
        # package='rclcpp_components',
        # executable='component_container',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='localization_manager',
        #         plugin='localization_manager::LocalizationManagerNodelet',
        #         name='localization_manager',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ]
        # ),
        # ComposableNodeContainer(
        # name='llp_serial',
        # namespace='',
        # package='rclcpp_components',
        # executable='component_container',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='localization_manager',
        #         plugin='localization_manager::LocalizationManagerNodelet',
        #         name='localization_manager',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ]
        # ),
        # ComposableNodeContainer(
        # name='llp_pmc',
        # namespace='',
        # package='rclcpp_components',
        # executable='component_container',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='localization_manager',
        #         plugin='localization_manager::LocalizationManagerNodelet',
        #         name='localization_manager',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ]
        # ),
        # ComposableNodeContainer(
        # name='llp_imu',
        # namespace='',
        # package='rclcpp_components',
        # executable='component_container',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='localization_manager',
        #         plugin='localization_manager::LocalizationManagerNodelet',
        #         name='localization_manager',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ]
        # ),
        # ComposableNodeContainer(
        # name='llp_lights',
        # namespace='',
        # package='rclcpp_components',
        # executable='component_container',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='localization_manager',
        #         plugin='localization_manager::LocalizationManagerNodelet',
        #         name='localization_manager',
        #         extra_arguments=[{'use_intra_process_comms': True}]),
        #     ]
        # ),
    ])


# <launch>

#   <!-- Additional options -->
#   <arg name="drivers"/>                       <!-- Should we launch drivers? -->
#   <arg name="spurn" default=""/>              <!-- PRevent a specific node   -->
#   <arg name="nodes" default=""/>              <!-- Launch specific nodes     -->
#   <arg name="extra" default=""/>              <!-- Inject an additional node -->
#   <arg name="debug" default=""/>              <!-- Debug a node set          -->
#   <arg name="output" default="log"/>          <!-- Where to log              -->
#   <arg name="gtloc" default="false"/>         <!-- Runs ground_truth_localizer -->



#   <!-- Launch GNC nodes -->
#   <group unless="$(arg gtloc)">
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="imu_augmentor/ImuAugmentorNodelet" />
#       <arg name="name" value="imu_aug" />
#       <arg name="manager" value="llp_imu_aug" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>
#   </group>


#   <!-- Launch driver nodes, if required -->
#   <group if="$(arg drivers)">
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="epson_imu/EpsonImuNodelet" />
#       <arg name="name" value="epson_imu" />
#       <arg name="manager" value="llp_imu" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="epson_imu/EpsonImuNodelet" />
#       <arg name="name" value="calibration_imu" />
#       <arg name="manager" value="llp_imu" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="false" />
#     </include>
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="pmc_actuator/PmcActuatorNodelet" />
#       <arg name="name" value="pmc_actuator" />
#       <arg name="manager" value="llp_pmc" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="eps_driver/EpsDriverNode" />
#       <arg name="name" value="eps_driver" />
#       <arg name="manager" value="llp_i2c" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="laser/LaserNodelet" />
#       <arg name="name" value="laser" />
#       <arg name="manager" value="llp_i2c" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="flashlight/FlashlightNodelet" />
#       <arg name="name" value="flashlight_front" />
#       <arg name="manager" value="llp_i2c" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="flashlight/FlashlightNodelet" />
#       <arg name="name" value="flashlight_aft" />
#       <arg name="manager" value="llp_i2c" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>

#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="signal_lights/SignalLightsNodelet" />
#       <arg name="name" value="signal_lights" />
#       <arg name="manager" value="llp_lights" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>

#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="light_flow/LightFlowNodelet" />
#       <arg name="name" value="light_flow" />
#       <arg name="manager" value="llp_lights" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>

#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="speed_cam/SpeedCamNode" />
#       <arg name="name" value="speed_cam" />
#       <arg name="manager" value="llp_serial" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="cpu_mem_monitor/CpuMemMonitor" />
#       <arg name="name" value="llp_cpu_mem_monitor" />
#       <arg name="manager" value="llp_monitors" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>
#     <include file="$(find ff_util)/launch/ff_nodelet.launch">
#       <arg name="class" value="disk_monitor/DiskMonitor" />
#       <arg name="name" value="llp_disk_monitor" />
#       <arg name="manager" value="llp_monitors" />
#       <arg name="spurn" value="$(arg spurn)" />
#       <arg name="nodes" value="$(arg nodes)" />
#       <arg name="extra" value="$(arg extra)" />
#       <arg name="debug" value="$(arg debug)" />
#       <arg name="default" value="true" />
#     </include>

#   </group>

# </launch>
