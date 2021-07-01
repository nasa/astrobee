/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef FF_UTIL_FF_NAMES_H_
#define FF_UTIL_FF_NAMES_H_

// UNIVERSAL ///////////////////////////////////////////////////////////////////

////////////
// GLOBAL //
////////////

#define TOPIC_CLOCK                                 "/clock"
#define TOPIC_DIAGNOSTICS                           "/diagnostics"
#define TOPIC_TF_DYNAMIC                            "/tf"
#define TOPIC_TF_STATIC                             "/tf_static"

////////////
// FRAMES //
////////////

#define FRAME_NAME_WORLD                            "world"
#define FRAME_NAME_BODY                             "body"
#define FRAME_NAME_TRUTH                            "truth"
#define FRAME_NAME_VIVE                             "vive"
#define FRAME_NAME_HANDRAIL                         "handrail/body"
#define FRAME_NAME_DOCK                             "dock/body"
#define FRAME_NAME_DOCK_BERTH_1                     "dock/berth1"
#define FRAME_NAME_DOCK_BERTH_2                     "dock/berth2"
#define FRAME_NAME_DOCK_MARKER                      "dock/marker"
#define FRAME_NAME_PERCH_CAM                        "perch_cam"
#define FRAME_NAME_HAZ_CAM                          "haz_cam"
#define FRAME_NAME_DOCK_CAM                         "dock_cam"
#define FRAME_NAME_NAV_CAM                          "nav_cam"
#define FRAME_NAME_FLASHLIGHT_FRONT                 "flashlight_front"
#define FRAME_NAME_FLASHLIGHT_AFT                   "flashlight_aft"
#define FRAME_NAME_LASER                            "laser"
#define FRAME_NAME_IMU                              "imu"

//////////////////
// LOCALIZATION //
//////////////////

#define LOCALIZATION_NONE                           "no"
#define LOCALIZATION_MAPPED_LANDMARKS               "ml"
#define LOCALIZATION_AR_TAGS                        "ar"
#define LOCALIZATION_HANDRAIL                       "hr"
#define LOCALIZATION_PERCH                          "pl"

//////////////////
// FLIGHT MODES //
//////////////////

#define FLIGHT_MODE_NOMINAL                         "nominal"
#define FLIGHT_MODE_DOCKING                         "docking"
#define FLIGHT_MODE_PERCHING                        "perching"


////////////
// SHARED //
////////////

#define PRIVATE_PREFIX                              "cfg/"

#define TOPIC_COMMAND                               "command"
#define TOPIC_HEARTBEAT                             "heartbeat"
#define TOPIC_JOINT_GOALS                           "joint_goals"
#define TOPIC_JOINT_STATES                          "joint_states"
#define TOPIC_PERFORMANCE                           "performance"
#define TOPIC_ROBOT_NAME                            "robot_name"
#define TOPIC_SIGNALS                               "signals"
#define TOPIC_TRIGGER                               "trigger"

// SUBSYSTEMS //////////////////////////////////////////////////////////////////

///////////
// COMMS //
///////////

#define SUBSYSTEM_COMMUNICATIONS                    "comm"

#define NODE_DDS_ROS_BRIDGE                         "dds_ros_bridge"
#define NODE_ASTROBEE_ASTROBEE_BRIDGE               "astrobee_astrobee_bridge"

#define SERVICE_COMMUNICATIONS_ASTROBEE_ASTROBEE_BRIDGE_TRIGGER "comm/astrobee_astrobee_bridge/trigger"
#define SERVICE_COMMUNICATIONS_DDS_SET_TELEM_RATES  "comm/dds/set_telem_rate"

#define TOPIC_COMMUNICATIONS_DDS_COMMAND            "comm/dds/command"
#define TOPIC_COMMUNICATIONS_DDS_DATA               "comm/dds/data"
#define TOPIC_COMMUNICATIONS_DDS_PLAN               "comm/dds/plan"
#define TOPIC_COMMUNICATIONS_DDS_ZONES              "comm/dds/zones"
#define TOPIC_COMMUNICATIONS_ROBOTS_AVAILABLE       "comm/robots_available"

/////////
// GNC //
/////////

#define SUBSYSTEM_GNC                               "gnc"

#define NODE_CTL                                    "ctl"
#define NODE_PERCH_CTL                              "perch_ctl"
#define NODE_EKF                                    "ekf"
#define NODE_GRAPH_LOC                              "graph_loc"
#define NODE_IMU_AUG                                "imu_aug"
#define NODE_IMU_BIAS_TESTER                        "imu_bias_tester"
#define NODE_SIM_LOC                                "sim_loc"
#define NODE_FAM                                    "fam"
#define NODE_SIM_WRAPPER                            "sim_wrapper"

#define TOPIC_GRAPH_LOC                             "graph_loc/graph"
#define TOPIC_GRAPH_LOC_STATE                       "graph_loc/state"
#define TOPIC_AR_TAG_POSE                           "ar_tag/pose"
#define TOPIC_SPARSE_MAPPING_POSE                   "sparse_mapping/pose"
#define TOPIC_IMU_BIAS_TESTER_POSE                  "imu_bias_tester/pose"
#define TOPIC_IMU_BIAS_TESTER_VELOCITY              "imu_bias_tester/velocity"

#define ACTION_GNC_CTL_CONTROL                      "gnc/control"

#define TOPIC_GNC_EKF                               "gnc/ekf"
#define TOPIC_GNC_EKF_FEATURES                      "gnc/ekf/features"
#define TOPIC_GNC_CTL_SHAPER                        "gnc/ctl/shaper"
#define TOPIC_GNC_CTL_TRAJ                          "gnc/ctl/traj"
#define TOPIC_GNC_CTL_SEGMENT                       "gnc/ctl/segment"
#define TOPIC_GNC_CTL_PROGRESS                      "gnc/ctl/progress"
#define TOPIC_GNC_CTL_SETPOINT                      "gnc/ctl/setpoint"
#define TOPIC_GNC_CTL_COMMAND                       "gnc/ctl/command"
#define TOPIC_GNC_EKF_RESET                         "gnc/ekf/reset"

#define SERVICE_GNC_EKF_RESET                       "gnc/ekf/reset"
#define SERVICE_GNC_EKF_RESET_HR                    "gnc/ekf/reset_hr"
#define SERVICE_GNC_EKF_INIT_BIAS                   "gnc/ekf/init_bias"
#define SERVICE_GNC_EKF_INIT_BIAS_FROM_FILE         "gnc/ekf/init_bias_from_file"
#define SERVICE_GNC_EKF_SET_INPUT                   "gnc/ekf/set_input"
#define SERVICE_GNC_CTL_ENABLE                      "gnc/ctl/enable"

///////////////////
// GUEST SCIENCE //
///////////////////

#define SUBSYSTEM_GUEST_SCIENCE                     "gs"

#define NODE_GUEST_SCIENCE_MANAGER                  "gs_manager"

#define TOPIC_GUEST_SCIENCE_DATA                    "gs/data"
#define TOPIC_GUEST_SCIENCE_MANAGER_ACK             "gs/gs_manager/ack"
#define TOPIC_GUEST_SCIENCE_MANAGER_CONFIG          "gs/gs_manager/config"
#define TOPIC_GUEST_SCIENCE_MANAGER_STATE           "gs/gs_manager/state"


///////////////
// MANGEMENT //
///////////////

#define SUBSYSTEM_MANAGEMENT                        "mgt"

#define NODE_ACCESS_CONTROL                         "access_control"
#define NODE_EXECUTIVE                              "executive"
#define NODE_SYS_MONITOR                            "sys_monitor"
#define NODE_IMG_SAMPLER                            "image_sampler"
#define NODE_DATA_BAGGER                            "data_bagger"

#define TOPIC_MANAGEMENT_ACK                          "mgt/ack"
#define TOPIC_MANAGEMENT_ACCESS_CONTROL_CMD           "mgt/access_control/cmd"
#define TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE         "mgt/access_control/state"
#define TOPIC_MANAGEMENT_CPU_MONITOR_STATE            "mgt/cpu_monitor/state"
#define TOPIC_MANAGEMENT_DISK_MONITOR_STATE           "mgt/disk_monitor/state"
#define TOPIC_MANAGEMENT_MEM_MONITOR_STATE            "mgt/mem_monitor/state"
#define TOPIC_MANAGEMENT_EXEC_AGENT_STATE             "mgt/executive/agent_state"
#define TOPIC_MANAGEMENT_EXEC_CF_ACK                  "mgt/executive/cf_ack"
#define TOPIC_MANAGEMENT_EXEC_COMMAND                 "mgt/executive/command"
#define TOPIC_MANAGEMENT_EXEC_PLAN                    "mgt/executive/plan"
#define TOPIC_MANAGEMENT_EXEC_PLAN_STATUS             "mgt/executive/plan_status"
#define TOPIC_MANAGEMENT_SYS_MONITOR_CONFIG           "mgt/sys_monitor/config"
#define TOPIC_MANAGEMENT_SYS_MONITOR_STATE            "mgt/sys_monitor/state"
#define TOPIC_MANAGEMENT_SYS_MONITOR_HEARTBEAT        "mgt/sys_monitor/heartbeat"
#define TOPIC_MANAGEMENT_SYS_MONITOR_TIME_DIFF        "mgt/sys_monitor/time_diff"
#define TOPIC_MANAGEMENT_DATA_BAGGER_STATE            "mgt/data_bagger/state"
#define TOPIC_MANAGEMENT_DATA_BAGGER_TOPICS           "mgt/data_bagger/topics"
#define TOPIC_MANAGEMENT_CAMERA_STATE                 "mgt/camera_state"
#define TOPIC_MANAGEMENT_IMG_SAMPLER_NAV_CAM_RECORD   "mgt/img_sampler/nav_cam/image_record"
#define TOPIC_MANAGEMENT_IMG_SAMPLER_NAV_CAM_STREAM   "mgt/img_sampler/nav_cam/image_stream"
#define TOPIC_MANAGEMENT_IMG_SAMPLER_DOCK_CAM_RECORD  "mgt/img_sampler/dock_cam/image_record"
#define TOPIC_MANAGEMENT_IMG_SAMPLER_DOCK_CAM_STREAM  "mgt/img_sampler/dock_cam/image_stream"
#define TOPIC_MANAGEMENT_IMG_SAMPLER_HAZ_CAM_RECORD   "mgt/img_sampler/haz_cam/image_record"
#define TOPIC_MANAGEMENT_IMG_SAMPLER_HAZ_CAM_STREAM   "mgt/img_sampler/haz_cam/image_stream"
#define TOPIC_MANAGEMENT_IMG_SAMPLER_PERCH_CAM_RECORD "mgt/img_sampler/perch_cam/image_record"
#define TOPIC_MANAGEMENT_IMG_SAMPLER_PERCH_CAM_STREAM "mgt/img_sampler/perch_cam/image_stream"

#define SERVICE_MANAGEMENT_DATA_BAGGER_ENABLE_RECORDING "mgt/data_bagger/enable_recording"
#define SERVICE_MANAGEMENT_DATA_BAGGER_SET_DATA_TO_DISK "mgt/data_bagger/set_data_to_disk"
#define SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_NAV   "mgt/img_sampler/nav_cam/configure"
#define SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_DOCK  "mgt/img_sampler/dock_cam/configure"
#define SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_HAZ   "mgt/img_sampler/haz_cam/configure"
#define SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_PERCH "mgt/img_sampler/perch_cam/configure"
#define SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_NAV   "mgt/img_sampler/nav_cam/enable"
#define SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_DOCK  "mgt/img_sampler/dock_cam/enable"
#define SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_HAZ   "mgt/img_sampler/haz_cam/enable"
#define SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_PERCH "mgt/img_sampler/perch_cam/enable"
#define SERVICE_MANAGEMENT_SYS_MONITOR_UNLOAD_LOAD_NODELET "mgt/sys_monitor/unload_load_nodelet"
#define SERVICE_MANAGEMENT_SCI_CAM_CONFIG           "configure_sci_cam"
#define SERVICE_MANAGEMENT_SCI_CAM_ENABLE           "enable_sci_cam"

//////////////
// MOBILITY //
//////////////

#define SUBSYSTEM_MOBILITY                          "mob"

#define NODE_CHOREOGRAPHER                          "choreographer"
#define NODE_MAPPER                                 "mapper"

// Exposed to peer nodes

#define ACTION_MOBILITY_MOTION                      "mob/motion"
#define ACTION_MOBILITY_VALIDATE                    "mob/validate"

#define SERVICE_MOBILITY_SET_INERTIA                "mob/set_inertia"
#define SERVICE_MOBILITY_SET_ZONES                  "mob/set_zones"
#define SERVICE_MOBILITY_GET_ZONES                  "mob/get_zones"
#define SERVICE_MOBILITY_SET_STATE                  "mob/set_state"
#define SERVICE_MOBILITY_UPDATE_MAP_RESOLUTION      "mob/mapper/update_resolution"
#define SERVICE_MOBILITY_UPDATE_MEMORY_TIME         "mob/mapper/update_memory_time"
#define SERVICE_MOBILITY_UPDATE_INFLATION           "mob/mapper/update_inflation_radius"
#define SERVICE_MOBILITY_RESET_MAP                  "mob/mapper/reset_map"
#define SERVICE_MOBILITY_GET_FREE_MAP               "mob/mapper/get_free_map"
#define SERVICE_MOBILITY_GET_OBSTACLE_MAP           "mob/mapper/get_obstacle_map"

#define TOPIC_MOBILITY_MOTION_STATE                 "mob/state"
#define TOPIC_MOBILITY_FLIGHT_MODE                  "mob/flight_mode"
#define TOPIC_MOBILITY_INERTIA                      "mob/inertia"
#define TOPIC_MOBILITY_MOTION_RESULT                "mob/motion/result"

// Used internally or for rviz plotting

#define SERVICE_MOBILITY_PLANNER_REGISTER           "mob/planner/register"

#define PREFIX_MOBILITY_PLANNER_PRIVATE             "planner_"
#define PREFIX_MOBILITY_PLANNER                     "mob/planner_"
#define SUFFIX_MOBILITY_PLANNER                     "/plan"

#define TOPIC_MOBILITY_SEGMENT                      "mob/choreographer/segment"
#define TOPIC_MOBILITY_HAZARD                       "mob/mapper/hazard"
#define TOPIC_MOBILITY_ZONES                        "mob/mapper/zones"
#define TOPIC_MAPPER_OCTOMAP_MARKERS                "mob/mapper/obstacle_markers"
#define TOPIC_MAPPER_OCTOMAP_FREE_MARKERS           "mob/mapper/free_space_markers"
#define TOPIC_MAPPER_OCTOMAP_CLOUD                  "mob/mapper/obstacle_cloud"
#define TOPIC_MAPPER_OCTOMAP_FREE_CLOUD             "mob/mapper/free_space_cloud"
#define TOPIC_MAPPER_OCTOMAP_INFLATED_MARKERS       "mob/mapper/inflated_obstacle_markers"
#define TOPIC_MAPPER_OCTOMAP_INFLATED_FREE_MARKERS  "mob/mapper/inflated_free_space_markers"
#define TOPIC_MAPPER_OCTOMAP_INFLATED_CLOUD         "mob/mapper/inflated_obstacle_cloud"
#define TOPIC_MAPPER_OCTOMAP_INFLATED_FREE_CLOUD    "mob/mapper/inflated_free_space_cloud"
#define TOPIC_MAPPER_FRUSTRUM_MARKERS               "mob/mapper/frustum_markers"
#define TOPIC_MAPPER_DISCRETE_TRAJECTORY_MARKERS    "mob/mapper/discrete_trajectory_markers"

//////////////////
// LOCALIZATION //
//////////////////

#define SUBSYSTEM_LOCALIZATION                      "loc"

#define NODE_HANDRAIL_DETECT                        "handrail_detect"
#define NODE_OPTICAL_FLOW                           "of"
#define NODE_LOCALIZATION_MANAGER                   "localization_manager"
#define NODE_AR_TAGS                                "ar_tags"
#define NODE_MAPPED_LANDMARKS                       "ml"
#define NODE_VIVE_LOCALIZATION                      "vive_localization"

#define TOPIC_LOCALIZATION_POSE                     "loc/pose"
#define TOPIC_LOCALIZATION_TWIST                    "loc/twist"
#define TOPIC_LOCALIZATION_TRUTH                    "loc/truth/pose"
#define TOPIC_LOCALIZATION_TRUTH_TWIST              "loc/truth/twist"
#define TOPIC_LOCALIZATION_OVERHEAD_IMAGE           "loc/overhead"

#define TOPIC_LOCALIZATION_ML_FEATURES              "loc/ml/features"
#define TOPIC_LOCALIZATION_ML_REGISTRATION          "loc/ml/registration"
#define TOPIC_LOCALIZATION_AR_FEATURES              "loc/ar/features"
#define TOPIC_LOCALIZATION_AR_REGISTRATION          "loc/ar/registration"
#define TOPIC_LOCALIZATION_OF_FEATURES              "loc/of/features"
#define TOPIC_LOCALIZATION_OF_REGISTRATION          "loc/of/registration"
#define TOPIC_LOCALIZATION_HR_FEATURES              "loc/hr/features"
#define TOPIC_LOCALIZATION_HR_REGISTRATION          "loc/hr/registration"
#define TOPIC_LOCALIZATION_HR_IMAGE                 "loc/hr/image"
#define TOPIC_LOCALIZATION_HR_CLOUD                 "loc/hr/cloud"
#define TOPIC_LOCALIZATION_HR_MARKER                "loc/hr/marker"
#define TOPIC_LOCALIZATION_SM_FEATURES              "loc/sm/features"

#define TOPIC_LOCALIZATION_OF_DEBUG                 "loc/of/debug_img"

#define SERVICE_LOCALIZATION_OF_ENABLE              "loc/of/enable"
#define SERVICE_LOCALIZATION_AR_ENABLE              "loc/ar/enable"
#define SERVICE_LOCALIZATION_ML_ENABLE              "loc/ml/enable"
#define SERVICE_LOCALIZATION_HR_ENABLE              "loc/hr/enable"
#define SERVICE_LOCALIZATION_PL_ENABLE              "loc/pl/enable"

// Localization manager

#define TOPIC_LOCALIZATION_MANAGER_STATE            "loc/manager/state"
#define ACTION_LOCALIZATION_MANAGER_LOCALIZATION    "loc/manager/localization"
#define SERVICE_LOCALIZATION_MANAGER_SET_STATE      "loc/manager/set_state"
#define SERVICE_LOCALIZATION_MANAGER_GET_PIPELINES  "loc/manager/get_pipelines"
#define SERVICE_LOCALIZATION_MANAGER_GET_CURR_PIPELINE "loc/manager/get_curr_pipeline"

// Poses and camera info as output by the simulator
#define TOPIC_NAV_CAM_SIM_POSE                     "sim/nav_cam/pose"
#define TOPIC_NAV_CAM_SIM_INFO                     "sim/nav_cam/info"
#define TOPIC_HAZ_CAM_SIM_POSE                     "sim/haz_cam/pose"
#define TOPIC_HAZ_CAM_SIM_INFO                     "sim/haz_cam/info"
#define TOPIC_SCI_CAM_SIM_POSE                     "sim/sci_cam/pose"
#define TOPIC_SCI_CAM_SIM_INFO                     "sim/sci_cam/info"

///////////////
// BEHAVIORS //
///////////////

#define SUBSYSTEM_BEHAVIORS                         "beh"

#define NODE_ARM                                    "arm"
#define NODE_DOCK                                   "dock"
#define NODE_PERCH                                  "perch"

#define ACTION_BEHAVIORS_ARM                        "beh/arm"
#define ACTION_BEHAVIORS_DOCK                       "beh/dock"
#define ACTION_BEHAVIORS_PERCH                      "beh/perch"

#define TOPIC_BEHAVIORS_DOCKING_STATE               "beh/dock/state"
#define TOPIC_BEHAVIORS_PERCHING_STATE              "beh/perch/state"

#define TOPIC_BEHAVIORS_ARM_STATE                   "beh/arm/state"
#define TOPIC_BEHAVIORS_ARM_ARM_STATE               "beh/arm/arm_state"
#define TOPIC_BEHAVIORS_ARM_JOINT_SAMPLE            "beh/arm/joint_sample"

#define SERVICE_BEHAVIORS_ARM_SET_STATE             "beh/arm/set_state"
#define SERVICE_BEHAVIORS_DOCK_SET_STATE            "beh/dock/set_state"
#define SERVICE_BEHAVIORS_PERCH_SET_STATE           "beh/perch/set_state"

//////////////
// HARDWARE //
//////////////

#define SUBSYSTEM_HARDWARE                          "hw"

#define NODE_FRAMESTORE                             "framestore"
#define NODE_EPSON_IMU                              "epson_imu"
#define NODE_EPS_DRIVER                             "eps_driver"
#define NODE_PMC_ACTUATOR                           "pmc_actuator"
#define NODE_LASER                                  "node_laser"
#define NODE_FLASHLIGHTS                            "flashlights"
#define NODE_PERCHING_ARM                           "perching_arm"
#define NODE_SPEED_CAM                              "speed_cam"
#define NODE_TEMP_MONITOR                           "temp_monitor"
#define NODE_NAV_CAM                                "nav_cam"
#define NODE_DOCK_CAM                               "dock_cam"
#define NODE_PERCH_CAM                              "perch_cam"
#define NODE_HAZ_CAM                                "haz_cam"
#define NODE_SIGNAL_LIGHTS                          "signal_lights"
#define NODE_VIVE                                   "vive"

#define TOPIC_HARDWARE_PMC_COMMAND                  "hw/pmc/command"
#define TOPIC_HARDWARE_PMC_TELEMETRY                "hw/pmc/telemetry"
#define TOPIC_HARDWARE_PMC_STATE                    "hw/pmc/state"
#define TOPIC_HARDWARE_IMU                          "hw/imu"
#define TOPIC_HARDWARE_NAV_CAM                      "hw/cam_nav"
#define TOPIC_HARDWARE_DOCK_CAM                     "hw/cam_dock"
#define TOPIC_HARDWARE_SCI_CAM                      "hw/cam_sci"
#define TOPIC_HARDWARE_LIGHT_FRONT                  "hw/light_front"
#define TOPIC_HARDWARE_LIGHT_AFT                    "hw/light_aft"
#define TOPIC_HARDWARE_LASER                        "hw/laser"
#define TOPIC_HARDWARE_LASER_RVIZ                   "hw/laser/rviz"
#define TOPIC_HARDWARE_LIGHTS_RVIZ                  "hw/lights/rviz"
#define TOPIC_HARDWARE_EPS_HOUSEKEEPING             "hw/eps/housekeeping"
#define TOPIC_HARDWARE_EPS_POWER_STATE              "hw/eps/power_state"
#define TOPIC_HARDWARE_EPS_BATTERY_STATE_TL         "hw/eps/battery/top_left/state"
#define TOPIC_HARDWARE_EPS_BATTERY_STATE_TR         "hw/eps/battery/top_right/state"
#define TOPIC_HARDWARE_EPS_BATTERY_STATE_BL         "hw/eps/battery/bottom_left/state"
#define TOPIC_HARDWARE_EPS_BATTERY_STATE_BR         "hw/eps/battery/bottom_right/state"
#define TOPIC_HARDWARE_EPS_BATTERY_TEMP_TL          "hw/eps/battery/top_left/temp"
#define TOPIC_HARDWARE_EPS_BATTERY_TEMP_TR          "hw/eps/battery/top_right/temp"
#define TOPIC_HARDWARE_EPS_BATTERY_TEMP_BL          "hw/eps/battery/bottom_left/temp"
#define TOPIC_HARDWARE_EPS_BATTERY_TEMP_BR          "hw/eps/battery/bottom_right/temp"
#define TOPIC_HARDWARE_EPS_DOCK_STATE               "hw/eps/dock"
#define TOPIC_HARDWARE_PICOFLEXX_PREFIX             "hw/depth_"
#define TOPIC_HARDWARE_PICOFLEXX_SUFFIX             "/points"
#define TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED    "/extended"
#define TOPIC_HARDWARE_PICOFLEXX_SUFFIX_DEPTH_IMAGE "/depth_image"
#define TOPIC_HARDWARE_NAME_HAZ_CAM                 "haz"
#define TOPIC_HARDWARE_NAME_PERCH_CAM               "perch"
#define TOPIC_HARDWARE_SPEED_CAM_CAMERA_IMAGE       "hw/speed_cam/camera_image"
#define TOPIC_HARDWARE_SPEED_CAM_OPTICAL_FLOW       "hw/speed_cam/optical_flow"
#define TOPIC_HARDWARE_SPEED_CAM_IMU                "hw/speed_cam/imu"
#define TOPIC_HARDWARE_SPEED_CAM_SPEED              "hw/speed_cam/speed"
#define TOPIC_HARDWARE_TEMP_MONITOR_PREFIX          "hw/temp_monitor/"
#define TOPIC_HARDWARE_DOCK_STATE                   "hw/dock/state"
#define TOPIC_HARDWARE_SIGNAL_LIGHTS_TELEMETRY      "hw/sig/telemetry"
#define TOPIC_HARDWARE_SIGNAL_LIGHTS_CONFIG         "hw/sig/config"
#define TOPIC_HARDWARE_SIGNAL_LIGHTS_STATE          "hw/sig/state"
#define TOPIC_HARDWARE_VIVE_BUTTON                  "hw/vive/buttons"
#define TOPIC_HARDWARE_VIVE_IMU                     "hw/vive/imu"
#define TOPIC_HARDWARE_VIVE_LIGHT                   "hw/vive/light"
#define TOPIC_HARDWARE_VIVE_TRACKERS                "hw/vive/trackers"
#define TOPIC_HARDWARE_VIVE_LIGHTHOUSES             "hw/vive/lighthouses"

#define SERVICE_HARDWARE_EPS_RESET                  "hw/eps/reset"
#define SERVICE_HARDWARE_EPS_CONF_LED_STATE         "hw/eps/configure_led_state"
#define SERVICE_HARDWARE_EPS_CONF_PAYLOAD_POWER     "hw/eps/configure_payload_power"
#define SERVICE_HARDWARE_EPS_CONF_ADVANCED_POWER    "hw/eps/configure_advanced_power"
#define SERVICE_HARDWARE_EPS_RING_BUZZER            "hw/eps/ring_buzzer"
#define SERVICE_HARDWARE_EPS_ENABLE_PMCS            "hw/eps/enable_pmcs"
#define SERVICE_HARDWARE_EPS_GET_BATTERY_STATUS     "hw/eps/get_battery_status"
#define SERVICE_HARDWARE_EPS_GET_TEMPERATURES       "hw/eps/get_temperatures"
#define SERVICE_HARDWARE_EPS_UNDOCK                 "hw/eps/undock"
#define SERVICE_HARDWARE_EPS_GET_BOARD_INFO         "hw/eps/get_board_info"
#define SERVICE_HARDWARE_EPS_CLEAR_TERMINATE        "hw/eps/clear_terminate"

#define SERVICE_HARDWARE_PERCHING_ARM_DIST_VEL      "hw/arm/set_dist_vel"
#define SERVICE_HARDWARE_PERCHING_ARM_PROX_VEL      "hw/arm/set_prox_vel"
#define SERVICE_HARDWARE_PERCHING_ARM_PROX_SERVO    "hw/arm/enable_proximal_servo"
#define SERVICE_HARDWARE_PERCHING_ARM_DIST_SERVO    "hw/arm/enable_distal_servo"
#define SERVICE_HARDWARE_PERCHING_ARM_GRIP_SERVO    "hw/arm/enable_gripper_servo"
#define SERVICE_HARDWARE_PERCHING_ARM_CALIBRATE     "hw/arm/calibrate_gripper"
#define SERVICE_HARDWARE_LIGHT_FRONT_CONTROL        "hw/light_front/control"
#define SERVICE_HARDWARE_LIGHT_AFT_CONTROL          "hw/light_aft/control"
#define SERVICE_HARDWARE_LASER_ENABLE               "hw/laser/enable"
#define SERVICE_HARDWARE_PMC_ENABLE                 "hw/pmc/enable"
#define SERVICE_HARDWARE_PMC_TIMEOUT                 "hw/pmc/set_timeout"

#define SERVICE_STREAMING_LIGHTS                    "hw/signal_lights/streaming"

#endif  // FF_UTIL_FF_NAMES_H_
