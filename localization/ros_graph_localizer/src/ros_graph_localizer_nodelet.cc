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

#include <ff_common/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>
#include <ros_graph_localizer/parameter_reader.h>
#include <ros_graph_localizer/ros_graph_localizer_nodelet.h>

#include <std_msgs/Empty.h>

namespace ros_graph_localizer {
namespace lc = localization_common;
namespace mc = msg_conversions;
namespace rv = ros_graph_vio;

RosGraphLocalizerNodelet::RosGraphLocalizerNodelet() : ff_util::FreeFlyerNodelet(NODE_GRAPH_LOC, true) {
  private_nh_.setCallbackQueue(&private_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();

  config_reader::ConfigReader config;
  config.AddFile("localization/ros_graph_localizer.config");
  lc::LoadGraphLocalizerConfig(config);
  LoadRosGraphLocalizerNodeletParams(config, params_);
  last_heartbeat_time_ = ros::Time::now();
}

void RosGraphLocalizerNodelet::Initialize(ros::NodeHandle* nh) {
  // Setup the platform name
  platform_name_ = GetPlatform();
  platform_name_ = (platform_name_.empty() ? "" : platform_name_ + "/");

  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
  Run();
}

void RosGraphLocalizerNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  graph_vio_state_pub_ = nh->advertise<ff_msgs::GraphVIOState>(TOPIC_GRAPH_VIO_STATE, 10);
  graph_loc_pub_ = nh->advertise<ff_msgs::GraphLocState>(TOPIC_GRAPH_LOC_STATE, 10);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 10);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);
  if (params_.publish_depth_odometry) {
    depth_odom_pub_ = nh->advertise<ff_msgs::DepthOdometry>(TOPIC_LOCALIZATION_DEPTH_ODOM, 10);
  }
  imu_sub_ = private_nh_.subscribe(TOPIC_HARDWARE_IMU, params_.max_imu_buffer_size,
                                   &RosGraphLocalizerNodelet::ImuCallback, this, ros::TransportHints().tcpNoDelay());

  if (params_.subscribe_to_depth_odometry) {
    depth_odom_sub_ =
      private_nh_.subscribe(TOPIC_LOCALIZATION_DEPTH_ODOM, params_.max_depth_odom_buffer_size,
                            &RosGraphLocalizerNodelet::DepthOdometryCallback, this, ros::TransportHints().tcpNoDelay());
  }

  if (params_.run_depth_odometry) {
    const std::string depth_point_cloud_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                                static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                                static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX);
    depth_point_cloud_sub_ = private_nh_.subscribe<sensor_msgs::PointCloud2>(
      depth_point_cloud_topic, params_.max_depth_cloud_buffer_size, &RosGraphLocalizerNodelet::DepthPointCloudCallback,
      this, ros::TransportHints().tcpNoDelay());
    image_transport::ImageTransport depth_image_transport(private_nh_);
    const std::string depth_image_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                          static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                          static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED) +
                                          static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_AMPLITUDE_IMAGE);
    depth_image_sub_ = depth_image_transport.subscribe(depth_image_topic, params_.max_depth_image_buffer_size,
                                                       &RosGraphLocalizerNodelet::DepthImageCallback, this);
  }

  fp_sub_ =
    private_nh_.subscribe(TOPIC_LOCALIZATION_OF_FEATURES, params_.max_feature_point_buffer_size,
                          &RosGraphLocalizerNodelet::FeaturePointsCallback, this, ros::TransportHints().tcpNoDelay());
  flight_mode_sub_ =
    private_nh_.subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 10, &RosGraphLocalizerNodelet::FlightModeCallback, this);
  ar_tag_vl_sub_ = private_nh_.subscribe(
    TOPIC_LOCALIZATION_AR_FEATURES, params_.max_ar_tag_matched_projections_buffer_size,
    &RosGraphLocalizerNodelet::ARVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  sparse_map_vl_sub_ = private_nh_.subscribe(
    TOPIC_LOCALIZATION_ML_FEATURES, params_.max_vl_matched_projections_buffer_size,
    &RosGraphLocalizerNodelet::SparseMapVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  bias_srv_ =
    private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &RosGraphLocalizerNodelet::ResetBiasesAndLocalizer, this);
  bias_from_file_srv_ = private_nh_.advertiseService(
    SERVICE_GNC_EKF_INIT_BIAS_FROM_FILE, &RosGraphLocalizerNodelet::ResetBiasesFromFileAndResetLocalizer, this);
  reset_map_srv_ =
    private_nh_.advertiseService(SERVICE_LOCALIZATION_RESET_MAP_LOC, &RosGraphLocalizerNodelet::ResetMap, this);
  reset_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_RESET,
                                            &RosGraphLocalizerNodelet::ResetBiasesFromFileAndResetLocalizer, this);
  input_mode_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_SET_INPUT, &RosGraphLocalizerNodelet::SetMode, this);
}

bool RosGraphLocalizerNodelet::SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {
  const auto input_mode = req.mode;
  if (input_mode == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LogInfo("Received Mode None request, turning off Localizer.");
    DisableLocalizerAndVIO();
  } else if (last_mode_ == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LogInfo(
      "Received Mode request that is not None and current mode is "
      "None, resetting Localizer.");
    ResetAndEnableLocalizer();
  }

  // Reset localizer when switch between ar mode
  if (input_mode == ff_msgs::SetEkfInputRequest::MODE_AR_TAGS &&
      last_mode_ != ff_msgs::SetEkfInputRequest::MODE_AR_TAGS) {
    LogInfo("SetMode: Switching to AR_TAG mode.");
    // Reset world_T_dock when switch back to ar mode
    ros_graph_localizer_wrapper_.ResetWorldTDock();
  }

  last_mode_ = input_mode;
  return true;
}

void RosGraphLocalizerNodelet::DisableLocalizerAndVIO() { localizer_and_vio_enabled_ = false; }

void RosGraphLocalizerNodelet::EnableLocalizerAndVIO() { localizer_and_vio_enabled_ = true; }

bool RosGraphLocalizerNodelet::localizer_and_vio_enabled() const { return localizer_and_vio_enabled_; }

bool RosGraphLocalizerNodelet::ResetBiasesAndLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  DisableLocalizerAndVIO();
  ros_graph_vio_wrapper_.ResetBiasesAndVIO();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizerAndVIO();
  return true;
}

bool RosGraphLocalizerNodelet::ResetBiasesFromFileAndResetLocalizer(std_srvs::Empty::Request& req,
                                                                    std_srvs::Empty::Response& res) {
  return ResetBiasesFromFileAndResetLocalizer();
}

bool RosGraphLocalizerNodelet::ResetBiasesFromFileAndResetLocalizer() {
  DisableLocalizerAndVIO();
  ros_graph_vio_wrapper_.ResetBiasesFromFileAndResetVIO();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizerAndVIO();
  return true;
}

void RosGraphLocalizerNodelet::ResetAndEnableLocalizer() {
  DisableLocalizerAndVIO();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizerAndVIO();
}

bool RosGraphLocalizerNodelet::ResetMap(ff_msgs::ResetMap::Request& req, ff_msgs::ResetMap::Response& res) {
  // Clear sparse map measurement buffer
  sparse_map_vl_sub_ = private_nh_.subscribe(
    TOPIC_LOCALIZATION_ML_FEATURES, params_.max_vl_matched_projections_buffer_size,
    &RosGraphLocalizerNodelet::SparseMapVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  ResetAndEnableLocalizer();
  return true;
}

void RosGraphLocalizerNodelet::PublishGraphLocalizerState() {
  const auto msg = ros_graph_localizer_wrapper_.GraphLocStateMsg();
  if (msg) graph_loc_pub_.publish(*msg);
}

void RosGraphLocalizerNodelet::FeaturePointsCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg) {
  if (!localizer_and_vio_enabled()) return;
  ros_graph_vio_wrapper_.FeaturePointsCallback(*feature_array_msg);
}

void RosGraphLocalizerNodelet::DepthPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  if (!localizer_and_vio_enabled()) return;
  const auto depth_odometry_msgs = depth_odometry_wrapper_.PointCloudCallback(point_cloud_msg);
  for (const auto& depth_odometry_msg : depth_odometry_msgs) {
    ros_graph_vio_wrapper_.DepthOdometryCallback(depth_odometry_msg);
    if (params_.publish_depth_odometry) {
      depth_odom_pub_.publish(depth_odometry_msg);
    }
  }
}

void RosGraphLocalizerNodelet::DepthImageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  if (!localizer_and_vio_enabled()) return;
  const auto depth_odometry_msgs = depth_odometry_wrapper_.ImageCallback(image_msg);
  for (const auto& depth_odometry_msg : depth_odometry_msgs) {
    ros_graph_vio_wrapper_.DepthOdometryCallback(depth_odometry_msg);
    if (params_.publish_depth_odometry) {
      depth_odom_pub_.publish(depth_odometry_msg);
    }
  }
}

void RosGraphLocalizerNodelet::DepthOdometryCallback(const ff_msgs::DepthOdometry::ConstPtr& depth_odom_msg) {
  if (!localizer_and_vio_enabled()) return;
  ros_graph_vio_wrapper_.DepthOdometryCallback(*depth_odom_msg);
}

void RosGraphLocalizerNodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  if (!localizer_and_vio_enabled()) return;
  ros_graph_vio_wrapper_.ImuCallback(*imu_msg);
  ros_graph_localizer_wrapper_.ImuCallback(*imu_msg);
}

void RosGraphLocalizerNodelet::FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode) {
  ros_graph_vio_wrapper_.FlightModeCallback(*mode);
  ros_graph_localizer_wrapper_.FlightModeCallback(*mode);
}

void RosGraphLocalizerNodelet::PublishReset() const {
  std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void RosGraphLocalizerNodelet::PublishHeartbeat() {
  heartbeat_.header.stamp = ros::Time::now();
  if ((heartbeat_.header.stamp - last_heartbeat_time_).toSec() < 1.0) return;
  heartbeat_pub_.publish(heartbeat_);
  last_heartbeat_time_ = heartbeat_.header.stamp;
}

void RosGraphLocalizerNodelet::PublishGraphLocalizerMessages() {
  if (!localizer_and_vio_enabled()) return;
  PublishGraphLocalizerState();
}

void RosGraphLocalizerNodelet::PublishWorldTDockTF() {
  const auto world_T_dock = ros_graph_localizer_wrapper_.WorldTDock();
  if (!world_T_dock) return;
  const auto world_T_dock_tf =
    lc::PoseToTF(*world_T_dock, "world", "dock/body", lc::TimeFromRosTime(ros::Time::now()), platform_name_);
  // If the rate is higher than the sim time, prevent sending repeat tfs
  if (world_T_dock_tf.header.stamp == last_tf_dock_time_) return;
  last_tf_dock_time_ = world_T_dock_tf.header.stamp;
  transform_pub_.sendTransform(world_T_dock_tf);
}

void RosGraphLocalizerNodelet::ARVisualLandmarksCallback(
  const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_and_vio_enabled()) return;
  ros_graph_localizer_wrapper_.ARVisualLandmarksCallback(*visual_landmarks_msg);
  PublishWorldTDockTF();
}

void RosGraphLocalizerNodelet::SparseMapVisualLandmarksCallback(
  const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_and_vio_enabled()) return;
  // Avoid adding sparse map measurements when in AR mode
  if (last_mode_ == ff_msgs::SetEkfInputRequest::MODE_AR_TAGS) return;
  ros_graph_localizer_wrapper_.SparseMapVisualLandmarksCallback(*visual_landmarks_msg);
}

void RosGraphLocalizerNodelet::Run() {
  ros::Rate rate(100);
  // Load Biases from file by default
  // Biases reestimated if a intialize bias service call is received
  ResetBiasesFromFileAndResetLocalizer();
  while (ros::ok()) {
    private_queue_.callAvailable();
    if (localizer_and_vio_enabled()) {
      ros_graph_vio_wrapper_.Update();
      // Pass pose covariance interpolater used for relative factors
      // from graph vio to graph localizer
      if (ros_graph_vio_wrapper_.Initialized() && ros_graph_localizer_wrapper_.Initialized()) {
        ros_graph_localizer_wrapper_.graph_localizer()->SetPoseCovarianceInterpolater(
          ros_graph_vio_wrapper_.graph_vio()->MarginalsPoseCovarianceInterpolater());
      }

      const auto graph_vio_state_msg = ros_graph_vio_wrapper_.GraphVIOStateMsg();
      if (!graph_vio_state_msg) {
        LogDebugEveryN(100, "PublishVIOState: Failed to get vio state msg.");
      } else {
        graph_vio_state_pub_.publish(*graph_vio_state_msg);
        ros_graph_localizer_wrapper_.GraphVIOStateCallback(*graph_vio_state_msg);
      }
      ros_graph_localizer_wrapper_.Update();
      PublishGraphLocalizerMessages();
    }
    PublishHeartbeat();
    rate.sleep();
  }
}
}  // namespace ros_graph_localizer

PLUGINLIB_EXPORT_CLASS(ros_graph_localizer::RosGraphLocalizerNodelet, nodelet::Nodelet);
