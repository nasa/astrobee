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
#ifndef ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_NODELET_H_
#define ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_NODELET_H_

#include <depth_odometry/depth_odometry_wrapper.h>
#include <ff_msgs/GraphVIOState.h>
#include <ff_msgs/VisualLandmarks.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_msgs/ResetMap.h>
#include <ff_msgs/SetEkfInput.h>
#include <ff_util/ff_nodelet.h>
#include <localization_common/ros_timer.h>
#include <localization_common/timer.h>
#include <ros_graph_localizer/ros_graph_localizer_nodelet_params.h>
#include <ros_graph_localizer/ros_graph_localizer_wrapper.h>
#include <ros_graph_vio/ros_graph_vio_wrapper.h>

#include <image_transport/image_transport.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <vector>

namespace ros_graph_localizer {
class RosGraphLocalizerNodelet : public ff_util::FreeFlyerNodelet {
 public:
  RosGraphLocalizerNodelet();

 private:
  // Subscribes to and advertises topics. Calls Run() to start processing loop.
  void Initialize(ros::NodeHandle* nh) final;

  // Subscribes to and advertises topics.
  void SubscribeAndAdvertise(ros::NodeHandle* nh);

  // Set mode for Localizer. Disables if mode is none, resets and enables if swtiched from none.
  bool SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res);

  // Disabled Localizer and VIO. Prevents any messages from being added, halts publishing
  // messages, and halts updating Localizer and VIO.
  void DisableLocalizerAndVIO();

  // Enables Localizer and VIO.
  void EnableLocalizerAndVIO();

  // Whether Localizer and VIO are enabled.
  bool localizer_and_vio_enabled() const;

  // Resets VIO using re-estimation of biases and resets the localizer.
  bool ResetBiasesAndLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // Resets VIO using biases in file (if available) and resets the localizer.
  bool ResetBiasesFromFileAndResetLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // Resets VIO using biases in file (if available) and resets the localizer.
  bool ResetBiasesFromFileAndResetLocalizer();

  // Resets the localizer.
  void ResetAndEnableLocalizer();

  // Resets the localizer and clears the sparse map matched projections buffer
  // so old measurements aren't used with a new map.
  bool ResetMap(ff_msgs::ResetMap::Request& req, ff_msgs::ResetMap::Response& res);

  // Publish latest graph localizer state msg.
  void PublishGraphLocalizerState();

  // Publishes empty reset message.
  void PublishReset() const;

  // Publishes Loc pose message and other graph messages if Localizer is enabled.
  void PublishGraphLocalizerMessages();

  // Publishes heartbeat message.
  void PublishHeartbeat();

  // Publishes world_T_body transform
  void PublishWorldTBodyTF();

  // Publishes world_T_dock transform
  void PublishWorldTDockTF();

  // Passes ar tag visual landmarks msg to ros_graph_localizer_wrapper if Localizer is enabled.
  void ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg);

  // Passes sparse map visual landmarks msg to ros_graph_localizer_wrapper if Localizer is enabled.
  void SparseMapVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg);

  // Passes feature points msg to ros_graph_vio_wrapper if VIO is enabled.
  void FeaturePointsCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg);

  // Passes depth odometry msg to ros_graph_vio_wrapper if VIO is enabled.
  void DepthOdometryCallback(const ff_msgs::DepthOdometry::ConstPtr& depth_odom_msg);

  // Passes depth point cloud msg to depth_odometry_wrapper if VIO is enabled.
  void DepthPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);

  // Passes depth image msg to depth_odometry_wrapper if VIO is enabled.
  void DepthImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  // Passes IMU msg to ros_graph_vio_wrapper if VIO is enabled.
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

  // Passes flight mode msg to ros_graph_vio_wrapper if VIO is enabled.
  void FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode);

  // Adds messages to ros_graph_localizer_wrapper from callback queue, updates
  // the ros_graph_localizer_wrapper, and pubishes messages.
  // Runs iteratively on start up at a max 100Hz rate.
  void Run();

  ros_graph_localizer::RosGraphLocalizerWrapper ros_graph_localizer_wrapper_;
  ros_graph_vio::RosGraphVIOWrapper ros_graph_vio_wrapper_;
  depth_odometry::DepthOdometryWrapper depth_odometry_wrapper_;
  ros::NodeHandle private_nh_;
  ros::CallbackQueue private_queue_;
  bool localizer_and_vio_enabled_ = true;
  ros::Subscriber sparse_map_vl_sub_, ar_tag_vl_sub_;
  ros::Publisher graph_loc_pub_, reset_pub_, heartbeat_pub_;
  tf2_ros::TransformBroadcaster transform_pub_;
  ros::ServiceServer bias_srv_, bias_from_file_srv_, reset_map_srv_, reset_srv_, input_mode_srv_;
  std::string platform_name_;
  ff_msgs::Heartbeat heartbeat_;
  RosGraphLocalizerNodeletParams params_;
  int last_mode_ = -1;

  ros::Time last_heartbeat_time_;
  ros::Time last_tf_body_time_;
  ros::Time last_tf_dock_time_;

  // VIO
  ros::Publisher graph_vio_state_pub_, graph_vio_pub_, depth_odom_pub_;
  ros::Subscriber imu_sub_, depth_point_cloud_sub_, depth_odom_sub_, fp_sub_, flight_mode_sub_;
  image_transport::Subscriber depth_image_sub_;
};
}  // namespace ros_graph_localizer

#endif  // ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_NODELET_H_
