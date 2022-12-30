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
#ifndef GRAPH_VIO_GRAPH_VIO_NODELET_H_
#define GRAPH_VIO_GRAPH_VIO_NODELET_H_

#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_msgs/SetEkfInput.h>
#include <ff_msgs/VisualLandmarks.h>
#include <ff_util/ff_nodelet.h>
#include <graph_vio/graph_vio_nodelet_params.h>
#include <graph_vio/graph_vio_wrapper.h>
#include <localization_common/ros_timer.h>
#include <localization_common/timer.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <vector>

namespace graph_vio {
class GraphVIONodelet : public ff_util::FreeFlyerNodelet {
 public:
  GraphVIONodelet();

 private:
  void Initialize(ros::NodeHandle* nh) final;

  bool SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res);

  void DisableVIO();

  void EnableVIO();

  bool vio_enabled() const;

  bool ResetMap(ff_msgs::ResetMap::Request& req, ff_msgs::ResetMap::Response& res);

  bool ResetBiasesAndVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool ResetBiasesFromFileAndResetVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool ResetBiasesFromFileAndResetVIO();

  bool ResetVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void ResetAndEnableVIO();

  void SubscribeAndAdvertise(ros::NodeHandle* nh);

  void InitializeGraph();

  void PublishVIOState();

  void PublishVIOGraph();

  void PublishReset() const;

  void PublishGraphMessages();

  void PublishHeartbeat();

  void OpticalFlowCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg);

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

  void FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode);

  void Run();

  graph_vio::GraphVIOWrapper graph_vio_wrapper_;
  ros::NodeHandle private_nh_;
  ros::CallbackQueue private_queue_;
  bool vio_enabled_ = true;
  ros::Subscriber imu_sub_, of_sub_, flight_mode_sub_;
  ros::Publisher state_pub_, graph_pub_, reset_pub_, heartbeat_pub_;
  ros::ServiceServer reset_srv_, bias_srv_, bias_from_file_srv_, input_mode_srv_;
  tf2_ros::TransformBroadcaster transform_pub_;
  std::string platform_name_;
  ff_msgs::Heartbeat heartbeat_;
  GraphVIONodeletParams params_;
  int last_mode_ = -1;

  ros::Time last_heartbeat_time_;

  // Timers
  localization_common::RosTimer of_timer_ = localization_common::RosTimer("OF msg");
  localization_common::RosTimer imu_timer_ = localization_common::RosTimer("Imu msg");
  localization_common::Timer callbacks_timer_ = localization_common::Timer("Callbacks");
  localization_common::Timer nodelet_runtime_timer_ = localization_common::Timer("Nodelet Runtime");
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_GRAPH_VIO_NODELET_H_
