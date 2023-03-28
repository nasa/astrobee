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
#ifndef ROS_GRAPH_VIO_ROS_GRAPH_VIO_NODELET_H_
#define ROS_GRAPH_VIO_ROS_GRAPH_VIO_NODELET_H_

#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_msgs/SetEkfInput.h>
#include <ff_util/ff_nodelet.h>
#include <localization_common/ros_timer.h>
#include <localization_common/timer.h>
#include <ros_graph_vio/ros_graph_vio_nodelet_params.h>
#include <ros_graph_vio/ros_graph_vio_wrapper.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <vector>

namespace ros_graph_vio {
class RosGraphVIONodelet : public ff_util::FreeFlyerNodelet {
 public:
  RosGraphVIONodelet();

 private:
  // Subscribes to and advertises topics. Calls Run() to start processing loop.
  void Initialize(ros::NodeHandle* nh) final;

  // Subscribes to and advertises topics.
  void SubscribeAndAdvertise(ros::NodeHandle* nh);

  // Set mode for VIO. Disables if mode is none, resets and enables if swtiched from none.
  bool SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res);

  // Disabled VIO. Prevents any messages from being added to VIO, halts publishing
  // messages from VIO, and halts updating VIO.
  void DisableVIO();

  // Enables VIO.
  void EnableVIO();

  // Whether VIO is enabled.
  bool vio_enabled() const;

  // Resets the graph and biases. Biases need to be estimated again by the bias initializer.
  bool ResetBiasesAndVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // Wrapper for ResetBiasesFromFileAndResetVIO triggered by service call.
  bool ResetBiasesFromFileAndResetVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // Resets the graph and and loads biases from a saved file.
  bool ResetBiasesFromFileAndResetVIO();

  // Wrapper for ResetAndEnableVIO triggered by service call.
  bool ResetVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // Resets the graph with the latest biases.
  void ResetAndEnableVIO();

  // void PublishVIOState();

  // void PublishVIOGraph();

  // Publishes CombinedNavStateArrayMsg using the history
  // of nav states and covariances in graph_vio.
  void PublishGraphVIOStates();

  // Publishes empty reset message.
  void PublishReset() const;

  // Publishes CombinedNavStateArrayMsg and other graph messages if VIO is enabled.
  void PublishGraphMessages();

  // Publishes heartbeat message.
  void PublishHeartbeat();

  // Passes feature points msg to ros_graph_vio_wrapper if VIO is enabled.
  void FeaturePointsCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg);

  // Passes IMU msg to ros_graph_vio_wrapper if VIO is enabled.
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

  // Passes flight mode msg to ros_graph_vio_wrapper if VIO is enabled.
  void FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode);

  // Adds messages to ros_graph_vio_wrapper from callback queue, updates
  // the ros_graph_vio_wrapper, and pubishes messages.
  // Runs iteratively on start up at a max 100Hz rate.
  void Run();

  ros_graph_vio::RosGraphVIOWrapper ros_graph_vio_wrapper_;
  ros::NodeHandle private_nh_;
  ros::CallbackQueue private_queue_;
  bool vio_enabled_ = true;
  ros::Subscriber imu_sub_, fp_sub_, flight_mode_sub_;
  ros::Publisher states_pub_, graph_pub_, reset_pub_, heartbeat_pub_;
  ros::ServiceServer reset_srv_, bias_srv_, bias_from_file_srv_, input_mode_srv_;
  std::string platform_name_;
  ff_msgs::Heartbeat heartbeat_;
  RosGraphVIONodeletParams params_;
  int last_mode_ = -1;

  ros::Time last_heartbeat_time_;
};
}  // namespace ros_graph_vio

#endif  // ROS_GRAPH_VIO_ROS_GRAPH_VIO_NODELET_H_
