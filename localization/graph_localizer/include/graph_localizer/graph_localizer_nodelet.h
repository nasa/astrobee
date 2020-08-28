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
#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_NODELET_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_NODELET_H_

#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/SetEkfInput.h>
#include <ff_msgs/VisualLandmarks.h>
#include <ff_util/ff_nodelet.h>
#include <graph_localizer/graph_localizer_wrapper.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <string>
#include <vector>

namespace graph_localizer {
class GraphLocalizerNodelet : public ff_util::FreeFlyerNodelet {
 public:
  GraphLocalizerNodelet();

  void Run();

 private:
  void Initialize(ros::NodeHandle* nh) final;

  bool SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res);

  void DisableLocalizer();

  void EnableLocalizer();

  bool localizer_enabled() const;

  bool ResetBiasesAndLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool ResetLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void ResetAndEnableLocalizer();

  void SubscribeAndAdvertise(ros::NodeHandle* nh);

  void InitializeGraph();

  void PublishLocalizationState();

  void PublishLocalizationGraph();

  void PublishPose() const;

  void OpticalFlowCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg);

  void VLVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg);

  void ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg);

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

  graph_localizer::GraphLocalizerWrapper graph_localizer_wrapper_;
  bool localizer_enabled_ = true;
  ros::Subscriber imu_sub_, of_sub_, vl_sub_, ar_sub_;
  ros::Publisher state_pub_, graph_pub_, pose_pub_;
  ros::ServiceServer reset_srv_, bias_srv_, input_mode_srv_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_NODELET_H_
