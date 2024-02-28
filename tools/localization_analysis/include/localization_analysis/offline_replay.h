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

#ifndef LOCALIZATION_ANALYSIS_OFFLINE_REPLAY_H_
#define LOCALIZATION_ANALYSIS_OFFLINE_REPLAY_H_

#include <camera/camera_params.h>
#include <localization_analysis/graph_localizer_simulator.h>
#include <localization_analysis/graph_vio_simulator.h>
#include <localization_analysis/offline_replay_params.h>
// #include <imu_bias_tester/imu_bias_tester_wrapper.h>
#include <localization_analysis/live_measurement_simulator.h>
#include <ros_pose_extrapolator/ros_pose_extrapolator_wrapper.h>

#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>
#include <vector>

namespace localization_analysis {
// Reads through a bag file and passes relevant messages to graph localizer
// and VIO wrappers.  Uses the LiveMeasurementSimulator which contains its own instances of sensor parsers
// (lk_optical_flow, localizer (for sparse map matching)) with optional measurement delays and passes the output of
// these sensor parses to the respective graph objects. This tool avoids using rosbag play and ROS core and enables
// parsing bags more quickly. Saves output to a new bagfile.
class OfflineReplay {
 public:
  OfflineReplay(const std::string& bag_name, const std::string& map_file, const std::string& image_topic,
                const std::string& results_bag, const std::string& output_stats_file,
                const bool use_bag_image_feature_msgs = true, const std::string& graph_config_path_prefix = "");
  void Run();

 private:
  void InitializeGraphs();
  /*void SaveOpticalFlowTracksImage(const sensor_msgs::ImageConstPtr& image_msg,
                                  const GraphLocalizerSimulator& graph_localizer);*/
  std::unique_ptr<GraphLocalizerSimulator> graph_localizer_simulator_;
  std::unique_ptr<GraphVIOSimulator> graph_vio_simulator_;
  std::unique_ptr<LiveMeasurementSimulator> live_measurement_simulator_;
  rosbag::Bag results_bag_;
  // imu_bias_tester::ImuBiasTesterWrapper imu_bias_tester_wrapper_;
  ros_pose_extrapolator::RosPoseExtrapolatorWrapper pose_extrapolator_wrapper_;
  std::string output_stats_file_;
  const std::string kFeatureTracksImageTopic_ = "feature_track_image";
  boost::optional<ff_msgs::VisualLandmarks> latest_ar_msg_;
  OfflineReplayParams params_;
};
}  // end namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_OFFLINE_REPLAY_H_
