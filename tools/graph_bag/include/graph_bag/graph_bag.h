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

#ifndef GRAPH_BAG_GRAPH_BAG_H_
#define GRAPH_BAG_GRAPH_BAG_H_

#include <camera/camera_params.h>
#include <graph_bag/imu_bias_tester_wrapper.h>
#include <graph_bag/live_measurement_simulator.h>
#include <graph_localizer/graph_localizer_wrapper.h>
#include <imu_augmentor/imu_augmentor_wrapper.h>

#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>
#include <vector>

namespace graph_bag {
// Reads through a bag file and passes relevant messages to graph localizer
// wrapper.  Uses LiveMeasurementSimulator which contains its own instances of sensor parsers (lk_optical_flow,
// localizer (for sparse map matching)) and passes output to graph localizer
// wrapper so this does not require a ROS core and can parse bags more quickly. Saves output to a new bagfile.
class GraphBag {
 public:
  GraphBag(const std::string& bag_name, const std::string& map_file, const std::string& image_topic,
           const bool save_feature_track_image, const std::string& results_bag);
  void Run();

 private:
  void InitializeGraph();
  void SaveOpticalFlowTracksImage(const sensor_msgs::ImageConstPtr& image_msg,
                                  const graph_localizer::FeatureTrackMap* const feature_tracks);
  void SaveSparseMappingPoseMsg(const geometry_msgs::PoseStamped& sparse_mapping_pose_msg);

  void SavePose(const geometry_msgs::PoseStamped& latest_pose_msg);
  void SaveImuBiasTesterPredictedStates(
      const std::vector<localization_common::CombinedNavState>& imu_bias_tester_predicted_states);
  void SaveLocState(const ff_msgs::EkfState& loc_msg, const std::string& topic);
  void FeatureTrackImage(const graph_localizer::FeatureTrackMap& feature_tracks, cv::Mat& feature_track_image) const;

  std::unique_ptr<LiveMeasurementSimulator> live_measurement_simulator_;
  rosbag::Bag results_bag_;
  graph_localizer::GraphLocalizerWrapper graph_localizer_wrapper_;
  imu_augmentor::ImuAugmentorWrapper imu_augmentor_wrapper_;
  ImuBiasTesterWrapper imu_bias_tester_wrapper_;
  const bool kSaveFeatureTrackImage_;
  const std::string kFeatureTracksImageTopic_ = "feature_track_image";
  const std::string kImuBiasTesterPoseTopic_ = "imu_bias_tester";
  std::unique_ptr<camera::CameraParameters> nav_cam_params_;
};
}  // end namespace graph_bag

#endif  // GRAPH_BAG_GRAPH_BAG_H_
