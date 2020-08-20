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
#include <ff_util/ff_names.h>
#include <graph_localizer/graph_localizer_wrapper.h>
#include <lk_optical_flow/lk_optical_flow.h>
#include <localization_node/localization.h>
#include <sparse_mapping/sparse_map.h>

#include <Eigen/Core>

#include <rosbag/bag.h>

#include <memory>
#include <string>
#include <vector>

namespace graph_bag {
// Reads through a bag file and passes relevant messages to graph localizer
// wrapper.  Contains its own instances of sensor parsers (lk_optical_flow,
// localizer (for sparse map matching)) and passes output to graph localizer
// wrapper so this does not require a ROS core and can parse bags more quickly.
class GraphBag {
 public:
  GraphBag(const std::string& bag_name, const std::string& map_file, const std::string& image_topic,
           const std::string& results_bag);
  void Run();

 private:
  void InitializeGraph();
  ff_msgs::Feature2dArray GenerateOFFeatures(const sensor_msgs::ImageConstPtr& image_msg);
  bool GenerateVLFeatures(const sensor_msgs::ImageConstPtr& image_msg, ff_msgs::VisualLandmarks& vl_features);
  void SaveOpticalFlowTracksImage(const sensor_msgs::ImageConstPtr& image_msg,
                                  const graph_localizer::FeatureTrackMap* const feature_tracks);
  void SaveGroundtruthPose(const ff_msgs::VisualLandmarks& vl_features);
  void SavePose(const geometry_msgs::PoseWithCovarianceStamped& latest_pose_msg);
  void FeatureTrackImage(const graph_localizer::FeatureTrackMap& feature_tracks, cv::Mat& feature_track_image) const;

  rosbag::Bag bag_;
  rosbag::Bag results_bag_;
  graph_localizer::GraphLocalizerWrapper graph_localizer_wrapper_;
  lk_optical_flow::LKOpticalFlow optical_flow_tracker_;
  sparse_mapping::SparseMap map_;
  localization_node::Localizer map_feature_matcher_;
  const std::string kImageTopic_;
  const std::string kSparseMappingPoseTopic_ = "sparse_mapping_pose";
  const std::string kGraphLocalizationPoseTopic_ = "graph_localization_pose";
  const std::string kFeatureTracksImageTopic_ = "feature_track_image";
  std::unique_ptr<camera::CameraParameters> nav_cam_params_;
  Eigen::Isometry3d body_T_nav_cam_;
};
}  // end namespace graph_bag

#endif  // GRAPH_BAG_GRAPH_BAG_H_
