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

#ifndef LOCALIZATION_ANALYSIS_MAP_MATCHER_H_
#define LOCALIZATION_ANALYSIS_MAP_MATCHER_H_

#include <ff_msgs/VisualLandmarks.h>
#include <localization_node/localization.h>
#include <localization_common/averager.h>
#include <sparse_mapping/sparse_map.h>

#include <rosbag/view.h>

#include <gtsam/geometry/Pose3.h>

#include <string>

namespace localization_analysis {
class MapMatcher {
 public:
  MapMatcher(const std::string& input_bag_name, const std::string& map_file, const std::string& image_topic,
             const std::string& output_bag_name, const std::string& config_prefix = "",
             const std::string& save_noloc_imgs = "");
  void AddMapMatches();
  void LogResults();

 private:
  bool GenerateVLFeatures(const sensor_msgs::ImageConstPtr& image_msg, ff_msgs::VisualLandmarks& vl_features);

  rosbag::Bag input_bag_;
  rosbag::Bag output_bag_;
  rosbag::Bag nonloc_bag_;
  std::string image_topic_;
  sparse_mapping::SparseMap map_;
  localization_node::Localizer map_feature_matcher_;
  std::string config_prefix_;
  gtsam::Pose3 body_T_nav_cam_;
  localization_common::Averager feature_averager_;
  int sparse_mapping_min_num_landmarks_;
  int match_count_;
  int image_count_;
};
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_MAP_MATCHER_H_
