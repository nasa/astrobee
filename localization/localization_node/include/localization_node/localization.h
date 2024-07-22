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

#ifndef LOCALIZATION_NODE_LOCALIZATION_H_
#define LOCALIZATION_NODE_LOCALIZATION_H_

#include <sparse_mapping/sparse_map.h>

#include <config_reader/config_reader.h>
#include <cv_bridge/cv_bridge.h>
#include <ff_msgs/VisualLandmarks.h>
#include <localization_common/timer.h>
#include <sensor_msgs/PointCloud2.h>

#include <deque>

namespace localization_node {

struct ThresholdParams {
  int success_history_size;
  double min_success_rate;
  double max_success_rate;
  int min_features;
  int max_features;
  bool adjust_num_similar;
  int min_num_similar;
  int max_num_similar;
};

class Localizer {
 public:
  explicit Localizer(sparse_mapping::SparseMap* map);
  bool ReadParams(config_reader::ConfigReader& config, bool fatal_failure = true);
  bool Localize(cv_bridge::CvImageConstPtr image_ptr, ff_msgs::VisualLandmarks* vl,
     Eigen::Matrix2Xd* image_keypoints = NULL);
  const camera::CameraParameters& camera_params() const { return *cam_params_; }
 private:
  void AdjustThresholds();

  sparse_mapping::SparseMap* map_;
  std::shared_ptr<camera::CameraParameters> cam_params_;
  // Success params for adjusting keypoint thresholds
  std::deque<int> successes_;
  ThresholdParams params_;
  localization_common::Timer timer_ = localization_common::Timer("VL Runtime");
};

};  // namespace localization_node

#endif  // LOCALIZATION_NODE_LOCALIZATION_H_
