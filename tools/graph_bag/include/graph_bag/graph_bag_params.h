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
#ifndef GRAPH_BAG_GRAPH_BAG_PARAMS_H_
#define GRAPH_BAG_GRAPH_BAG_PARAMS_H_

#include <camera/camera_params.h>

#include <gtsam/geometry/Pose3.h>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <map>

namespace graph_bag {
struct GraphBagParams {
  bool save_optical_flow_images;
  bool save_semantic_matches_images;
  bool show_semantic_matches_images;
  bool log_relative_time;
  std::unique_ptr<camera::CameraParameters> nav_cam_params;
  gtsam::Pose3 body_T_nav_cam;
  int ar_min_num_landmarks;
  int sparse_mapping_min_num_landmarks;

  cv::Mat undist_map_x;
  cv::Mat undist_map_y;

  std::map<int, std::string> class_names;
};
}  // namespace graph_bag

#endif  // GRAPH_BAG_GRAPH_BAG_PARAMS_H_
