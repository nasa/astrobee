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
#ifndef LOCALIZATION_ANALYSIS_OFFLINE_REPLAY_PARAMS_H_
#define LOCALIZATION_ANALYSIS_OFFLINE_REPLAY_PARAMS_H_

#include <camera/camera_params.h>

#include <gtsam/geometry/Pose3.h>

#include <memory>

namespace localization_analysis {
struct OfflineReplayParams {
  // Save images with optical flow feature tracks to the output bagfile.
  bool save_optical_flow_images;
  // Log the relative time since the start of the bagfile.
  bool log_relative_time;
  std::unique_ptr<camera::CameraParameters> nav_cam_params;
  gtsam::Pose3 body_T_nav_cam;
  gtsam::Pose3 body_T_dock_cam;
  // Min number of ar landmarks for an AR msg to be valid.
  int ar_min_num_landmarks;
  // Min number of sparse mapping landmarks for a Sparse Mapping msg to be valid.
  int sparse_mapping_min_num_landmarks;
};
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_OFFLINE_REPLAY_PARAMS_H_
