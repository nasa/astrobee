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

#ifndef GRAPH_LOCALIZER_HUNGARIAN_ASSIGNER_H_
#define GRAPH_LOCALIZER_HUNGARIAN_ASSIGNER_H_

#include <localization_measurements/semantic_dets_measurement.h>
#include <localization_common/utilities.h>
#include <map>

namespace graph_localizer {
class HungarianAssigner {
public:
  HungarianAssigner(const gtsam::Pose3& body_T_cam, const gtsam::Cal3_S2& cam_intrinsics);
  void assign(const Eigen::Isometry3d& world_T_body, const localization_measurements::SemanticDets &dets);
private:
  gtsam::Pose3 body_T_cam_;
  gtsam::Cal3_S2 cam_intrinsics_;
  std::map<int, std::vector<Eigen::Isometry3d>> object_poses_; // map from class to vector of positions
};
} // namespace graph_localizer

#endif // GRAPH_LOCALIZER_HUNGARIAN_ASSIGNER_H_
