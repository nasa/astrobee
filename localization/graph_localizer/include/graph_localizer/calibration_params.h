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
#ifndef GRAPH_LOCALIZER_CALIBRATION_PARAMS_H_
#define GRAPH_LOCALIZER_CALIBRATION_PARAMS_H_

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

namespace graph_localizer {
struct CalibrationParams {
  gtsam::Pose3 body_T_dock_cam;
  gtsam::Pose3 body_T_nav_cam;
  gtsam::Pose3 body_T_haz_cam;
  gtsam::Pose3 world_T_dock;
  boost::shared_ptr<gtsam::Cal3_S2> nav_cam_intrinsics;
  boost::shared_ptr<gtsam::Cal3_S2> dock_cam_intrinsics;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_CALIBRATION_PARAMS_H_
