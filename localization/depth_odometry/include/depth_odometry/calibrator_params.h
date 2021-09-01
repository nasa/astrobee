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
#ifndef DEPTH_ODOMETRY_CALIBRATOR_PARAMS_H_
#define DEPTH_ODOMETRY_CALIBRATOR_PARAMS_H_

#include <camera/camera_params.h>

#include <memory>

namespace depth_odometry {
struct CalibratorParams {
  bool calibrate_intrinsics;
  bool calibrate_depth_image_A_depth_haz;
  bool calibrate_distortion;
  // Optimization Options
  int max_num_iterations;
  double function_tolerance;
  // Other
  int max_num_match_sets;
  std::shared_ptr<camera::CameraParameters> camera_params;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_CALIBRATOR_PARAMS_H_
