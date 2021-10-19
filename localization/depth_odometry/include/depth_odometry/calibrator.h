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
#ifndef DEPTH_ODOMETRY_CALIBRATOR_H_
#define DEPTH_ODOMETRY_CALIBRATOR_H_

#include <depth_odometry/calibrator_params.h>
#include <depth_odometry/depth_matches.h>
#include <depth_odometry/optimization_utilities.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace depth_odometry {
class Calibrator {
 public:
  Calibrator(const CalibratorParams& params) : params_(params) {}
  void Calibrate(const std::vector<DepthMatches>& match_sets, const Eigen::Affine3d& initial_depth_image_A_depth_cloud,
                 const Eigen::Matrix3d& initial_intrinsics, const Eigen::Matrix<double, 4, 1>& intial_distortion,
                 Eigen::Affine3d& calibrated_depth_image_A_depth_cloud, Eigen::Matrix3d& calibrated_intrinsics,
                 Eigen::Matrix<double, 4, 1>& calibrated_distortion);

  const CalibratorParams& params() { return params_; }

 private:
  CalibratorParams params_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_CALIBRATOR_H_
