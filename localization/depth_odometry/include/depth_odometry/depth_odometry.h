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
#ifndef DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_
#define DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_

#include <depth_odometry/depth_odometry_params.h>
#include <depth_odometry/pose_with_covariance_and_matches.h>
#include <localization_common/time.h>
#include <localization_measurements/depth_image_measurement.h>

#include <boost/optional.hpp>

namespace depth_odometry {
class DepthOdometry {
 public:
  explicit virtual DepthOdometry(const DepthOdometryParams& params) = 0;
  virtual boost::optional<PoseWithCovarianceAndMatches> DepthImageCallback(
    const localization_measurements::DepthImageMeasurement& depth_image) = 0;
  const DepthOdometryParams& params() const { return params_; }

  // TODO(rsoussan): move this to utils
  bool CovarianceSane(const Eigen::Matrix<double, 6, 6>& covariance) const {
    const auto position_covariance_norm = covariance.block<3, 3>(0, 0).diagonal().norm();
    const auto orientation_covariance_norm = covariance.block<3, 3>(3, 3).diagonal().norm();
    return (position_covariance_norm <= params_.position_covariance_threshold &&
            orientation_covariance_norm <= params_.orientation_covariance_threshold);
  }

  DepthOdometryParams params_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_
