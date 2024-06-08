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

#ifndef LOCALIZATION_COMMON_DISTANCE_SCALED_POSE_COVARIANCE_INTERPOLATER_H_
#define LOCALIZATION_COMMON_DISTANCE_SCALED_POSE_COVARIANCE_INTERPOLATER_H_

#include <localization_common/pose_covariance_interpolater.h>

namespace localization_common {
struct DistanceScaledPoseCovarianceInterpolaterParams {
  double min_covariance = 1e-4;
  double translation_scale = 1;
  double orientation_scale = 0.01;
};

class DistanceScaledPoseCovarianceInterpolater : public PoseCovarianceInterpolater {
 public:
  DistanceScaledPoseCovarianceInterpolater(
    const DistanceScaledPoseCovarianceInterpolaterParams& params = DistanceScaledPoseCovarianceInterpolaterParams());

  PoseCovariance Interpolate(const PoseWithCovariance& a, const PoseWithCovariance& b, const Time& timestamp_a,
                             const Time& timestamp_b) final;

 private:
  DistanceScaledPoseCovarianceInterpolaterParams params_;
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_DISTANCE_SCALED_POSE_COVARIANCE_INTERPOLATER_H_
