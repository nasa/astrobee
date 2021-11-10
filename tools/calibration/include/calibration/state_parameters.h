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
#ifndef CALIBRATION_STATE_PARAMETERS_H_
#define CALIBRATION_STATE_PARAMETERS_H_

#include <ff_common/eigen_vectors.h>
#include <optimization_common/utilities.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace calibration {
struct StateParameters {
  bool operator==(const StateParameters& rhs) const {
    bool equal = true;
    equal &= focal_lengths.matrix().isApprox(rhs.focal_lengths.matrix(), 1e-6);
    equal &= principal_points.matrix().isApprox(rhs.principal_points.matrix(), 1e-6);
    equal &= distortion.matrix().isApprox(rhs.distortion.matrix(), 1e-6);
    return equal;
  }

  Eigen::Vector2d focal_lengths;
  Eigen::Vector2d principal_points;
  Eigen::VectorXd distortion;
};

// Container for state parameters in vector form used during optimization
struct OptimizationStateParameters {
  void SetInitialStateParameters(const StateParameters& initial_state_parameters) {
    focal_lengths = initial_state_parameters.focal_lengths;
    principal_points = initial_state_parameters.principal_points;
    distortion = initial_state_parameters.distortion;
  }

  void AddCameraTTarget(const Eigen::Isometry3d& camera_T_target) {
    camera_T_targets.emplace_back(optimization_common::VectorFromIsometry3d(camera_T_target));
  }

  StateParameters OptimizedStateParameters() const {
    StateParameters optimized_state_parameters;
    optimized_state_parameters.focal_lengths = focal_lengths;
    optimized_state_parameters.principal_points = principal_points;
    optimized_state_parameters.distortion = distortion;
    return optimized_state_parameters;
  }

  std::vector<Eigen::Isometry3d> OptimizedCameraTTargets() const {
    std::vector<Eigen::Isometry3d> optimized_camera_T_targets;
    for (const auto& camera_T_target : camera_T_targets)
      optimized_camera_T_targets.emplace_back(optimization_common::Isometry3d(camera_T_target));
    return optimized_camera_T_targets;
  }

  Eigen::Vector2d focal_lengths;
  Eigen::Vector2d principal_points;
  Eigen::VectorXd distortion;
  std::vector<Eigen::Matrix<double, 6, 1>> camera_T_targets;
};
}  // namespace calibration

#endif  // CALIBRATION_STATE_PARAMETERS_H_