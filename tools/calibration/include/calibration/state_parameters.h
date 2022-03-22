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
#include <localization_common/logger.h>
#include <optimization_common/utilities.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace calibration {
struct StateParameters {
  void Print() {
    LogInfo("Focal lengths: " << std::endl << focal_lengths.matrix());
    LogInfo("Principal points: " << std::endl << principal_points.matrix());
    LogInfo("Distortion: " << std::endl << distortion.matrix());
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

struct StateParametersCovariances {
  void Print() {
    LogInfo("Focal length covariances: " << std::endl << focal_lengths.matrix());
    LogInfo("Principal point covariances: " << std::endl << principal_points.matrix());
    LogInfo("Distortion covariances: " << std::endl << distortion.matrix());
  }

  Eigen::Matrix2d focal_lengths;
  Eigen::Matrix2d principal_points;
  Eigen::MatrixXd distortion;
};
}  // namespace calibration

#endif  // CALIBRATION_STATE_PARAMETERS_H_
