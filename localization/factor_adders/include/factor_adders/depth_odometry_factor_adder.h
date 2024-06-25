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
#ifndef FACTOR_ADDERS_DEPTH_ODOMETRY_FACTOR_ADDER_H_
#define FACTOR_ADDERS_DEPTH_ODOMETRY_FACTOR_ADDER_H_

#include <factor_adders/depth_odometry_factor_adder_params.h>
#include <factor_adders/single_measurement_based_factor_adder.h>
#include <graph_factors/point_to_point_between_factor.h>
#include <localization_common/time.h>
#include <localization_common/utilities.h>
#include <localization_measurements/depth_odometry_measurement.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

namespace factor_adders {
// Adds GTSAM Pose Between factors for depth odometry relative pose measurements
// point to point between factors for depth odometry correspondence measurements.
// Adds pose nodes using PoseNodeAdder at the same timestamps as the measurements.
template <class PoseNodeAdderType>
class DepthOdometryFactorAdder
    : public SingleMeasurementBasedFactorAdder<localization_measurements::DepthOdometryMeasurement> {
 public:
  DepthOdometryFactorAdder(const DepthOdometryFactorAdderParams& params,
                           const std::shared_ptr<PoseNodeAdderType> node_adder);

 private:
  // Creates a pose between factor or point to point between factors and pose nodes for the given measurement.
  int AddFactorsForSingleMeasurement(const localization_measurements::DepthOdometryMeasurement& measurement,
                                     gtsam::NonlinearFactorGraph& factors) final;

  bool CanAddFactor(const localization_measurements::DepthOdometryMeasurement& measurement) const final;

  std::shared_ptr<PoseNodeAdderType> node_adder_;
  DepthOdometryFactorAdderParams params_;
};

template <class PoseNodeAdderType>
DepthOdometryFactorAdder<PoseNodeAdderType>::DepthOdometryFactorAdder(
  const DepthOdometryFactorAdderParams& params, const std::shared_ptr<PoseNodeAdderType> node_adder)
    : SingleMeasurementBasedFactorAdder<localization_measurements::DepthOdometryMeasurement>(params),
      params_(params),
      node_adder_(node_adder) {}

template <class PoseNodeAdderType>
int DepthOdometryFactorAdder<PoseNodeAdderType>::AddFactorsForSingleMeasurement(
  const localization_measurements::DepthOdometryMeasurement& measurement, gtsam::NonlinearFactorGraph& factors) {
  const double translation_norm = measurement.odometry.sensor_F_source_T_target.pose.translation().norm();
  if (params_.reject_large_translation_norm && translation_norm > params_.pose_translation_norm_threshold) {
    LogDebug("AddFactors: Ignoring pose with large translation norm. Norm: "
             << translation_norm << ", threshold: " << params_.pose_translation_norm_threshold);
    return 0;
  }

  if (!node_adder_->AddNode(measurement.odometry.source_time, factors) ||
      !node_adder_->AddNode(measurement.odometry.target_time, factors)) {
    LogError("AddFactorsForSingleMeasurement: Failed to add nodes at source and target time.");
    return 0;
  }
  const auto keys_a = node_adder_->Keys(measurement.odometry.source_time);
  // First key is pose key
  const auto& pose_key_a = keys_a[0];
  const auto keys_b = node_adder_->Keys(measurement.odometry.target_time);
  const auto& pose_key_b = keys_b[0];

  if (params_.use_points_between_factor) {
    int num_between_factors = 0;
    for (int i = 0; i < measurement.correspondences.source_3d_points.size() &&
                    num_between_factors < params_.max_num_points_between_factors;
         ++i) {
      const Eigen::Vector3d& sensor_t_point_source = measurement.correspondences.source_3d_points[i];
      const Eigen::Vector3d& sensor_t_point_target = measurement.correspondences.target_3d_points[i];

      double noise_sigma = 1.0;
      if (params_.scale_point_between_factors_with_inverse_distance) {
        noise_sigma = sensor_t_point_source.norm();
      } else if (params_.scale_point_between_factors_with_estimate_error) {
        const Eigen::Vector3d estimate_error =
          sensor_t_point_source - measurement.odometry.sensor_F_source_T_target.pose * sensor_t_point_target;
        noise_sigma = estimate_error.norm();
      }
      if (params_.reject_large_point_to_point_error) {
        const Eigen::Vector3d estimate_error =
          sensor_t_point_source - measurement.odometry.sensor_F_source_T_target.pose * sensor_t_point_target;
        if (estimate_error.norm() > params_.point_to_point_error_threshold) {
          continue;
        }
      }

      const auto points_between_factor_noise = localization_common::Robust(
        gtsam::noiseModel::Isotropic::Sigma(3, noise_sigma * params_.point_noise_scale), params_.huber_k);
      gtsam::PointToPointBetweenFactor::shared_ptr points_between_factor(
        new gtsam::PointToPointBetweenFactor(sensor_t_point_source, sensor_t_point_target, params_.body_T_sensor,
                                             points_between_factor_noise, pose_key_a, pose_key_b));
      factors.push_back(points_between_factor);
      ++num_between_factors;
    }
    LogDebug("AddFactors: Added " << num_between_factors << " points between factors.");
    return num_between_factors;
  } else {
    const auto relative_pose_noise = localization_common::Robust(
      gtsam::noiseModel::Gaussian::Covariance(params_.pose_covariance_scale *
                                              measurement.odometry.body_F_source_T_target.covariance),
      params_.huber_k);
    const gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr pose_between_factor(new gtsam::BetweenFactor<gtsam::Pose3>(
      pose_key_a, pose_key_b, localization_common::GtPose(measurement.odometry.body_F_source_T_target.pose),
      relative_pose_noise));
    factors.push_back(pose_between_factor);
    return 1;
  }
}

template <class PoseNodeAdderType>
bool DepthOdometryFactorAdder<PoseNodeAdderType>::CanAddFactor(
  const localization_measurements::DepthOdometryMeasurement& measurement) const {
  return node_adder_->CanAddNode(measurement.odometry.source_time) &&
         node_adder_->CanAddNode(measurement.odometry.target_time);
}
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_DEPTH_ODOMETRY_FACTOR_ADDER_H_
