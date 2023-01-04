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

#include <factor_adders/handrail_factor_adder.h>
#include <factor_adders/point_to_handrail_endpoint_factor.h>
#include <factor_adders/point_to_line_factor.h>
#include <factor_adders/point_to_line_segment_factor.h>
#include <factor_adders/point_to_plane_factor.h>
#include <factor_adders/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/inference/Symbol.h>

namespace factor_adders {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
HandrailFactorAdder::HandrailFactorAdder(const HandrailFactorAdderParams& params) : HandrailFactorAdder::Base(params) {}

void HandrailFactorAdder::AddPointToLineOrLineSegmentFactors(
  const lm::HandrailPointsMeasurement& handrail_points_measurement, std::vector<go::FactorsToAdd>& factors_to_add) {
  const int num_line_measurements = static_cast<int>(handrail_points_measurement.sensor_t_line_points.size());
  if (num_line_measurements < params().min_num_line_matches) {
    LogDebug("AddPointToLineOrLineSegmentFactors: Not enough handrail line measurements.");
    return;
  }
  go::FactorsToAdd point_to_line_or_line_segment_factors_to_add;
  point_to_line_or_line_segment_factors_to_add.reserve(num_line_measurements);
  point_to_line_or_line_segment_factors_to_add.SetTimestamp(handrail_points_measurement.timestamp);
  const gtsam::Vector2 point_to_line_noise_sigmas(
    (gtsam::Vector(2) << params().point_to_line_stddev, params().point_to_line_stddev).finished());
  const auto point_to_line_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(point_to_line_noise_sigmas)),
           params().huber_k);
  const gtsam::Vector3 point_to_line_segment_noise_sigmas(
    (gtsam::Vector(3) << params().point_to_line_stddev, params().point_to_line_stddev, params().point_to_line_stddev)
      .finished());
  const auto point_to_line_segment_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(point_to_line_segment_noise_sigmas)),
           params().huber_k);
  const go::KeyInfo key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, handrail_points_measurement.timestamp);
  for (const auto& sensor_t_line_point : handrail_points_measurement.sensor_t_line_points) {
    if (handrail_points_measurement.world_T_handrail.accurate_z_position) {
      gtsam::PointToLineSegmentFactor::shared_ptr point_to_line_segment_factor(new gtsam::PointToLineSegmentFactor(
        sensor_t_line_point, handrail_points_measurement.world_T_handrail.pose, params().body_T_perch_cam,
        handrail_points_measurement.world_T_handrail.length, point_to_line_segment_noise, key_info.UninitializedKey()));
      point_to_line_or_line_segment_factors_to_add.push_back({{key_info}, point_to_line_segment_factor});
    } else {
      gtsam::PointToLineFactor::shared_ptr point_to_line_factor(
        new gtsam::PointToLineFactor(sensor_t_line_point, handrail_points_measurement.world_T_handrail.pose,
                                     params().body_T_perch_cam, point_to_line_noise, key_info.UninitializedKey()));
      point_to_line_or_line_segment_factors_to_add.push_back({{key_info}, point_to_line_factor});
    }
  }
  if (handrail_points_measurement.world_T_handrail.accurate_z_position) {
    LogDebug("AddPointToLineOrLineSegmentFactors: Added " << point_to_line_or_line_segment_factors_to_add.size()
                                                          << " point to line segment factors.");
  } else {
    LogDebug("AddPointToLineOrLineSegmentFactors: Added " << point_to_line_or_line_segment_factors_to_add.size()
                                                          << " point to line factors.");
  }
  factors_to_add.emplace_back(point_to_line_or_line_segment_factors_to_add);
}

void HandrailFactorAdder::AddPointToPlaneFactors(const lm::HandrailPointsMeasurement& handrail_points_measurement,
                                                 std::vector<go::FactorsToAdd>& factors_to_add) {
  const int num_plane_measurements = static_cast<int>(handrail_points_measurement.sensor_t_plane_points.size());
  if (num_plane_measurements < params().min_num_plane_matches) {
    LogDebug("AddPointToPlaneFactors: Not enough handrail plane measurements.");
    return;
  }

  go::FactorsToAdd point_to_plane_factors_to_add;
  point_to_plane_factors_to_add.reserve(num_plane_measurements);
  point_to_plane_factors_to_add.SetTimestamp(handrail_points_measurement.timestamp);
  const gtsam::Vector1 point_to_plane_noise_sigmas((gtsam::Vector(1) << params().point_to_plane_stddev).finished());
  const auto point_to_plane_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(point_to_plane_noise_sigmas)),
           params().huber_k);
  const go::KeyInfo key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, handrail_points_measurement.timestamp);
  for (const auto& sensor_t_plane_point : handrail_points_measurement.sensor_t_plane_points) {
    gtsam::PointToPlaneFactor::shared_ptr point_to_plane_factor(
      new gtsam::PointToPlaneFactor(sensor_t_plane_point, handrail_points_measurement.world_T_handrail_plane,
                                    params().body_T_perch_cam, point_to_plane_noise, key_info.UninitializedKey()));
    point_to_plane_factors_to_add.push_back({{key_info}, point_to_plane_factor});
  }
  LogDebug("AddPointToPlaneFactors: Added " << point_to_plane_factors_to_add.size() << " point to plane factors.");
  factors_to_add.emplace_back(point_to_plane_factors_to_add);
}

void HandrailFactorAdder::AddPointToHandrailEndpointFactors(
  const lm::HandrailPointsMeasurement& handrail_points_measurement, std::vector<go::FactorsToAdd>& factors_to_add) {
  const int num_handrail_endpoint_measurements =
    static_cast<int>(handrail_points_measurement.sensor_t_line_endpoints.size());
  if (num_handrail_endpoint_measurements == 0) {
    LogDebug("AddPointToHandrailEndpointFactors: Not enough handrail endpoint measurements.");
    return;
  }

  go::FactorsToAdd point_to_handrail_endpoint_factors_to_add;
  point_to_handrail_endpoint_factors_to_add.reserve(num_handrail_endpoint_measurements);
  point_to_handrail_endpoint_factors_to_add.SetTimestamp(handrail_points_measurement.timestamp);
  const gtsam::Vector3 point_to_handrail_endpoint_noise_sigmas(
    (gtsam::Vector(3) << params().point_to_line_stddev, params().point_to_line_stddev, params().point_to_line_stddev)
      .finished());
  const auto point_to_handrail_endpoint_noise = Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(point_to_handrail_endpoint_noise_sigmas)),
    params().huber_k);
  const go::KeyInfo key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, handrail_points_measurement.timestamp);
  for (const auto& sensor_t_handrail_endpoint : handrail_points_measurement.sensor_t_line_endpoints) {
    gtsam::PointToHandrailEndpointFactor::shared_ptr point_to_handrail_endpoint_factor(
      new gtsam::PointToHandrailEndpointFactor(
        sensor_t_handrail_endpoint, handrail_points_measurement.world_t_handrail_endpoints->first,
        handrail_points_measurement.world_t_handrail_endpoints->second, params().body_T_perch_cam,
        point_to_handrail_endpoint_noise, key_info.UninitializedKey()));
    point_to_handrail_endpoint_factors_to_add.push_back({{key_info}, point_to_handrail_endpoint_factor});
  }
  LogDebug("AddPointToHandrailEndpointFactors: Added " << point_to_handrail_endpoint_factors_to_add.size()
                                                       << " point to handrail endpoint factors.");
  factors_to_add.emplace_back(point_to_handrail_endpoint_factors_to_add);
}

std::vector<go::FactorsToAdd> HandrailFactorAdder::AddFactors(
  const lm::HandrailPointsMeasurement& handrail_points_measurement) {
  if (handrail_points_measurement.sensor_t_line_points.empty() &&
      handrail_points_measurement.sensor_t_plane_points.empty()) {
    LogDebug("AddFactors: Empty measurement.");
    return {};
  }

  std::vector<go::FactorsToAdd> factors_to_add;
  AddPointToLineOrLineSegmentFactors(handrail_points_measurement, factors_to_add);
  AddPointToPlaneFactors(handrail_points_measurement, factors_to_add);
  if (handrail_points_measurement.world_t_handrail_endpoints) {
    AddPointToHandrailEndpointFactors(handrail_points_measurement, factors_to_add);
  }
  return factors_to_add;
}
}  // namespace factor_adders
