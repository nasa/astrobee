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

#include <graph_vio/feature_track.h>
#include <graph_vio/serialization.h>
#include <graph_vio/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <cstdlib>
#include <string>

namespace graph_vio {
namespace go = graph_optimizer;
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

bool ValidPointSet(const int num_points, const double average_distance_from_mean,
                   const double min_avg_distance_from_mean, const int min_num_points) {
  if (num_points < min_num_points) return false;
  return (average_distance_from_mean >= min_avg_distance_from_mean);
}

double AverageDistanceFromMean(const std::vector<lm::FeaturePoint>& points) {
  // Calculate mean point and avg distance from mean
  Eigen::Vector2d sum_of_points = Eigen::Vector2d::Zero();
  for (const auto& point : points) {
    sum_of_points += point.image_point;
  }
  const Eigen::Vector2d mean_point = sum_of_points / points.size();

  double sum_of_distances_from_mean = 0;
  for (const auto& point : points) {
    const Eigen::Vector2d mean_centered_point = point.image_point - mean_point;
    sum_of_distances_from_mean += mean_centered_point.norm();
  }
  const double average_distance_from_mean = sum_of_distances_from_mean / points.size();
  return average_distance_from_mean;
}

ff_msgs::GraphVIOState GraphStateMsg(const lc::CombinedNavState& combined_nav_state,
                                  const lc::CombinedNavStateCovariances& covariances,
                                  const FeatureCounts& detected_feature_counts, const bool estimating_bias,
                                  const double position_log_det_threshold, const double orientation_log_det_threshold,
                                  const bool standstill, const GraphVIOStats& graph_stats,
                                  const lm::FanSpeedMode fan_speed_mode) {
  ff_msgs::GraphVIOState msg;

  // Set Header Frames
  msg.header.frame_id = "odom";
  msg.child_frame_id = "body";

  // Set CombinedNavState
  lc::CombinedNavStateToMsg(combined_nav_state, msg);

  // Set Variances
  lc::CombinedNavStateCovariancesToMsg(covariances, msg);

  // Set Confidence
  // Controller can't handle a confidence other than 0.  TODO(rsoussan): switch back when this is fixed.
  msg.confidence = 0;  // covariances.PoseConfidence(position_log_det_threshold, orientation_log_det_threshold);

  // Set Graph Feature Counts/Information
  msg.num_detected_of_features = detected_feature_counts.of;
  msg.estimating_bias = estimating_bias;

  // Set Graph Stats
  msg.iterations = graph_stats.iterations_averager_.last_value();
  msg.optimization_time = graph_stats.optimization_timer_.last_value();
  msg.update_time = graph_stats.update_timer_.last_value();
  msg.num_factors = graph_stats.num_factors_averager_.last_value();
  msg.num_of_factors = graph_stats.num_optical_flow_factors_averager_.last_value();
  msg.num_states = graph_stats.num_states_averager_.last_value();
  // Other
  msg.standstill = standstill;
  msg.fan_speed_mode = static_cast<uint8_t>(fan_speed_mode);
  return msg;
}

ff_msgs::SerializedGraph GraphMsg(const GraphVIO& graph_vio) {
  ff_msgs::SerializedGraph graph_msg;

  // Set Header Frames
  graph_msg.header.frame_id = "odom";
  graph_msg.child_frame_id = "body";

  // TODO(rsoussan): set correct time
  lc::TimeToHeader(5, graph_msg.header);

  graph_msg.serialized_graph = SerializeBinary(graph_vio);
  return graph_msg;
}

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const std_msgs::Header& header) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = header;
  mc::EigenPoseToMsg(global_T_body, pose_msg.pose);
  return pose_msg;
}

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const lc::Time time) {
  std_msgs::Header header;
  lc::TimeToHeader(time, header);
  header.frame_id = "world";
  return PoseMsg(global_T_body, header);
}

geometry_msgs::PoseStamped PoseMsg(const gtsam::Pose3& global_T_body, const lc::Time time) {
  return PoseMsg(lc::EigenPose(global_T_body), time);
}

geometry_msgs::PoseStamped PoseMsg(const lm::TimestampedPose& timestamped_pose) {
  return PoseMsg(timestamped_pose.pose, timestamped_pose.time);
}

gtsam::noiseModel::Robust::shared_ptr Robust(const gtsam::SharedNoiseModel& noise, const double huber_k) {
  return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(huber_k), noise);
}

boost::optional<SharedRobustSmartFactor> FixSmartFactorByRemovingIndividualMeasurements(
  const SmartProjectionFactorAdderParams& params, const RobustSmartFactor& smart_factor,
  const gtsam::SmartProjectionParams& smart_projection_params, const CombinedNavStateGraphValues& graph_values) {
  // TODO(rsoussan): Make this more efficient by enabled removal of measurements and keys in smart factor
  const auto original_measurements = smart_factor.measured();
  const auto original_keys = smart_factor.keys();
  int measurement_index_to_remove;
  // Start with latest measurement
  for (measurement_index_to_remove = original_measurements.size() - 1; measurement_index_to_remove >= 0;
       --measurement_index_to_remove) {
    gtsam::PinholePose<gtsam::Cal3_S2>::MeasurementVector measurements_to_add;
    gtsam::KeyVector keys_to_add;
    for (int i = 0; i < static_cast<int>(original_measurements.size()); ++i) {
      if (i == measurement_index_to_remove) continue;
      measurements_to_add.emplace_back(original_measurements[i]);
      keys_to_add.emplace_back(original_keys[i]);
    }
    auto new_smart_factor = boost::make_shared<RobustSmartFactor>(
      params.cam_noise, params.cam_intrinsics, params.body_T_cam, smart_projection_params,
      params.rotation_only_fallback, params.robust, params.huber_k);
    new_smart_factor->add(measurements_to_add, keys_to_add);
    const auto new_point = new_smart_factor->triangulateSafe(new_smart_factor->cameras(graph_values.values()));
    if (new_point.valid()) {
      LogDebug("FixSmartFactorByRemovingIndividualMeasurements: Fixed by removing measurement "
               << measurement_index_to_remove << ", num original measurements: " << original_measurements.size());
      return new_smart_factor;
    }
  }
  return boost::none;
}

boost::optional<SharedRobustSmartFactor> FixSmartFactorByRemovingMeasurementSequence(
  const SmartProjectionFactorAdderParams& params, const RobustSmartFactor& smart_factor,
  const gtsam::SmartProjectionParams& smart_projection_params, const CombinedNavStateGraphValues& graph_values) {
  constexpr int min_num_measurements = 2;
  // TODO(rsoussan): Make this more efficient by enabled removal of measurements and keys in smart factor
  const auto original_measurements = smart_factor.measured();
  const auto original_keys = smart_factor.keys();
  int num_measurements_to_add = original_measurements.size() - 1;
  // Try to remove min number of most recent measurements
  while (num_measurements_to_add >= min_num_measurements) {
    gtsam::PinholePose<gtsam::Cal3_S2>::MeasurementVector measurements_to_add;
    gtsam::KeyVector keys_to_add;
    for (int i = 0; i < num_measurements_to_add; ++i) {
      measurements_to_add.emplace_back(original_measurements[i]);
      keys_to_add.emplace_back(original_keys[i]);
    }
    auto new_smart_factor = boost::make_shared<RobustSmartFactor>(
      params.cam_noise, params.cam_intrinsics, params.body_T_cam, smart_projection_params,
      params.rotation_only_fallback, params.robust, params.huber_k);
    new_smart_factor->add(measurements_to_add, keys_to_add);
    const auto new_point = new_smart_factor->triangulateSafe(new_smart_factor->cameras(graph_values.values()));
    if (new_point.valid()) {
      LogDebug(
        "FixSmartFactorByRemovingMeasurementSequence: Fixed smart factor by removing most recent "
        "measurements. Original "
        "measurement size: "
        << original_measurements.size() << ", new size: " << num_measurements_to_add);
      return new_smart_factor;
    } else {
      --num_measurements_to_add;
    }
  }
  if (num_measurements_to_add < min_num_measurements) {
    num_measurements_to_add = original_measurements.size() - 1;
    // Try to remove min number of oldest measurements
    while (num_measurements_to_add >= min_num_measurements) {
      gtsam::PinholePose<gtsam::Cal3_S2>::MeasurementVector measurements_to_add;
      gtsam::KeyVector keys_to_add;
      for (int i = num_measurements_to_add;
           i >= static_cast<int>(original_measurements.size()) - num_measurements_to_add; --i) {
        measurements_to_add.emplace_back(original_measurements[i]);
        keys_to_add.emplace_back(original_keys[i]);
      }
      auto new_smart_factor = boost::make_shared<RobustSmartFactor>(
        params.cam_noise, params.cam_intrinsics, params.body_T_cam, smart_projection_params,
        params.rotation_only_fallback, params.robust, params.huber_k);
      new_smart_factor->add(measurements_to_add, keys_to_add);
      const auto new_point = new_smart_factor->triangulateSafe(new_smart_factor->cameras(graph_values.values()));
      if (new_point.valid()) {
        LogDebug(
          "FixSmartFactorByRemovingMeasurementSequence: Fixed smart factor by removing oldest measurements. "
          "Original "
          "measurement size: "
          << original_measurements.size() << ", new size: " << num_measurements_to_add);
        return new_smart_factor;
      } else {
        --num_measurements_to_add;
      }
    }
  }
  // Failed to fix smart factor
  return boost::none;
  // TODO(rsoussan): delete factor if fail to find acceptable new one?
  // TODO(rsoussan): attempt to make a second factor with remaining measuremnts!!!
}

SharedRobustSmartFactor RemoveSmartFactorMeasurements(const RobustSmartFactor& smart_factor,
                                                      const std::unordered_set<int>& factor_key_indices_to_remove,
                                                      const SmartProjectionFactorAdderParams& params,
                                                      const gtsam::SmartProjectionParams& smart_projection_params) {
  auto new_smart_factor = boost::make_shared<RobustSmartFactor>(
    params.cam_noise, params.cam_intrinsics, params.body_T_cam, smart_projection_params, params.rotation_only_fallback,
    params.robust, params.huber_k);
  for (int i = 0; i < static_cast<int>(smart_factor.keys().size()); ++i) {
    if (factor_key_indices_to_remove.count(i) == 0)
      new_smart_factor->add(smart_factor.measured()[i], smart_factor.keys()[i]);
  }
  return new_smart_factor;
}

int NumSmartFactors(const gtsam::NonlinearFactorGraph& graph_factors, const gtsam::Values& values,
                    const bool check_valid) {
  int num_of_factors = 0;
  for (const auto& factor : graph_factors) {
    const auto smart_factor = dynamic_cast<const RobustSmartFactor*>(factor.get());
    if (smart_factor) {
      if (check_valid) {
        if (smart_factor->valid(values)) ++num_of_factors;
      } else {
        ++num_of_factors;
      }
    }
  }
  return num_of_factors;
}
}  // namespace graph_vio
