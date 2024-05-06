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

#include <localization_common/utilities.h>
#include <localization_measurements/timestamped_pose.h>
#include <node_adders/pose_node_adder_model.h>

namespace node_adders {
namespace lc = localization_common;
namespace lm = localization_measurements;

gtsam::KeyVector PoseNodeAdderModel::AddNode(const lc::Time timestamp, NodesType& nodes) const {
  const auto lower_bound_or_equal_node = nodes.LowerBoundOrEqual(timestamp);
  if (!lower_bound_or_equal_node) {
    LogError("AddNode: Failed to get lower bound or equal node.");
    return gtsam::KeyVector();
  }
  const auto relative_pose = pose_interpolater_.Relative(lower_bound_or_equal_node->timestamp, timestamp);
  if (!relative_pose) {
    LogError("RelativeNodeAndNoise: Failed to get relative estimate.");
    return gtsam::KeyVector();
  }

  const gtsam::Pose3 extrapolated_pose = lower_bound_or_equal_node->value * lc::GtPose(relative_pose->pose);
  return nodes.Add(timestamp, extrapolated_pose);
}

boost::optional<std::pair<gtsam::Pose3, gtsam::SharedNoiseModel>> PoseNodeAdderModel::RelativeNodeAndNoise(
  const lc::Time timestamp_a, const lc::Time timestamp_b) const {
    if (!nodes_) {
      std::cout << "Not initialized nodes!" << std::endl;
      return boost::none;
    }
    const auto closest_t_a = nodes_->ClosestTimestamp(timestamp_a);
    const auto closest_t_b = nodes_->ClosestTimestamp(timestamp_b);
    if (!closest_t_a || !closest_t_b) {
      std::cout << "Failed to get closest timestamp!!!" << std::endl;
      return boost::none;
    }
    constexpr double eps = 0.5;
    if (std::abs(*closest_t_a - timestamp_a) > eps) {
        std::cout << "timestamp a too far from closest t!" << std::endl;
        return boost::none;
    }
    if (std::abs(*closest_t_b - timestamp_b) > eps) {
        std::cout << "timestamp b too far from closest t!" << std::endl;
        return boost::none;
    }

    const auto keys_a = nodes_->Keys(*closest_t_a);
    if (keys_a.empty()) {
      std::cout << "failed to get keys a!!" << std::endl;
      std::cout << "keys a size: " << keys_a.size() << std::endl;
      std::cout << "time a: " << std::setprecision(15) << timestamp_a << std::endl
                << "latest time: " << *(nodes_->LatestTimestamp()) << std::endl;
      const auto timestamps = nodes_->Timestamps();
      for (const auto t : timestamps) {
        std::cout << "t: " << std::setprecision(15) << t << std::endl;
      }
      return boost::none;
    }
    const auto keys_b = nodes_->Keys(*closest_t_b);
    if (keys_b.empty()) {
      std::cout << "failed to get keys b!!" << std::endl;
      std::cout << "time b: " << std::setprecision(15) << timestamp_b << std::endl
                << "latest time: " << *(nodes_->LatestTimestamp()) << std::endl;
      return boost::none;
    }
    const auto key_a = keys_a[0];
    const auto key_b = keys_b[0];
    // TODO(rsoussan): is this the right order?
    const auto covariance_a_b = marginals_.jointMarginalCovariance({key_a, key_b}).fullMatrix();
  pose_interpolater_.covariance_a_b = covariance_a_b;
  const auto relative_pose = pose_interpolater_.Relative(timestamp_a, timestamp_b);
  if (!relative_pose) {
    LogError("RelativeNodeAndNoise: Failed to get relative estimate.");
    return boost::none;
  }
  const auto relative_pose_noise =
    lc::Robust(gtsam::noiseModel::Gaussian::Covariance(relative_pose->covariance, false), params_.huber_k);
  return std::pair<gtsam::Pose3, gtsam::SharedNoiseModel>(localization_common::GtPose(relative_pose->pose),
                                                          relative_pose_noise);
}

void PoseNodeAdderModel::AddMeasurement(const lm::PoseWithCovarianceMeasurement& measurement) {
  pose_interpolater_.Add(measurement.timestamp, measurement.PoseWithCovariance());
}

void PoseNodeAdderModel::RemoveMeasurements(const lc::Time oldest_allowed_time) {
  // Keep lower bound so future measurements can be interpolated using it.
  pose_interpolater_.RemoveBelowLowerBoundValues(oldest_allowed_time);
}

bool PoseNodeAdderModel::CanAddNode(const localization_common::Time timestamp) const {
  return pose_interpolater_.WithinBounds(timestamp);
}
}  // namespace node_adders
