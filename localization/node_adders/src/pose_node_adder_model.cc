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
  const auto pose = pose_interpolater_.Interpolate(timestamp);
  if (!pose) {
    LogError("AddNode: Failed to get value.");
    return gtsam::KeyVector();
  }
  return nodes.Add(timestamp, lc::GtPose(pose->pose));
}

boost::optional<std::pair<gtsam::Pose3, gtsam::SharedNoiseModel>> PoseNodeAdderModel::RelativeNodeAndNoise(
  const lc::Time timestamp_a, const lc::Time timestamp_b) const {
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
