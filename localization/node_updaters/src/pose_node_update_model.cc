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

#include <graph_optimizer/utilities.h>
#include <localization_common/utilities.h>
#include <node_updaters/pose_node_update_model.h>

#include <gtsam/slam/BetweenFactor.h>

namespace node_updaters {
namespace lc = localization_common;
namespace go = graph_optimizer;
boost::optional<gtsam::Key> PoseNodeUpdateModel::AddNode(const localization_common::Time timestamp, NodesType& nodes) {
  const auto pose = pose_interpolater_.Interpolate(timestamp);
  if (!pose) {
    LogError("AddNode: Failed to get value.");
  }
  return nodes.Add(timestamp, localization_common::GtPose(pose->pose));
}

bool PoseNodeUpdateModel::AddRelativeFactor(const gtsam::Key key_a, const localization_common::Time timestamp_a,
                                            const gtsam::Key key_b, const localization_common::Time timestamp_b,
                                            gtsam::NonlinearFactorGraph& factors) const {
  const auto relative_pose = pose_interpolater_.Relative(timestamp_a, timestamp_b);
  if (!relative_pose) {
    LogError("AddRelativeFactor: Failed to get relative estimate.");
    return false;
  }
  const auto relative_pose_noise =
    go::Robust(gtsam::noiseModel::Gaussian::Covariance(relative_pose->covariance, false), params_.huber_k);

  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr relative_pose_factor(
    new gtsam::BetweenFactor<gtsam::Pose3>(key_a, key_b, lc::GtPose(relative_pose->pose), relative_pose_noise));
  factors.push_back(relative_pose_factor);
}
}  // namespace node_updaters
