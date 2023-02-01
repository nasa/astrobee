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

#ifndef NODE_UPDATERS_POSE_NODE_UPDATER_H_
#define NODE_UPDATERS_POSE_NODE_UPDATER_H_

#include <graph_optimizer/timestamped_nodes.h>
#include <localization_measurements/timestamped_pose_with_covariance.h>
#include <node_updaters/pose_node_update_model.h>
#include <node_updaters/measurement_based_timestamped_node_updater.h>

#include <gtsam/geometry/Pose3.h>

namespace node_updaters {
namespace go = graph_optimizer;
using PoseNodeUpdater =
  MeasurementBasedTimestampedNodeUpdater<localization_measurements::TimestampedPoseWithCovariance, gtsam::Pose3,
                                         graph_optimizer::TimestampedNodes<gtsam::Pose3>, PoseNodeUpdateModel>;

template <>
graph_optimizer::NodeUpdaterType PoseNodeUpdater::type() const {
  return go::NodeUpdaterType::Pose;
}
}  // namespace node_updaters

#endif  // NODE_UPDATERS_POSE_NODE_UPDATER_H_
