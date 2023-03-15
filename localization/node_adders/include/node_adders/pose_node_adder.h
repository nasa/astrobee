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

#ifndef NODE_ADDERS_POSE_NODE_ADDER_H_
#define NODE_ADDERS_POSE_NODE_ADDER_H_

#include <nodes/timestamped_nodes.h>
#include <localization_measurements/timestamped_pose_with_covariance.h>
#include <node_adders/pose_node_adder_model.h>
#include <node_adders/measurement_based_timestamped_node_adder.h>

#include <gtsam/geometry/Pose3.h>

#include <string>

namespace node_adders {
// Timestamp-based node adder using timestamped poses with covariances.
using PoseNodeAdder =
  MeasurementBasedTimestampedNodeAdder<localization_measurements::TimestampedPoseWithCovariance, gtsam::Pose3,
                                         nodes::TimestampedNodes<gtsam::Pose3>, PoseNodeAdderModel>;

template <>
std::string TimestampedNodeAdder<gtsam::Pose3, nodes::TimestampedNodes<gtsam::Pose3>,
                                   PoseNodeAdderModel>::type() const {
  return "pose";
}
}  // namespace node_adders

#endif  // NODE_ADDERS_POSE_NODE_ADDER_H_