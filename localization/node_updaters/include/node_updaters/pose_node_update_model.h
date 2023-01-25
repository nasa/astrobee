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

#ifndef NODE_UPDATERS_POSE_NODE_UPDATE_MODEL_H_
#define NODE_UPDATERS_POSE_NODE_UPDATE_MODEL_H_

#include <graph_optimizer/timestamped_nodes.h>
#include <localization_common/pose_with_covariance_interpolater.h>
#include <node_updaters/node_update_model.h>

#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace node_updaters {
using NodesType = graph_optimizer::TimestampedNodes<gtsam::Pose3>;
using Base = NodeUpdateModel<NodesType>;
class PoseNodeUpdateModel : Base {
 public:
  boost::optional<gtsam::Key> AddNode(const localization_common::Time timestamp, NodesType& nodes) final;
  bool AddRelativeFactor(const gtsam::Key key_a, const localization_common::Time timestamp_a, const gtsam::Key key_b,
                         const localization_common::Time timestamp_b, gtsam::NonlinearFactorGraph& factors) const final;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(pose_interpolater_);
  }

  localization_common::PoseWithCovarianceInterpolater pose_interpolater_;
};
}  // namespace node_updaters

#endif  // NODE_UPDATERS_POSE_NODE_UPDATE_MODEL_H_
