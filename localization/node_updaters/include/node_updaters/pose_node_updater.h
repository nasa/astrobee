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

#include <graph_optimizer/node_updater_with_priors.h>
#include <graph_optimizer/timestamped_nodes.h>
#include <localization_common/pose_with_covariance_interpolater.h>
#include <node_updaters/pose_node_updater_params.h>

#include <gtsam/geometry/Pose3.h>

namespace node_updaters {
using TimestampedPoseNodes = graph_optimizer::TimestampedNodes<gtsam::Pose3>;
class PoseNodeUpdater
    : public graph_optimizer::NodeUpdaterWithPriors<gtsam::Pose3, gtsam::SharedNoiseModel> {
 public:
  PoseNodeUpdater(std::shared_ptr<TimestampedPoseNodes> nodes,
                  std::shared_ptr<localization_common::PoseWithCovarianceInterpolater> pose_interpolater);
  PoseNodeUpdater() = default;

  void AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& factors);

  void AddInitialValuesAndPriors(const gtsam::Pose3& global_T_body, const gtsam::SharedNoiseModel& noise,
                                 const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  void AddPriors(const gtsam::Pose3& global_T_body, const gtsam::SharedNoiseModel& noise,
                 const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  // TODO(rsousan): Add function to split if needed?
  bool Update(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<gtsam::Marginals>& marginals, const gtsam::KeyVector& old_keys,
                   const double huber_k, gtsam::NonlinearFactorGraph& factors) final;

  graph_optimizer::NodeUpdaterType type() const final;

  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const final;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final;

  boost::optional<gtsam::Key> GetKey(graph_optimizer::KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const final;

  boost::optional<localization_common::Time> OldestTimestamp() const final;

  boost::optional<localization_common::Time> LatestTimestamp() const final;

 private:
  void RemovePriors(const gtsam::KeyVector& old_keys, gtsam::NonlinearFactorGraph& factors);
bool AddLatestNodeAndRelativeFactor(
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);
boost::optional<gtsam::Key> LatestKey();
boost::optional<gtsam::Key> AddNode(const localization_common::Time timestamp);
bool AddRelativeFactor(const gtsam::Key key_a, const localization_common::Time timestamp_a, const gtsam::Key key_b,
                       const localization_common::Time timestamp_b, gtsam::NonlinearFactorGraph& factors) const;
bool AddNodeAndRelativeFactor(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
                              gtsam::NonlinearFactorGraph& factors);
bool SplitOldRelativeFactor(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);
bool RemoveFactors(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);

// Serialization function
friend class boost::serialization::access;
template <class Archive>
void serialize(Archive& ar, const unsigned int file_version) {
  ar& BOOST_SERIALIZATION_NVP(nodes_);
  ar& BOOST_SERIALIZATION_NVP(params_);
  ar& BOOST_SERIALIZATION_NVP(global_T_body_start_noise_);
  }

  std::shared_ptr<TimestampedPoseNodes> nodes_;
  std::shared_ptr<localization_common::PoseWithCovarianceInterpolater> pose_interpolater_;
  PoseNodeUpdaterParams params_;
  gtsam::SharedNoiseModel global_T_body_start_noise_;
};
}  // namespace node_updaters

#endif  // NODE_UPDATERS_POSE_NODE_UPDATER_H_
