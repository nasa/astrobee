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

#ifndef NODE_ADDERS_POSE_NODE_ADDER_MODEL_H_
#define NODE_ADDERS_POSE_NODE_ADDER_MODEL_H_

#include <localization_common/pose_with_covariance_interpolater.h>
#include <localization_measurements/timestamped_pose_with_covariance.h>
#include <node_adders/between_factor_node_adder_model.h>
#include <nodes/timestamped_nodes.h>

#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose3.h>

#include <utility>

namespace node_adders {
class PoseNodeAdderModel : public BetweenFactorMeasurementBasedTimestampedNodeAdderModel<
                              localization_measurements::TimestampedPoseWithCovariance, gtsam::Pose3> {
  using Base =
    BetweenFactorMeasurementBasedTimestampedNodeAdderModel<localization_measurements::TimestampedPoseWithCovariance,
                                                            gtsam::Pose3>;
  using NodesType = nodes::TimestampedNodes<gtsam::Pose3>;

 public:
  using Params = TimestampedNodeAdderModelParams;

  explicit PoseNodeAdderModel(const Params& params) : Base(params) {}

  gtsam::KeyVector AddNode(const localization_common::Time timestamp, NodesType& nodes) const final;
  boost::optional<std::pair<gtsam::Pose3, gtsam::SharedNoiseModel>> RelativeNodeAndNoise(
    const localization_common::Time timestamp_a, const localization_common::Time timestamp_b) const final;
  void AddMeasurement(const localization_measurements::TimestampedPoseWithCovariance& measurement);
  void RemoveMeasurements(const localization_common::Time oldest_allowed_time);
  bool CanAddNode(const localization_common::Time timestamp) const final;

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
}  // namespace node_adders

#endif  // NODE_ADDERS_POSE_NODE_ADDER_MODEL_H_