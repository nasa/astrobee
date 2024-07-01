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

#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_

#include <factor_adders/loc_factor_adder.h>
#include <graph_localizer/graph_localizer_params.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <localization_measurements/pose_measurement.h>
#include <node_adders/pose_node_adder.h>
#include <nodes/timestamped_nodes.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>

#include <boost/serialization/serialization.hpp>

namespace graph_localizer {
// Siding window graph optimizer that uses matched projections and odomery poses to perform localization.
// Uses the PoseNodeAdder to add relative odometry pose factors between graph nodes.
// Matched projections typically come from matching images to a map of image
// features.
class GraphLocalizer : public sliding_window_graph_optimizer::SlidingWindowGraphOptimizer {
 public:
  explicit GraphLocalizer(const GraphLocalizerParams& params);

  // For Serialization Only
  GraphLocalizer() {}

  // Adds pose measurement to the pose node adder.
  void AddPoseMeasurement(const localization_measurements::PoseWithCovarianceMeasurement& pose_measurement);

  // Adds sparse map matched projections measurement to loc factor adder.
  void AddSparseMapMatchedProjectionsMeasurement(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);

  // Adds AR tag matched projections measurement to loc factor adder.
  void AddArTagMatchedProjectionsMeasurement(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);

  // Returns a const reference to pose nodes.
  const nodes::TimestampedNodes<gtsam::Pose3>& pose_nodes() const;

  // Sets pose covariance interpolater for relative odometry pose node creation
  void SetPoseCovarianceInterpolater(
    const std::shared_ptr<localization_common::MarginalsPoseCovarianceInterpolater<nodes::CombinedNavStateNodes>>&
      pose_covariance_interpolater);

 private:
  // bool ValidGraph() const final;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(sliding_window_graph_optimizer::SlidingWindowGraphOptimizer);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(sparse_map_loc_factor_adder_);
    ar& BOOST_SERIALIZATION_NVP(ar_tag_loc_factor_adder_);
    ar& BOOST_SERIALIZATION_NVP(pose_node_adder_);
  }

  GraphLocalizerParams params_;

  // Factor Adders
  std::shared_ptr<factor_adders::LocFactorAdder<node_adders::PoseNodeAdder>> sparse_map_loc_factor_adder_;
  std::shared_ptr<factor_adders::LocFactorAdder<node_adders::PoseNodeAdder>> ar_tag_loc_factor_adder_;

  // Node Adders
  std::shared_ptr<node_adders::PoseNodeAdder> pose_node_adder_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_
