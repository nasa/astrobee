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

#ifndef NODE_ADDERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_ADDER_H_
#define NODE_ADDERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_ADDER_H_

#include <node_adders/timestamped_node_adder.h>
#include <node_adders/timestamped_node_adder_params.h>

namespace node_adders {

// Timestamped node adder that uses provided measurements to create nodes.
template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeAdderModelType>
class MeasurementBasedTimestampedNodeAdder
    : public TimestampedNodeAdder<NodeType, TimestampedNodesType, MeasurementBasedTimestampedNodeAdderModelType> {
  using Base = TimestampedNodeAdder<NodeType, TimestampedNodesType, MeasurementBasedTimestampedNodeAdderModelType>;

 public:
  using Params = TimestampedNodeAdderParams<NodeType>;
  // Construct using nodes. Creates timestamped nodes interally.
  MeasurementBasedTimestampedNodeAdder(
    const Params& params, const typename MeasurementBasedTimestampedNodeAdderModelType::Params& node_adder_model_params,
    std::shared_ptr<nodes::Values> values);

  // Construct using already constructed timestamped nodes.
  MeasurementBasedTimestampedNodeAdder(
    const Params& params, const typename MeasurementBasedTimestampedNodeAdderModelType::Params& node_adder_model_params,
    std::shared_ptr<TimestampedNodesType> timestamped_nodes);

  void AddMeasurement(const MeasurementType& measurement);

  // Removes measurements using oldest_allowed_time. Depending on the measurement type,
  // the lower bound <= oldest_allowed_time may be kept so it can be used to create
  // relative factors in the future.
  void RemoveMeasurements(const localization_common::Time oldest_allowed_time);

  // Slides window and removes old measurements
  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<const gtsam::Marginals&>& marginals, const gtsam::KeyVector& old_keys,
                   const double huber_k, gtsam::NonlinearFactorGraph& factors) final;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

// Implementation
template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeAdderModelType>
MeasurementBasedTimestampedNodeAdder<MeasurementType, NodeType, TimestampedNodesType,
                                     MeasurementBasedTimestampedNodeAdderModelType>::
  MeasurementBasedTimestampedNodeAdder(
    const Params& params, const typename MeasurementBasedTimestampedNodeAdderModelType::Params& node_adder_model_params,
    std::shared_ptr<nodes::Values> values)
    : Base(params, node_adder_model_params, std::move(values)) {}

template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeAdderModelType>
MeasurementBasedTimestampedNodeAdder<MeasurementType, NodeType, TimestampedNodesType,
                                     MeasurementBasedTimestampedNodeAdderModelType>::
  MeasurementBasedTimestampedNodeAdder(
    const Params& params, const typename MeasurementBasedTimestampedNodeAdderModelType::Params& node_adder_model_params,
    std::shared_ptr<TimestampedNodesType> timestamped_nodes)
    : Base(params, node_adder_model_params, std::move(timestamped_nodes)) {}

template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeAdderModelType>
void MeasurementBasedTimestampedNodeAdder<
  MeasurementType, NodeType, TimestampedNodesType,
  MeasurementBasedTimestampedNodeAdderModelType>::AddMeasurement(const MeasurementType& measurement) {
  Base::node_adder_model().AddMeasurement(measurement);
}

template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeAdderModelType>
void MeasurementBasedTimestampedNodeAdder<MeasurementType, NodeType, TimestampedNodesType,
                                          MeasurementBasedTimestampedNodeAdderModelType>::
  RemoveMeasurements(const localization_common::Time oldest_allowed_time) {
  Base::node_adder_model().RemoveMeasurements(oldest_allowed_time);
}

template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeAdderModelType>
bool MeasurementBasedTimestampedNodeAdder<
  MeasurementType, NodeType, TimestampedNodesType,
  MeasurementBasedTimestampedNodeAdderModelType>::SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                                                              const boost::optional<const gtsam::Marginals&>& marginals,
                                                              const gtsam::KeyVector& old_keys, const double huber_k,
                                                              gtsam::NonlinearFactorGraph& factors) {
  if (!Base::SlideWindow(oldest_allowed_timestamp, marginals, old_keys, huber_k, factors)) {
    LogError("Failed to slide window.");
    return false;
  }
  RemoveMeasurements(oldest_allowed_timestamp);
  return true;
}
}  // namespace node_adders

#endif  // NODE_ADDERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_ADDER_H_
