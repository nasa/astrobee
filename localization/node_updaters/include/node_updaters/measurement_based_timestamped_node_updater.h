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

#ifndef NODE_UPDATERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_UPDATER_H_
#define NODE_UPDATERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_UPDATER_H_

#include <node_updaters/timestamped_node_updater.h>

namespace node_updaters {

// Timestamped node updater that uses provided measurements to create nodes.
template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeUpdateModelType>
class MeasurementBasedTimestampedNodeUpdater
    : public TimestampedNodeUpdater<NodeType, TimestampedNodesType, MeasurementBasedTimestampedNodeUpdateModelType> {
  using Base = TimestampedNodeUpdater<NodeType, TimestampedNodesType, MeasurementBasedTimestampedNodeUpdateModelType>;

 public:
  MeasurementBasedTimestampedNodeUpdater(
    const MeasurementBasedTimestampedNodeUpdaterParams<MeasurementType, NodeType>& params,
    const typename MeasurementBasedTimestampedNodeUpdateModelType::Params& node_update_model_params,
    std::shared_ptr<TimestampedNodesType> nodes = std::make_shared<TimestampedNodesType>());
  void AddMeasurement(const MeasurementType& measurement);
  void RemoveMeasurements(const localization_common::Time oldest_allowed_time);
  // Slides window and removes old measurements
  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<gtsam::Marginals>& marginals, const gtsam::KeyVector& old_keys,
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
          typename MeasurementBasedTimestampedNodeUpdateModelType>
MeasurementBasedTimestampedNodeUpdater<MeasurementType, NodeType, TimestampedNodesType,
                                       MeasurementBasedTimestampedNodeUpdateModelType>::
  MeasurementBasedTimestampedNodeUpdater(
    const MeasurementBasedTimestampedNodeUpdaterParams<MeasurementType, NodeType>& params,
    const typename MeasurementBasedTimestampedNodeUpdateModelType::Params& node_update_model_params,
    std::shared_ptr<TimestampedNodesType> nodes)
    : Base(params, node_update_model_params, std::move(nodes)) {
  // Store start measurement so future relative estimates can be computed wrt this
  AddMeasurement(params.start_measurement);
}

template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeUpdateModelType>
void MeasurementBasedTimestampedNodeUpdater<
  MeasurementType, NodeType, TimestampedNodesType,
  MeasurementBasedTimestampedNodeUpdateModelType>::AddMeasurement(const MeasurementType& measurement) {
  this->node_update_model_.AddMeasurement(measurement);
}

template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeUpdateModelType>
void MeasurementBasedTimestampedNodeUpdater<MeasurementType, NodeType, TimestampedNodesType,
                                            MeasurementBasedTimestampedNodeUpdateModelType>::
  RemoveMeasurements(const localization_common::Time oldest_allowed_time) {
  this->node_update_model_.RemoveMeasurements(oldest_allowed_time);
}

template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedTimestampedNodeUpdateModelType>
bool MeasurementBasedTimestampedNodeUpdater<
  MeasurementType, NodeType, TimestampedNodesType,
  MeasurementBasedTimestampedNodeUpdateModelType>::SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                                                               const boost::optional<gtsam::Marginals>& marginals,
                                                               const gtsam::KeyVector& old_keys, const double huber_k,
                                                               gtsam::NonlinearFactorGraph& factors) {
  if (!Base::SlideWindow(oldest_allowed_timestamp, marginals, old_keys, huber_k, factors)) {
    LogError("Failed to slide window.");
    return false;
  }
  RemoveMeasurements(oldest_allowed_timestamp);
  return true;
}
}  // namespace node_updaters

#endif  // NODE_UPDATERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_UPDATER_H_
