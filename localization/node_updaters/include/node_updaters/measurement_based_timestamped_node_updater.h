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
template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedNodeUpdateModelType>
class MeasurementBasedTimestampedNodeUpdater
    : public TimestampedNodeUpdater<NodeType, TimestampedNodesType, MeasurementBasedNodeUpdateModelType> {
  using Base = TimestampedNodeUpdater<NodeType, TimestampedNodesType, MeasurementBasedNodeUpdateModelType>;

 public:
  void AddMeasurement(const MeasurementType& measurement);
  void RemoveMeasurements(const localization_common::Time oldest_allowed_time);

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
          typename MeasurementBasedNodeUpdateModelType>
void MeasurementBasedTimestampedNodeUpdater<MeasurementType, NodeType, TimestampedNodesType,
                                            MeasurementBasedNodeUpdateModelType>::AddMeasurement(const MeasurementType&
                                                                                                   measurement) {
  node_update_model_->AddMeasurement(measurement);
}

template <typename MeasurementType, typename NodeType, typename TimestampedNodesType,
          typename MeasurementBasedNodeUpdateModelType>
void MeasurementBasedTimestampedNodeUpdater<
  MeasurementType, NodeType, TimestampedNodesType,
  MeasurementBasedNodeUpdateModelType>::RemoveMeasurements(const localization_common::Time oldest_allowed_time) {
  node_update_model_->RemoveMeasurements(oldest_allowed_time);
}
}  // namespace node_updaters

#endif  // NODE_UPDATERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_UPDATER_H_
