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

#ifndef NODE_ADDERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_ADDER_MODEL_H_
#define NODE_ADDERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_ADDER_MODEL_H_

#include <node_adders/timestamped_node_adder_model.h>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace node_adders {
// Node adder model that uses provided measurements to generate nodes and relative factors.
template <typename MeasurementType, typename NodeType, typename NodesType>
class MeasurementBasedTimestampedNodeAdderModel : public TimestampedNodeAdderModel<NodeType, NodesType> {
 public:
  explicit MeasurementBasedTimestampedNodeAdderModel(const TimestampedNodeAdderModelParams& params)
      : TimestampedNodeAdderModel<NodeType, NodesType>(params) {}
  virtual ~MeasurementBasedTimestampedNodeAdderModel() = default;
  virtual void AddMeasurement(const MeasurementType& measurement) = 0;
  virtual void RemoveMeasurements(const localization_common::Time oldest_allowed_time) = 0;
};
}  // namespace node_adders

#endif  // NODE_ADDERS_MEASUREMENT_BASED_TIMESTAMPED_NODE_ADDER_MODEL_H_
