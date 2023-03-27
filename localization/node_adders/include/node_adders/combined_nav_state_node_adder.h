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

#ifndef NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_H_
#define NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_H_

#include <localization_measurements/imu_measurement.h>
#include <node_adders/combined_nav_state_node_adder_model.h>
#include <node_adders/measurement_based_timestamped_node_adder.h>
#include <nodes/combined_nav_state_nodes.h>

namespace node_adders {
using CombinedNavStateNodeAdder =
  MeasurementBasedTimestampedNodeAdder<localization_measurements::ImuMeasurement, localization_common::CombinedNavState,
                                       nodes::CombinedNavStateNodes, CombinedNavStateNodeAdderModel>;
}  // namespace node_adders

#endif  // NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_H_
