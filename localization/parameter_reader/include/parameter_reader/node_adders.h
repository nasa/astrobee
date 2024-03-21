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
#ifndef PARAMETER_READER_NODE_ADDERS_H_
#define PARAMETER_READER_NODE_ADDERS_H_

#include <config_reader/config_reader.h>
#include <node_adders/combined_nav_state_node_adder.h>
#include <node_adders/combined_nav_state_node_adder_model_params.h>
#include <node_adders/pose_node_adder_params.h>
#include <node_adders/timestamped_node_adder_model_params.h>
#include <node_adders/timestamped_node_adder_params.h>

#include <string>

namespace parameter_reader {
template <typename NodeType>
void LoadBaseTimestampedNodeAdderParams(config_reader::ConfigReader& config,
                                        node_adders::TimestampedNodeAdderParams<NodeType>& params,
                                        const std::string& prefix = "");

void LoadTimestampedNodeAdderModelParams(config_reader::ConfigReader& config,
                                         node_adders::TimestampedNodeAdderModelParams& params,
                                         const std::string& prefix = "");

void LoadCombinedNavStateNodeAdderParams(config_reader::ConfigReader& config,
                                         node_adders::CombinedNavStateNodeAdder::Params& params,
                                         const std::string& prefix = "");

void LoadCombinedNavStateNodeAdderModelParams(config_reader::ConfigReader& config,
                                              node_adders::CombinedNavStateNodeAdderModelParams& params,
                                              const std::string& prefix = "");

void LoadPoseNodeAdderParams(config_reader::ConfigReader& config, node_adders::PoseNodeAdderParams& params,
                             const std::string& prefix = "");

// Implementation
template <typename NodeType>
void LoadBaseTimestampedNodeAdderParams(config_reader::ConfigReader& config,
                                        node_adders::TimestampedNodeAdderParams<NodeType>& params,
                                        const std::string& prefix) {
  // Note, starting measurement and time should be set elsewhere, typically using measurements
  LOAD_PARAM(params.huber_k, config, prefix);
  LOAD_PARAM(params.add_priors, config, prefix);
  LOAD_PARAM(params.ideal_duration, config, prefix);
  LOAD_PARAM(params.min_num_states, config, prefix);
  LOAD_PARAM(params.max_num_states, config, prefix);
}
}  // namespace parameter_reader

#endif  // PARAMETER_READER_NODE_ADDERS_H_
