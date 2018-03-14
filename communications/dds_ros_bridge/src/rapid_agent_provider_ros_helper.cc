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

#include "dds_ros_bridge/rapid_agent_provider_ros_helper.h"

namespace rapid {
AgentProviderRosHelper::AgentProviderRosHelper(
    AgentTopicPairParameters const& params, const std::string& entity_name)
  : AgentProvider(params, entity_name) {}

void AgentProviderRosHelper::Publish(
                          std::vector<rapid::ParameterUnion> const& values) {
  rapid::AgentState& agent_state = m_dataSupplier.event();

  // update header with current time
  rapid::RapidHelper::updateHeader(agent_state.hdr);

  // set status code
  agent_state.hdr.statusCode = 0;
  agent_state.hdr.serial = 0;

  // set values
  agent_state.values.ensure_length(values.size(), 64);
  for (int i = 0; i < agent_state.values.length(); ++i) {
    agent_state.values[i] = values[i];
  }

  m_dataSupplier.sendEvent();
}
}  // end namespace rapid
