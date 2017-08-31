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

#include <vector>
#include <string>

#include "dds_ros_bridge/rapid_agent_provider_ros_helper.h"

#include "rapidUtil/RapidHelper.h"

namespace rapid {
AgentProviderRosHelper::AgentProviderRosHelper(
    AgentTopicPairParameters const& params, const std::string& entityName)
  : AgentProvider(params, entityName) {}

void AgentProviderRosHelper::Publish(
                          std::vector<rapid::ParameterUnion> const& values) {
  rapid::AgentState& agentState = m_dataSupplier.event();

  // update header with current time
  rapid::RapidHelper::updateHeader(agentState.hdr);

  // set status code
  agentState.hdr.statusCode = 0;
  agentState.hdr.serial = 0;

  // set values
  agentState.values.ensure_length(values.size(), 64);
  for (int i = 0; i < agentState.values.length(); ++i) {
    agentState.values[i] = values[i];
  }

  m_dataSupplier.sendEvent();
}
}  // end namespace rapid
