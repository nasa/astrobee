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

// Class implementation for the choreographer
#include <ff_util/config_client.h>

#include <vector>

FF_DEFINE_LOGGER("config_client");

namespace ff_util {

ConfigClient::ConfigClient(NodeHandle & nh, const std::string &node) {
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(nh, node);
    parameters_client_->wait_for_service();
}

ConfigClient::~ConfigClient() {}

// Templated functions

template<typename T>
bool ConfigClient::Get(const std::string &name, T &value) {
  if (!parameters_client_->has_parameter(name)) {
      FF_ERROR_STREAM("Cannot query parameter " << name);
      return false;
  }
  value = parameters_client_->get_parameter<T>(name);
  return true;
}

template<typename T>
bool ConfigClient::Set(const std::string &name, const T &value) {
  rclcpp::Parameter p(name, value);
  auto result = parameters_client_->set_parameters({p});
  if (result.size() != 1 || !result[0].successful) {
    FF_WARN_STREAM("Failed to set parameter " << name);
    return false;
  }
  return true;
}

template<typename T>
T ConfigClient::Get(const std::string &name) {
  T tmp;
  if (!Get(name, tmp)) {
    FF_ERROR_STREAM("Cannot query parameter " << name);
  }
  return tmp;
}

}  //  namespace ff_util
