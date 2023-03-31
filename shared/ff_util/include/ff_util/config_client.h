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

#ifndef FF_UTIL_CONFIG_CLIENT_H_
#define FF_UTIL_CONFIG_CLIENT_H_

#include <ff_common/ff_ros.h>
#include <ff_common/ff_names.h>

#include <string>
#include <list>

namespace ff_util {

class ConfigClient {
 public:
  // Constructor and destructor
  explicit ConfigClient(NodeHandle & platform_nh, const std::string &node);
  virtual ~ConfigClient();
  // Call a reconfigure with all set variables
  bool Reconfigure();
  // Getters and Setters
  template<typename T>
  bool Get(const std::string &name, T &value) {
    if (!parameters_client_->has_parameter(name)) {
        Error("Cannot query parameter " + name);
        return false;
    }
    value = parameters_client_->get_parameter<T>(name);
    return true;
  }

  template<typename T>
  bool Set(const std::string &name, const T &value) {
    rclcpp::Parameter p(name, value);
    auto result = parameters_client_->set_parameters({p});
    if (result.size() != 1 || !result[0].successful) {
      Error("Failed to set parameter " + name);
      return false;
    }
    return true;
  }

  template<typename T>
  T Get(const std::string &name) {
    T tmp;
    if (!Get(name, tmp)) {
      Error("Cannot query parameter " + name);
    }
    return tmp;
  }

 private:
  void Error(std::string out);
  // Private members
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
};
}  // namespace ff_util

#endif  // FF_UTIL_CONFIG_CLIENT_H_
