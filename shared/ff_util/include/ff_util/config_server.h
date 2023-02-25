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

#ifndef FF_UTIL_CONFIG_SERVER_H_
#define FF_UTIL_CONFIG_SERVER_H_

// ROS includes
#include <ff_common/ff_names.h>
#include <ff_common/ff_ros.h>

#include <config_reader/config_reader.h>

#include <climits>
#include <cfloat>
#include <string>
#include <utility>
#include <vector>
#include <map>

namespace ff_util {

class ConfigServer {
 public:
  // Constructor opens the ASTROBEE_CONFIG_DIR/subsystem/node.config file by default
  ConfigServer();
  virtual ~ConfigServer();
  // Initialize the system
  bool Initialize(NodeHandle & private_nh);
  // Manually set the path to the config file
  void SetPath(const std::string &pname);
  // Add another (optional) LUA config file to the configuration
  void AddFile(const std::string &fname);
  // Get and Set after initialization
  template<typename T> bool Get(const std::string &name, T &value) {
    if (!initialized_) {
      Error("ConfigServer is not listening or parameter " + name + " does not exist");
      return false;
    }
    bool result = nh_->get()->get_parameter(name, value);
    if (!result) {
      Error("Parameter " + name + " does not exist");
      return false;
    }
    return true;
  }
  template<typename T> bool Set(const std::string &name, const T &value) {
    if (!initialized_ || !nh_->get()->has_parameter(name)) {
      Error("ConfigServer is not listening or parameter " + name + " does not exist");
      return false;
    }
    rclcpp::Parameter param(name, value);
    rcl_interfaces::msg::SetParametersResult r = nh_->get()->set_parameter(param);
    if (!r.successful) {
      Error("Failed to set parameter " + name + ".");
    }

    return r.successful;
  }
  template<typename T> T Get(const std::string &name) {
    T out = T();
    Get(name, out);
    return out;
  }
  // Get the ConfigReader pointer for advanced usage
  config_reader::ConfigReader* GetConfigReader();

 private:
  void Error(std::string out);
  bool initialized_;
  NodeHandle* nh_;
  config_reader::ConfigReader config_params_;
};
}  // namespace ff_util

#endif  // FF_UTIL_CONFIG_SERVER_H_
