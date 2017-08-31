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

#include <ros/ros.h>

#include <ff_util/ff_names.h>

#include <dynamic_reconfigure/Reconfigure.h>

#include <string>
#include <list>

namespace ff_util {
class ConfigClient {
 public:
  // Constructor and destructor
  explicit ConfigClient(ros::NodeHandle *platform_nh, const std::string &node);
  virtual ~ConfigClient();
  // Call a reconfigure with all set variables
  bool Reconfigure();
  // Getters and Setters
  template<typename T> bool Set(const std::string &name, const T &value);
  template<typename T> bool Get(const std::string &name, T &value);
  template<typename T> T Get(const std::string &name) {
    T tmp;
    if (!Get(name, tmp)) {
      ROS_ERROR_STREAM("Cannot query parameter " << name);
    }
    return tmp;
  }
 private:
  // Private members
  ros::NodeHandle nh_;
  ros::ServiceClient service_;
  dynamic_reconfigure::ReconfigureRequest request_;
};

template<> bool ConfigClient::Get<int>(const std::string &name, int &value);
template<> bool ConfigClient::Get<double>(const std::string &name, double &value);
template<> bool ConfigClient::Get<bool>(const std::string &name, bool &value);
template<> bool ConfigClient::Get<std::string>(const std::string &name, std::string &value);

template<> bool ConfigClient::Set<int>(const std::string &name, const int &value);
template<> bool ConfigClient::Set<double>(const std::string &name, const double &value);
template<> bool ConfigClient::Set<bool>(const std::string &name, const bool &value);
template<> bool ConfigClient::Set<std::string>(const std::string &name, const std::string &value);

}  // namespace ff_util

#endif  // FF_UTIL_CONFIG_CLIENT_H_
