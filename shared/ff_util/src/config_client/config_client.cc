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

// Dynamic reconfiguration support
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>

#include <vector>

namespace ff_util {

ConfigClient::ConfigClient(ros::NodeHandle *platform_nh, const std::string &node)
  : nh_(*platform_nh, PRIVATE_PREFIX + node) {
  service_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);
}

ConfigClient::~ConfigClient() {}

// Templated functions

template<typename T>
bool ConfigClient::Get(const std::string &name, T &value) {
  ROS_WARN_STREAM("Invalid type for parameter " << name);
  return false;
}

template<typename T>
bool ConfigClient::Set(const std::string &name, const T &value) {
  ROS_WARN_STREAM("Invalid type for parameter " << name);
  return false;
}


// Template specialization : boolean

template<>
bool ConfigClient::Get(const std::string &name, bool &value) {
  if (!nh_.getParam(name, value)) {
    ROS_WARN_STREAM("Could not get parameter " << name);
    return false;
  }
  return true;
}

template<>
bool ConfigClient::Set(const std::string &name, const bool &value) {
  dynamic_reconfigure::BoolParameter p;
  p.name = name;
  p.value = value;
  request_.config.bools.push_back(p);
  return true;
}

// Template specialization : int

template<>
bool ConfigClient::Get(const std::string &name, int &value) {
  if (!nh_.getParam(name, value)) {
    ROS_WARN_STREAM("Could not get parameter " << name);
    return false;
  }
  return true;
}

template<>
bool ConfigClient::Set(const std::string &name, const int &value) {
  dynamic_reconfigure::IntParameter p;
  p.name = name;
  p.value = value;
  request_.config.ints.push_back(p);
  return true;
}

// Template specialization : string

template<>
bool ConfigClient::Get(const std::string &name, std::string &value) {
  if (!nh_.getParam(name, value)) {
    ROS_WARN_STREAM("Could not get parameter " << name);
    return false;
  }
  return true;
}

template<>
bool ConfigClient::Set(const std::string &name, const std::string &value) {
  dynamic_reconfigure::StrParameter p;
  p.name = name;
  p.value = value;
  request_.config.strs.push_back(p);
  return true;
}

// Template specialization : doubles

template<>
bool ConfigClient::Get(const std::string &name, double &value) {
  if (!nh_.getParam(name, value)) {
    ROS_WARN_STREAM("Could not get parameter " << name);
    return false;
  }
  return true;
}

template<>
bool ConfigClient::Set(const std::string &name, const double &value) {
  dynamic_reconfigure::DoubleParameter p;
  p.name = name;
  p.value = value;
  request_.config.doubles.push_back(p);
  return true;
}

// Reconfigure calls

bool ConfigClient::Reconfigure() {
  if (!service_.isValid()) {
    ROS_ERROR_STREAM("Reconfigure service is not valid.");
    return false;
  }

  // Check that the reconfigure service exists
  if (!service_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR_STREAM("Reconfigure request timed out");
    return false;
  }
  // Call the reconfigure service
  dynamic_reconfigure::ReconfigureResponse response;
  if (service_.call(request_, response)) {
    request_.config.ints.clear();
    request_.config.bools.clear();
    request_.config.doubles.clear();
    request_.config.strs.clear();
    return true;
  }
  ROS_ERROR_STREAM("Reconfigure request denied");
  return false;
}

}  // namespace ff_util

