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
#include <ff_util/config_server.h>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

FF_DEFINE_LOGGER("config_server");

namespace ff_util {

// Constructor and destructor

ConfigServer::ConfigServer() :  initialized_(false) {
}

ConfigServer::~ConfigServer() {}

// External functions


void ConfigServer::SetPath(const std::string &pname) {
  if (initialized_) {
    FF_WARN_STREAM("Cannot set path " << pname << " to ConfigServer after listening has started");
  }
  config_params_.SetPath(pname.c_str());
}

void ConfigServer::AddFile(const std::string &fname) {
  if (initialized_) {
    FF_WARN_STREAM("Cannot add file " << fname << " to ConfigServer after listening has started");
  }
  config_params_.AddFile(fname.c_str());
}

// must AddFile before this
bool ConfigServer::Initialize(NodeHandle & private_nh) {
  // Copy the node handle
  nh_ = &private_nh;
  // Read all of the config files
  config_params_.ReadFiles();
  // Parse the config file for reconfigurable metadata
  config_reader::ConfigReader::Table table;
  if (!config_params_.GetTable("parameters", &table)) {
    FF_WARN_STREAM("There are no reconfigurable parameters");
    return false;
  }
  for (int i = 0; i < table.GetSize(); i++) {
    // Get the row
    config_reader::ConfigReader::Table group;
    if (!table.GetTable(i + 1, &group)) {
      FF_ERROR_STREAM("Could not read parameter table row" << i + 1);
      continue;
    }
    rcl_interfaces::msg::ParameterDescriptor d;
    // Get the name / ID of the parameter
    std::string name;
    if (!group.GetStr("id", &name)) {
      FF_ERROR_STREAM("Could not read ID for row" << i + 1);
      continue;
    }
    // Is this parameter reconfigurable?
    bool reconfigurable;
    if (!group.GetBool("reconfigurable", &reconfigurable)) {
      FF_ERROR_STREAM("Could not find a id field for parameter reconfigurable.");
      continue;
    }
    // What is the type of th paramter
    std::string type;
    if (!group.GetStr("type", &type)) {
      FF_ERROR_STREAM("Could not find a type field for parameter type.");
      continue;
    }

    d.name = name;
    d.read_only = !reconfigurable;
    group.GetStr("unit", &d.additional_constraints);
    group.GetStr("description", &d.description);

    // *** STRINGS *** //
    if (!type.compare("string")) {
      std::string default_value;
      if (!group.GetStr("default", &default_value)) {
        FF_WARN_STREAM("Could not find a default value for " << name);
      }
      nh_->get()->declare_parameter(d.name, default_value, d);
    } else if (!type.compare("integer")) {
      int default_value;
      if (!group.GetInt("default", &default_value)) {
        FF_WARN_STREAM("Could not get the default value for parameter " << name);
      }
      rcl_interfaces::msg::IntegerRange range;
      int d1, d2;
      if (reconfigurable && !group.GetInt("min", &d1) && !group.GetInt("max", &d2)) {
        range.from_value = d1;
        range.to_value = d2;
        d.integer_range.push_back(range);
      }
      nh_->get()->declare_parameter(d.name, default_value, d);
    } else if (!type.compare("boolean")) {
      bool default_value;
      if (!group.GetBool("default", &default_value)) {
        FF_WARN_STREAM("Could not get the default value for parameter " << name);
      }
      nh_->get()->declare_parameter(d.name, default_value, d);
    } else if (!type.compare("double")) {
      double default_value;
      if (!group.GetReal("default", &default_value)) {
        FF_WARN_STREAM("Could not get the default value for parameter " << name);
      }
      rcl_interfaces::msg::FloatingPointRange range;
      if (reconfigurable && !group.GetReal("min", &range.from_value) && !group.GetReal("max", &range.to_value))
        d.floating_point_range.push_back(range);
      nh_->get()->declare_parameter(d.name, default_value, d);
    } else {
      FF_ERROR_STREAM("Could not understand type " << type << " of parameter " << name);
      continue;
    }
  }

  initialized_ = true;

  return true;
}

// Get the ConfigReader pointer for advanced usage
config_reader::ConfigReader* ConfigServer::GetConfigReader() {
  if (!initialized_) {
    FF_ERROR_STREAM("Cannot get the ConfigReader handle until the system is listening");
    return nullptr;
  }
  return &config_params_;
}

void ConfigServer::Error(std::string s) {
  FF_ERROR_STREAM(s);
}

}  // namespace ff_util

