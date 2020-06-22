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

// Dynamic reconfiguration support
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/GroupState.h>

namespace ff_util {

// Constructor and destructor

ConfigServer::ConfigServer() :  initialized_(false), listening_(false) {
  // Add a single group
  dynamic_reconfigure::Group group;
  group.name = "Default";
  group.type = "";
  group.parent = 0;
  group.id = 0;
  description_.groups.push_back(group);
  // Add the state of the group
  dynamic_reconfigure::GroupState gs;
  gs.name = "Default";
  gs.state = true;
  gs.parent = 0;
  gs.id = 0;
  description_.max.groups.push_back(gs);
  description_.min.groups.push_back(gs);
  description_.dflt.groups.push_back(gs);
  // Set the default callback
  callback_ = boost::bind(&ConfigServer::Reconfigure, this, _1);
}

ConfigServer::~ConfigServer() {}

// External functions

void ConfigServer::Initialize(ros::NodeHandle *private_nh, const std::string &fname) {
  if (listening_) {
    ROS_WARN_STREAM("Cannot initialize ConfigServer after listening has started");
  }
  // Copy the node handle
  nh_ = *private_nh;
  initialized_ = true;
  // Add the node's configfile
  AddFile(fname);
}

void ConfigServer::SetPath(const std::string &pname) {
  if (listening_) {
    ROS_WARN_STREAM("Cannot set path " << pname << " to ConfigServer after listening has started");
  }
  config_params_.SetPath(pname.c_str());
}

void ConfigServer::AddFile(const std::string &fname) {
  if (listening_) {
    ROS_WARN_STREAM("Cannot add file " << fname << " to ConfigServer after listening has started");
  }
  config_params_.AddFile(fname.c_str());
}

bool ConfigServer::Listen() {
  if (!initialized_) {
    ROS_WARN_STREAM("Cannot listen until ConfigServer is initialized");
    return false;
  }
  // Read all of the config files
  config_params_.ReadFiles();
  // Parse the config file for reconfigurable metadata
  config_reader::ConfigReader::Table table;
  if (!config_params_.GetTable("parameters", &table)) {
    ROS_WARN_STREAM("There are no reconfigurable parameters");
    return false;
  }
  uint32_t shift = 0;
  for (int i = 0; i < table.GetSize(); i++) {
    // Get the row
    config_reader::ConfigReader::Table group;
    if (!table.GetTable(i + 1, &group)) {
      ROS_ERROR_STREAM("Could not read parameter table row" << i + 1);
      continue;
    }
    // Get the name / ID of the parameter
    std::string id;
    if (!group.GetStr("id", &id)) {
      ROS_ERROR_STREAM("Could not read ID for row" << i + 1);
      continue;
    }
    // Is this parameter reconfigurable?
    bool reconfigurable;
    if (!group.GetBool("reconfigurable", &reconfigurable)) {
      ROS_ERROR_STREAM("Could not find a id field for parameter " << id);
      continue;
    }
    // What is the type of th paramter
    std::string type;
    if (!group.GetStr("type", &type)) {
      ROS_ERROR_STREAM("Could not find a type field for parameter " << id);
      continue;
    }
    // Try and get a unit
    std::string unit;
    if (!group.GetStr("unit", &unit))
      unit = "unknown";
    // Try and get a description
    std::string description;
    if (!group.GetStr("description", &description))
      description = "Update parameter " + id + " of type " + type + " and unit " + unit;
    // Package up the variable type
    dynamic_reconfigure::ParamDescription pd;
    pd.name = id;
    pd.level = (1 << shift++);
    pd.description = description;
    // *** STRINGS *** //
    if (!type.compare("string")) {
      if (strs_.find(id) != strs_.end()) {
        ROS_ERROR_STREAM("String variable " << id << " already declared. Ignoring.");
        continue;
      }
      if (!group.GetStr("default", &strs_[id])) {
        ROS_ERROR_STREAM("Could not get the default value for parameter " << id);
        continue;
      }
      if (reconfigurable) {
        pd.type = "str";
        dynamic_reconfigure::StrParameter tmp;
        tmp.name = id;
        tmp.value = strs_[id];
        description_.min.strs.push_back(tmp);
        description_.max.strs.push_back(tmp);
        description_.dflt.strs.push_back(tmp);
        description_.groups[0].parameters.push_back(pd);
        if (!unit.compare("enumeration")) {
          config_reader::ConfigReader::Table values;
          if (group.GetTable("values", &values)) {
            std::map<std::string, std::string> enumeration;
            for (int j = 0; j < values.GetSize(); j++) {
              config_reader::ConfigReader::Table row;
              std::string key;
              std::string name;
              if (!values.GetTable(j + 1, &row) || !row.GetStr("key", &key) || !row.GetStr("name", &name)) {
                ROS_ERROR_STREAM("Could not read value for table row " << j + 1);
                continue;
              }
              enumeration[key] = name;
            }
            if (!enumeration.empty())
              Lim<std::string>(id, enumeration);
          }
        }
      }
      nh_.getParam(id, strs_[id]);
      nh_.setParam(id, strs_[id]);
      // *** INTEGERS *** //
    } else if (!type.compare("integer")) {
      if (ints_.find(id) != ints_.end()) {
        ROS_ERROR_STREAM("String variable " << id << " already declared. Ignoring.");
        continue;
      }
      if (!group.GetInt("default", &ints_[id])) {
        ROS_ERROR_STREAM("Could not get the default value for parameter " << id);
        continue;
      }
      if (reconfigurable) {
        pd.type = "int";
        dynamic_reconfigure::IntParameter tmp;
        tmp.name = id;
        if (!group.GetInt("min", &tmp.value))
          ROS_WARN_STREAM("Could not get the min value for parameter " << id);
        description_.min.ints.push_back(tmp);
        tmp.name = id;
        if (!group.GetInt("max", &tmp.value))
          ROS_WARN_STREAM("Could not get the max value for parameter " << id);
        description_.max.ints.push_back(tmp);
        tmp.name = id;
        tmp.value = ints_[id];
        description_.dflt.ints.push_back(tmp);
        description_.groups[0].parameters.push_back(pd);
        if (!unit.compare("enumeration")) {
          config_reader::ConfigReader::Table values;
          if (group.GetTable("values", &values)) {
            std::map<int, std::string> enumeration;
            for (int j = 0; j < values.GetSize(); j++) {
              config_reader::ConfigReader::Table row;
              int key;
              std::string name;
              if (!values.GetTable(j + 1, &row) || !row.GetInt("key", &key) || !row.GetStr("name", &name)) {
                ROS_ERROR_STREAM("Could not read value for table row " << j + 1);
                continue;
              }
              enumeration[key] = name;
            }
            if (!enumeration.empty())
              Lim<int>(id, enumeration);
          }
        }
      }
      nh_.getParam(id, ints_[id]);
      nh_.setParam(id, ints_[id]);
      // *** BOOLEANS *** //
    } else if (!type.compare("boolean")) {
      if (bools_.find(id) != bools_.end()) {
        ROS_ERROR_STREAM("Boolean variable " << id << " already declared. Ignoring.");
        continue;
      }
      if (!group.GetBool("default", &bools_[id])) {
        ROS_ERROR_STREAM("Could not get the default value for parameter " << id);
        continue;
      }
      if (reconfigurable) {
        pd.type = "bool";
        dynamic_reconfigure::BoolParameter tmp;
        tmp.name = id;
        tmp.value = bools_[id];
        description_.min.bools.push_back(tmp);
        description_.max.bools.push_back(tmp);
        description_.dflt.bools.push_back(tmp);
        description_.groups[0].parameters.push_back(pd);
        if (!unit.compare("enumeration")) {
          config_reader::ConfigReader::Table values;
          if (group.GetTable("values", &values)) {
            std::map<bool, std::string> enumeration;
            for (int j = 0; j < values.GetSize(); j++) {
              config_reader::ConfigReader::Table row;
              bool key;
              std::string name;
              if (!values.GetTable(j + 1, &row) || !row.GetBool("key", &key) || !row.GetStr("name", &name)) {
                ROS_ERROR_STREAM("Could not read value for table row " << j + 1);
                continue;
              }
              enumeration[key] = name;
            }
            if (!enumeration.empty())
              Lim<bool>(id, enumeration);
          }
        }
      }
      // Grab the persistent value from the parameter server
      nh_.getParam(id, bools_[id]);
      nh_.setParam(id, bools_[id]);
      // *** DOUBLES *** //
    } else if (!type.compare("double")) {
      if (doubles_.find(id) != doubles_.end()) {
        ROS_ERROR_STREAM("Double variable " << id << " already declared. Ignoring.");
        continue;
      }
      if (!group.GetReal("default", &doubles_[id])) {
        ROS_ERROR_STREAM("Could not get the default value for parameter " << id);
        continue;
      }
      if (reconfigurable) {
        pd.type = "double";
        dynamic_reconfigure::DoubleParameter tmp;
        tmp.name = id;
        if (!group.GetReal("min", &tmp.value))
          ROS_WARN_STREAM("Could not get the min value for parameter " << id);
        description_.min.doubles.push_back(tmp);
        tmp.name = id;
        if (!group.GetReal("max", &tmp.value))
          ROS_WARN_STREAM("Could not get the max value for parameter " << id);
        description_.max.doubles.push_back(tmp);
        tmp.name = id;
        tmp.value = doubles_[id];
        description_.dflt.doubles.push_back(tmp);
        description_.groups[0].parameters.push_back(pd);
        config_reader::ConfigReader::Table values;
        if (!unit.compare("enumeration")) {
          if (group.GetTable("values", &values)) {
            std::map<double, std::string> enumeration;
            for (int j = 0; j < values.GetSize(); j++) {
              config_reader::ConfigReader::Table row;
              double key;
              std::string name;
              if (!values.GetTable(j + 1, &row) || !row.GetReal("key", &key) || !row.GetStr("name", &name)) {
                ROS_ERROR_STREAM("Could not read value for table row " << j + 1);
                continue;
              }
              enumeration[key] = name;
            }
            if (!enumeration.empty())
              Lim<double>(id, enumeration);
          }
        }
      }
      nh_.getParam(id, doubles_[id]);
      nh_.setParam(id, doubles_[id]);
    } else {
      ROS_ERROR_STREAM("Could not understand type " << type << " of parameter " << id);
      continue;
    }
  }

  // Dynamic_reconfigure compatibility
  pub_d_ = nh_.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
  pub_u_ = nh_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);
  srv_r_ = nh_.advertiseService("set_parameters", &ConfigServer::ReconfigureImpl, this);

  // Update the parameter descriptions and initial values
  dynamic_reconfigure::ConfigDescription description = UpdateDescription();
  dynamic_reconfigure::Config config = UpdateValues();

  // We can now Get and Set the variables
  listening_ = true;

  // Success!
  return true;
}

// Assemble an edit method
std::string ConfigServer::EditMethod(const std::map<std::string, std::string> &values,
                                     const std::string &ctype, const std::string &cpptype,
                                     const std::string &cconsttype) {
  std::stringstream ss;
  ss << "{'enum_description': 'Possible options', 'enum': [";
  std::map<std::string, std::string>::const_iterator it;
  for (it = values.begin(); it != values.end(); it++) {
    ss << "{";
    ss << "'type': '" << ctype << "', ";
    ss << "'ctype': '" << cpptype << "', ";
    ss << "'cconsttype': '" << cconsttype << "', ";
    ss << "'value': " << it->first << ",";
    ss << "'name': '" << it->second << "'";
    ss << "}";
    if (std::distance(it, values.end()) > 1)
      ss << ",";
  }
  ss << "]}";
  return ss.str();
}

bool ConfigServer::Listen(ConfigCallback f) {
  callback_ = f;
  return Listen();
}

// Get the ConfigReader pointer for advanced usage
config_reader::ConfigReader* ConfigServer::GetConfigReader() {
  if (!listening_) {
    ROS_ERROR_STREAM("Cannot get the ConfigReader handle until the system is listening");
    return nullptr;
  }
  return &config_params_;
}

// Internal functions

bool ConfigServer::ReconfigureImpl(dynamic_reconfigure::Reconfigure::Request  &req,
                                   dynamic_reconfigure::Reconfigure::Response &res) {
  return callback_(req.config);
}

dynamic_reconfigure::ConfigDescription ConfigServer::UpdateDescription() {
  pub_d_.publish(description_);
  return description_;
}

dynamic_reconfigure::Config ConfigServer::UpdateValues() {
  dynamic_reconfigure::Config config;
  for (std::vector<dynamic_reconfigure::ParamDescription>::const_iterator it =
         description_.groups[0].parameters.begin(); it != description_.groups[0].parameters.end(); it++) {
    if (it->type == "bool") {
      dynamic_reconfigure::BoolParameter dp;
      dp.name = it->name;
      dp.value = bools_[it->name];
      config.bools.push_back(dp);
    }
    if (it->type == "int") {
      dynamic_reconfigure::IntParameter dp;
      dp.name = it->name;
      dp.value = ints_[it->name];
      config.ints.push_back(dp);;
    }
    if (it->type == "str") {
      dynamic_reconfigure::StrParameter dp;
      dp.name = it->name;
      dp.value = strs_[it->name];
      config.strs.push_back(dp);
    }
    if (it->type == "double") {
      dynamic_reconfigure::DoubleParameter dp;
      dp.name = it->name;
      dp.value = doubles_[it->name];
      config.doubles.push_back(dp);
    }
  }
  pub_u_.publish(config);
  return config;
}

// Templated functions

template<typename T>
bool ConfigServer::Get(const std::string &name, T &value) {
  ROS_WARN_STREAM("Invalid parameter type");
  return false;
}

template<typename T>
bool ConfigServer::Set(const std::string &name, const T &value) {
  ROS_WARN_STREAM("Invalid parameter type");
  return false;
}

template<typename T>
bool ConfigServer::Lim(const std::string &name, const std::map<T, std::string> &vals) {
  ROS_WARN_STREAM("Invalid parameter type");
  return false;
}

// Template specialization : boolean

template<>
bool ConfigServer::Get<bool>(const std::string &name, bool &value) {
  if (bools_.find(name) == bools_.end() || !listening_) {
    ROS_WARN_STREAM("ConfigServer is not listening or parameter " << name << " does not exist");
    return false;
  }
  value = bools_[name];
  return true;
}

template<>
bool ConfigServer::Set<bool>(const std::string &name, const bool &value) {
  if (bools_.find(name) == bools_.end() || !listening_) {
    ROS_WARN_STREAM("ConfigServer is not listening or parameter " << name << " does not exist");
    return false;
  }
  bools_[name] = value;
  nh_.setParam(name, bools_[name]);
  UpdateValues();
  return true;
}

template<>
bool ConfigServer::Lim<bool>(const std::string &name, const std::map<bool, std::string> &vals) {
  std::map<std::string, std::string> stringified;
  std::map<bool, std::string>::const_iterator jt;
  for (jt = vals.begin(); jt != vals.end(); jt++)
    stringified[std::to_string(jt->first)] = jt->second;
  std::vector<dynamic_reconfigure::ParamDescription>::iterator it;
  for (it = description_.groups[0].parameters.begin(); it != description_.groups[0].parameters.end(); it++) {
    if (it->name.compare(name) || it->type.compare("bool"))
      continue;
    it->edit_method = EditMethod(stringified, "bool", "bool", "const bool");
    UpdateDescription();
    return true;
  }
  return false;
}

// Template specialization : integer

template<>
bool ConfigServer::Get<int>(const std::string &name, int &value) {
  if (ints_.find(name) == ints_.end() || !listening_) {
    ROS_WARN_STREAM("ConfigServer is not listening or parameter " << name << " does not exist");
    return false;
  }
  value = ints_[name];
  return true;
}

template<>
bool ConfigServer::Set<int>(const std::string &name, const int &value) {
  if (ints_.find(name) == ints_.end() || !listening_) {
    ROS_WARN_STREAM("ConfigServer is not listening or parameter " << name << " does not exist");
    return false;
  }
  ints_[name] = value;
  nh_.setParam(name, ints_[name]);
  UpdateValues();
  return true;
}

template<>
bool ConfigServer::Lim<int>(const std::string &name, const std::map<int, std::string> &vals) {
  std::map<std::string, std::string> stringified;
  std::map<int, std::string>::const_iterator jt;
  for (jt = vals.begin(); jt != vals.end(); jt++)
    stringified[std::to_string(jt->first)] = jt->second;
  std::vector<dynamic_reconfigure::ParamDescription>::iterator it;
  for (it = description_.groups[0].parameters.begin(); it != description_.groups[0].parameters.end(); it++) {
    if (it->name.compare(name) || it->type.compare("int"))
      continue;
    it->edit_method = EditMethod(stringified, "int", "int", "const int");
    UpdateDescription();
    return true;
  }
  return false;
}

// Template specialization : double

template<>
bool ConfigServer::Get<double>(const std::string &name, double &value) {
  if (doubles_.find(name) == doubles_.end() || !listening_) {
    ROS_WARN_STREAM("ConfigServer is not listening or parameter " << name << " does not exist");
    return false;
  }
  value = doubles_[name];
  return true;
}

template<>
bool ConfigServer::Set<double>(const std::string &name, const double &value) {
  if (doubles_.find(name) == doubles_.end() || !listening_) {
    ROS_WARN_STREAM("ConfigServer is not listening or parameter " << name << " does not exist");
    return false;
  }
  doubles_[name] = value;
  nh_.setParam(name, doubles_[name]);
  UpdateValues();
  return true;
}

template<>
bool ConfigServer::Lim<double>(const std::string &name, const std::map<double, std::string> &vals) {
  std::map<std::string, std::string> stringified;
  std::map<double, std::string>::const_iterator jt;
  for (jt = vals.begin(); jt != vals.end(); jt++)
    stringified[std::to_string(jt->first)] = jt->second;
  std::vector<dynamic_reconfigure::ParamDescription>::iterator it;
  for (it = description_.groups[0].parameters.begin(); it != description_.groups[0].parameters.end(); it++) {
    if (it->name.compare(name) || it->type.compare("double"))
      continue;
    it->edit_method = EditMethod(stringified, "double", "double", "const double");
    UpdateDescription();
    return true;
  }
  return false;
}

// Template specialization : string

template<>
bool ConfigServer::Get<std::string>(const std::string &name, std::string &value) {
  if (strs_.find(name) == strs_.end() || !listening_) {
    ROS_WARN_STREAM("ConfigServer is not listening or parameter " << name << " does not exist");
    return false;
  }
  value = strs_[name];
  return true;
}

template<>
bool ConfigServer::Set<std::string>(const std::string &name, const std::string &value) {
  if (strs_.find(name) == strs_.end() || !listening_) {
    ROS_WARN_STREAM("ConfigServer is not listening or parameter " << name << " does not exist");
    return false;
  }
  strs_[name] = value;
  nh_.setParam(name, strs_[name]);
  UpdateValues();
  return true;
}

template<>
bool ConfigServer::Lim<std::string>(const std::string &name, const std::map<std::string, std::string> &vals) {
  std::map<std::string, std::string> stringified;
  std::map<std::string, std::string>::const_iterator jt;
  for (jt = vals.begin(); jt != vals.end(); jt++)
    stringified["'" + jt->first + "'"] = jt->second;
  std::vector<dynamic_reconfigure::ParamDescription>::iterator it;
  for (it = description_.groups[0].parameters.begin(); it != description_.groups[0].parameters.end(); it++) {
    if (it->name.compare(name) || it->type.compare("str"))
      continue;
    it->edit_method = EditMethod(stringified, "str", "std::string", "const char * const");
    UpdateDescription();
    return true;
  }
  return false;
}

// Dump all parameters into a KeyValue message to be used in diagnostics
std::vector<diagnostic_msgs::KeyValue> ConfigServer::Dump() {
  std::vector<diagnostic_msgs::KeyValue> keyval;
  diagnostic_msgs::KeyValue kv;
  for (std::map<std::string, int>::iterator it = ints_.begin(); it != ints_.end(); it++) {
    kv.key = it->first;
    kv.value = std::to_string(it->second);
    keyval.push_back(kv);
  }
  for (std::map<std::string, double>::iterator it = doubles_.begin(); it != doubles_.end(); it++) {
    kv.key = it->first;
    kv.value = std::to_string(it->second);
    keyval.push_back(kv);
  }
  for (std::map<std::string, bool>::iterator it = bools_.begin(); it != bools_.end(); it++) {
    kv.key = it->first;
    kv.value = (it->second ? "TRUE" : "FALSE");
    keyval.push_back(kv);
  }
  for (std::map<std::string, std::string>::iterator it = strs_.begin(); it != strs_.end(); it++) {
    kv.key = it->first;
    kv.value = it->second;
    keyval.push_back(kv);
  }
  return keyval;
}

// Callback handling

bool ConfigServer::Reconfigure(dynamic_reconfigure::Config &config) {
  bool success = true;
  for (std::vector<dynamic_reconfigure::IntParameter>::const_iterator it =
         config.ints.begin(); it != config.ints.end(); it++) {
    if (!Set<int>(it->name, it->value)) {
      ROS_WARN_STREAM("Problem setting integer parameter " << it->name << "=" << it->value);
      success = false;
    }
  }
  for (std::vector<dynamic_reconfigure::DoubleParameter>::const_iterator it =
         config.doubles.begin(); it != config.doubles.end(); it++) {
    if (!Set<double>(it->name, it->value)) {
      ROS_WARN_STREAM("Problem setting double parameter " << it->name << "=" << it->value);
      success = false;
    }
  }
  for (std::vector<dynamic_reconfigure::StrParameter>::const_iterator it =
         config.strs.begin(); it != config.strs.end(); it++) {
    if (!Set<std::string>(it->name, it->value)) {
      ROS_WARN_STREAM("Problem setting string parameter " << it->name << "=" << it->value);
      success = false;
    }
  }
  for (std::vector<dynamic_reconfigure::BoolParameter>::const_iterator it =
         config.bools.begin(); it != config.bools.end(); it++) {
    if (!Set<bool>(it->name, it->value)) {
      ROS_WARN_STREAM("Problem setting boolean parameter " << it->name << "=" << it->value);
      success = false;
    }
  }
  return success;
}

}  // namespace ff_util

