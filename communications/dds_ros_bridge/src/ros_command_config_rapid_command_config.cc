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

#include <ros/ros.h>

#include <string>
#include <sstream>

#include "dds_ros_bridge/ros_command_config_rapid_command_config.h"

#include "rapidDds/RapidConstants.h"
#include "rapidDds/CommandConfigSupport.h"
#include "rapidUtil/RapidHelper.h"
#include "knDds/DdsTypedSupplier.h"
#include "AstrobeeCommandConstants.h"

namespace ff {

RosCommandConfigRapidCommandConfig::RosCommandConfigRapidCommandConfig(
    const std::string& pubTopic, const ros::NodeHandle &nh,
    config_reader::ConfigReader& config_params)
  : RapidPub(pubTopic),
    m_commandConfigSupplier_(
      rapid::COMMAND_CONFIG_TOPIC + m_publishTopic_, "",
      "RapidCommandConfigProfile", "", "") {
  // TODO(all): confirm topic suffix has "-"
  // may need to set profile to RapidCommandConfigProfile

  // supplier must be kept in scope to allow for reliable durable transmission
  // kn::DdsTypedSupplier<rapid::CommandConfig> commandConfigSupplier(
  //   rapid::COMMAND_CONFIG_TOPIC + m_publishTopic,
  //   "", /*publisher name*/
  //   "RapidCommandConfigProfile", /*profile*/
  //   "", /*library*/
  //   ""  /*entity_name*/);


  if (AssembleConfig(m_commandConfigSupplier_.event(), config_params)) {
    m_commandConfigSupplier_.sendEvent();
  }
}

bool RosCommandConfigRapidCommandConfig::AssembleConfig(
                            rapid::CommandConfig& config,
                            config_reader::ConfigReader& config_params) {
  std::string tempName;
  std::string tempKey;
  std::string tempType;

  config_reader::ConfigReader::Table cmd_config(&config_params,
                                                "commandConfig");
  rapid::RapidHelper::initHeader(config.hdr);

  // Extract Available Subsystems
  config_reader::ConfigReader::Table avail_subsys(&cmd_config,
                                                  "availableSubsystems");
  int num_avail_subsys = avail_subsys.GetSize();
  config.availableSubsystems.length(num_avail_subsys);

  // Extract subsystem names and subsystem type names
  for (int i = 0; i < num_avail_subsys; i++) {
    config_reader::ConfigReader::Table subsys(&avail_subsys, (i + 1));

    // Get subsystem name
    if (!subsys.GetStr("name", &tempName)) {
      ROS_FATAL("DDS Bridge: name not listed for avail subsys %i!", i);
      return false;
    }

    strncpy(config.availableSubsystems[i].name, tempName.data(), 32);
    config.availableSubsystems[i].name[31] = '\0';

    // Get subsystem type name
    if (!subsys.GetStr("subsystemTypeName", &tempType)) {
      ROS_FATAL("DDS Bridge: type name not listed for avail subsys %i!", i);
      return false;
    }

    strncpy(config.availableSubsystems[i].subsystemTypeName, tempType.data(),
                                                                            32);
    config.availableSubsystems[i].subsystemTypeName[31] = '\0';
  }

  // Extract Available subsystem types
  config_reader::ConfigReader::Table avail_subsys_types(&cmd_config,
                                                    "availableSubsystemTypes");
  int num_avail_subsys_types = avail_subsys_types.GetSize();
  config.availableSubsystemTypes.length(num_avail_subsys_types);

  // Extract subsystem commands
  for (int i = 0; i < num_avail_subsys_types; i++) {
    config_reader::ConfigReader::Table subsys_type(&avail_subsys_types, (i +1));

    // Get subsystem type name
    if (!subsys_type.GetStr("name", &tempName)) {
      ROS_FATAL("DDS Bridge: name not listed for avail subsys type %i!", i);
      return false;
    }

    strncpy(config.availableSubsystemTypes[i].name, tempName.data(),  32);
    config.availableSubsystemTypes[i].name[31] = '\0';

    // Get commands
    config_reader::ConfigReader::Table commands(&subsys_type, "commands");

    int num_commands = commands.GetSize();
    config.availableSubsystemTypes[i].commands.length(num_commands);

    for (int j = 0; j < num_commands; j++) {
      config_reader::ConfigReader::Table command(&commands, (j + 1));
      rapid::CommandDef &cmd_def =
                                  config.availableSubsystemTypes[i].commands[j];

      // Get command name
      if (!command.GetStr("name", &tempName)) {
        ROS_FATAL("DDS Bridge: name not listed for cmd %i in subsys type %s",
                                    j, config.availableSubsystemTypes[i].name);
        return false;
      }

      strncpy(cmd_def.name, tempName.data(), 64);
      cmd_def.name[63] = '\0';

      // Get parameters
      config_reader::ConfigReader::Table params(&command, "parameters");
      int num_params = params.GetSize();
      // There can only be 16 parameters at most
      if (num_params > 16) {
        ROS_FATAL("DDS Bridge: too many parameters listed for cmd %s!",
                                                              tempName.c_str());
        return false;
      }
      cmd_def.parameters.length(num_params);

      for (int k = 0; k < num_params; k++) {
        config_reader::ConfigReader::Table param(&params, (k + 1));

        // Get key
        if (!param.GetStr("key", &tempKey)) {
          ROS_FATAL("DDS Bridge: key not listed for param %i for command %s!",
                                                          k, tempName.c_str());
          return false;
        }

        strncpy(cmd_def.parameters[k].key, tempKey.data(), 32);
        cmd_def.parameters[k].key[31] = '\0';

        // Get type
        if (!param.GetStr("type", &tempType)) {
          ROS_FATAL("DDS Bridge: type not listed for param %i for command %s!",
                                                          k, tempName.c_str());
          return false;
        }

        // check RAPID type
        if (tempType == "RAPID_STRING") {
          cmd_def.parameters[k].type = rapid::RAPID_STRING;
        } else if (tempType == "RAPID_INT") {
          cmd_def.parameters[k].type = rapid::RAPID_INT;
        } else if (tempType == "RAPID_BOOL") {
          cmd_def.parameters[k].type = rapid::RAPID_BOOL;
        } else if (tempType == "RAPID_FLOAT") {
          cmd_def.parameters[k].type = rapid::RAPID_FLOAT;
        } else if (tempType == "RAPID_VEC3d") {
          cmd_def.parameters[k].type = rapid::RAPID_VEC3d;
        } else if (tempType == "RAPID_MAT33f") {
          cmd_def.parameters[k].type = rapid::RAPID_MAT33f;
        } else {
          ROS_FATAL("DDS Bridge: %s is invalid param type.", tempType.c_str());
          return false;
        }
      }
    }
  }

  return true;
}

}  // end namespace ff
