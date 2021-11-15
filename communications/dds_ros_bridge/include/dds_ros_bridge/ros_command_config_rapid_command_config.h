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

#ifndef DDS_ROS_BRIDGE_ROS_COMMAND_CONFIG_RAPID_COMMAND_CONFIG_H_
#define DDS_ROS_BRIDGE_ROS_COMMAND_CONFIG_RAPID_COMMAND_CONFIG_H_

#include <ros/ros.h>

#include <sstream>
#include <string>

#include "config_reader/config_reader.h"

#include "dds_ros_bridge/rapid_pub.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidDds/CommandConfig.h"
#include "rapidDds/CommandConfigSupport.h"
#include "rapidDds/RapidConstants.h"

#include "rapidUtil/RapidHelper.h"

#include "dds_msgs/AstrobeeCommandConstants.h"

namespace ff {

/**
 * @brief publish a single CommandConfig message
 * @details publish a single CommandConfig message, populated
 *          with available commands from params in a launch file.
 *          This class must be kept in scope, to ensure config message
 *          remains reliable durable
 */
class RosCommandConfigRapidCommandConfig : public RapidPub {
 public:
  explicit RosCommandConfigRapidCommandConfig(const std::string& pub_topic,
                                  const ros::NodeHandle &nh,
                                  config_reader::ConfigReader& config_params);

 protected:
  /**
   * Set command config from ros params in launch file
   * this is fairly ugly and easily allows format errors
   *
   * @param config config from DdsSupplier
   * @return 0 on success, -1 on error
   */
  bool AssembleConfig(rapid::CommandConfig& config,
                     config_reader::ConfigReader& config_params);

  kn::DdsTypedSupplier<rapid::CommandConfig> command_config_supplier_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_COMMAND_CONFIG_RAPID_COMMAND_CONFIG_H_
