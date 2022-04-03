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

#ifndef GROUND_DDS_ROS_BRIDGE_ROS_COMMAND_H_
#define GROUND_DDS_ROS_BRIDGE_ROS_COMMAND_H_

#include <cstring>
#include <string>
#include <memory>

#include "ground_dds_ros_bridge/ros_sub_rapid_pub.h"
#include "ground_dds_ros_bridge/util.h"

#include "ff_msgs/CommandArg.h"
#include "ff_msgs/CommandStamped.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidUtil/RapidHelper.h"

#include "rapidDds/CommandSupport.h"
#include "rapidDds/RapidConstants.h"

namespace ff {

class RosCommandToRapid : public RosSubRapidPub {
 public:
  RosCommandToRapid(const std::string& subscribe_topic,
                    const std::string& pub_topic,
                    const std::string& connecting_robot,
                    const ros::NodeHandle &nh,
                    const unsigned int queue_size = 10);

  void CmdCallback(ff_msgs::CommandStampedConstPtr const& cmd);
 private:
  using CommandSupplier = kn::DdsTypedSupplier<rapid::Command>;
  using CommandSupplierPtr = std::unique_ptr<CommandSupplier>;

  CommandSupplierPtr command_supplier_;
};

}  // end namespace ff

#endif  // GROUND_DDS_ROS_BRIDGE_ROS_COMMAND_H_
