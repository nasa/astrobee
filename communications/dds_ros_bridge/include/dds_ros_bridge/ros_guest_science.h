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


#ifndef DDS_ROS_BRIDGE_ROS_GUEST_SCIENCE_H_
#define DDS_ROS_BRIDGE_ROS_GUEST_SCIENCE_H_

#include <string>
#include <cstring>
#include <memory>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/GuestScienceApk.h"
#include "ff_msgs/GuestScienceCommand.h"
#include "ff_msgs/GuestScienceConfig.h"
#include "ff_msgs/GuestScienceData.h"
#include "ff_msgs/GuestScienceState.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidUtil/RapidHelper.h"

#include "AstrobeeConstants.h"
#include "GuestScienceConfigSupport.h"
#include "GuestScienceDataSupport.h"
#include "GuestScienceStateSupport.h"

namespace ff {

class RosGuestScienceToRapid : public RosSubRapidPub {
 public:
  RosGuestScienceToRapid(const std::string& state_subscribe_topic,
                         const std::string& config_subscribe_topic,
                         const std::string& data_subscribe_topic,
                         const std::string& pub_topic,
                         const ros::NodeHandle &nh,
                         const unsigned int queue_size = 10);

  void DataCallback(ff_msgs::GuestScienceDataConstPtr const& data);
  void ConfigCallback(ff_msgs::GuestScienceConfigConstPtr const& config);
  void StateCallback(ff_msgs::GuestScienceStateConstPtr const& state);

 private:
  using StateSupplier =
      kn::DdsTypedSupplier<rapid::ext::astrobee::GuestScienceState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr state_supplier_;

  using ConfigSupplier =
      kn::DdsTypedSupplier<rapid::ext::astrobee::GuestScienceConfig>;
  using ConfigSupplierPtr = std::unique_ptr<ConfigSupplier>;

  ConfigSupplierPtr config_supplier_;

  using DataSupplier =
      kn::DdsTypedSupplier<rapid::ext::astrobee::GuestScienceData>;
  using DataSupplierPtr = std::unique_ptr<DataSupplier>;

  DataSupplierPtr data_supplier_;

  ros::Subscriber config_sub_, data_sub_;

  std::string config_subscribe_topic_, data_subscribe_topic_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_GUEST_SCIENCE_H_
