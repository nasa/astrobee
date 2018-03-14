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


#ifndef DDS_ROS_BRIDGE_ROS_BATTERY_STATE_H_
#define DDS_ROS_BRIDGE_ROS_BATTERY_STATE_H_

#include <string>
#include <cstring>
#include <memory>

#include "knDds/DdsTypedSupplier.h"

#include "dds_ros_bridge/enum_helper.h"
#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_hw_msgs/EpsBatteryLocation.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Temperature.h"

#include "AstrobeeConstants.h"
#include "EpsStateSupport.h"
#include "EpsConfigSupport.h"

#include "rapidUtil/RapidHelper.h"

namespace ff {

class RosBatteryStateToRapid : public RosSubRapidPub {
 public:
  RosBatteryStateToRapid(std::string const& sub_topic_battery_state_TL,
                         std::string const& sub_topic_battery_state_TR,
                         std::string const& sub_topic_battery_state_BL,
                         std::string const& sub_topic_battery_state_BR,
                         std::string const& sub_topic_battery_temp_TL,
                         std::string const& sub_topic_battery_temp_TR,
                         std::string const& sub_topic_battery_temp_BL,
                         std::string const& sub_topic_battery_temp_BR,
                         std::string const& pub_topic,
                         ros::NodeHandle const& nh,
                         const unsigned int queue_size = 10);

  void AddTempToState(rapid::ext::astrobee::BatterySlot slot, float temp);
  rapid::ext::astrobee::BatterySlot ConvertBatteryLoc(std::string const& slot);
  void SetBatteryTimeMultiple(int multiple);
  void StateCallback(sensor_msgs::BatteryStateConstPtr const& state);
  void TempTLCallback(sensor_msgs::TemperatureConstPtr const& temp);
  void TempTRCallback(sensor_msgs::TemperatureConstPtr const& temp);
  void TempBLCallback(sensor_msgs::TemperatureConstPtr const& temp);
  void TempBRCallback(sensor_msgs::TemperatureConstPtr const& temp);

 private:
  using ConfigSupplier = kn::DdsTypedSupplier<rapid::ext::astrobee::EpsConfig>;
  using ConfigSupplierPtr = std::unique_ptr<ConfigSupplier>;

  ConfigSupplierPtr config_supplier_;

  using StateSupplier = kn::DdsTypedSupplier<rapid::ext::astrobee::EpsState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr state_supplier_;

  ros::Subscriber sub_battery_state_tl_, sub_battery_state_tr_;
  ros::Subscriber sub_battery_state_bl_, sub_battery_state_br_;
  ros::Subscriber sub_battery_temp_tl_, sub_battery_temp_tr_;
  ros::Subscriber sub_battery_temp_bl_, sub_battery_temp_br_;

  int battery_time_multiple_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_BATTERY_STATE_H_
