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


#ifndef EPS_SIM_EPS_SIM_H_
#define EPS_SIM_EPS_SIM_H_

#include <pluginlib/class_list_macros.h>

// ROS includes
#include <ros/ros.h>

// FSW standard naming
#include <ff_util/ff_names.h>

// Battery services
#include <eps_sim/AddRemoveBattery.h>

// Standard messages
#include <ff_hw_msgs/EpsBatteryLocation.h>
#include <ff_hw_msgs/EpsChannelState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>

#include <ff_util/ff_nodelet.h>

// C++ STL includes
#include <string>

#define MESSAGE_QUEUE_SIZE 10
#define MAX_TEMPERATURE 42

namespace eps_sim {

// TODO(Katie) Add battery temperature when we figure out what we are doing with
// battery temperatures
class EpsSim : public ff_util::FreeFlyerNodelet {
 public:
  EpsSim();
  ~EpsSim();

  bool BatteryService(eps_sim::AddRemoveBattery::Request &req,
                      eps_sim::AddRemoveBattery::Response &res);

 protected:
  virtual void Initialize(ros::NodeHandle *nh);

 private:
  void TelemetryCallback(const ros::TimerEvent &event);

  int rate_;

  float tl_charge_dec_, tr_charge_dec_, bl_charge_dec_, br_charge_dec_;
  float tl_orig_charge_, tr_orig_charge_, bl_orig_charge_, br_orig_charge_;
  float tl_temp_inc_, tr_temp_inc_, bl_temp_inc_, br_temp_inc_;
  float tl_orig_temp_, tr_orig_temp_, bl_orig_temp_, br_orig_temp_;

  ros::Publisher battery_state_pub_tl_, battery_state_pub_tr_;
  ros::Publisher battery_state_pub_bl_, battery_state_pub_br_;
  ros::Publisher battery_temp_pub_tl_, battery_temp_pub_tr_;
  ros::Publisher battery_temp_pub_bl_, battery_temp_pub_br_;

  ros::ServiceServer battery_srv_;

  ros::Timer telem_timer;

  sensor_msgs::BatteryState state_tl_, state_tr_, state_bl_, state_br_;
  sensor_msgs::Temperature temp_tl_, temp_tr_, temp_bl_, temp_br_;
};

}  // namespace eps_sim

#endif  // EPS_SIM_EPS_SIM_H_
