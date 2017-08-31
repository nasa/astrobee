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

// ROS includes
#include <ros/ros.h>

// FSW standard naming
#include <ff_util/ff_names.h>

// Standard messages
#include <ff_hw_msgs/EpsBatteryLocation.h>
#include <ff_hw_msgs/EpsChannelState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>

// PMC helper libraries
#include <eps_driver/eps_driver.h>

// C++ STL includes
#include <string>

#define MESSAGE_QUEUE_SIZE 10

// Publishers
ros::Publisher pub_batt_tl_;
ros::Publisher pub_batt_tr_;
ros::Publisher pub_batt_bl_;
ros::Publisher pub_batt_br_;
ros::Publisher pub_temp_tl_;
ros::Publisher pub_temp_tr_;
ros::Publisher pub_temp_bl_;
ros::Publisher pub_temp_br_;
bool tl_, tr_, bl_, br_;
std::string name = "/";

// Convert battery information from EPS to ROS message
sensor_msgs::BatteryState RandomBatteryState(std::string const& location,
  std_msgs::Header const& header, bool present) {
  sensor_msgs::BatteryState state;
  state.header = header;
  state.present = present;
  state.location = location;
  if (state.present) {
    state.design_capacity = 3.4;
    state.capacity = 3.0;
    state.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    state.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    state.charge = 2.8;
    state.percentage = state.charge / state.capacity * 100.0;
    state.voltage = 11.4;
    state.current = 0.0;
    state.serial_number = "1423850-348645";
  }
  return state;
}

// Convert battery temperature information from EPS to ROS message
sensor_msgs::Temperature RandomBatteryTemperature(std_msgs::Header const& header) {
  sensor_msgs::Temperature temperature;
  temperature.header = header;
  temperature.temperature = 27.0;
  temperature.variance = 0.0;
  return temperature;
}

// Callback to prepare a diagnostic message
void TelemetryCallback(const ros::TimerEvent & event) {
  // Publish the channel information
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = name;
  // Publish the battery information
  for (int i = 0; i < eps_driver::NUM_BATTERIES; i++) {
    // Get the battery index
    eps_driver::BatteryIndex bid = static_cast < eps_driver::BatteryIndex > (i);
    sensor_msgs::Temperature temperature = RandomBatteryTemperature(header);
    sensor_msgs::BatteryState state;
    bool present = false;
    std::string location = "UNKNOWN";
    switch (bid) {
    case eps_driver::BATTERY_TOP_LEFT:
      location = ff_hw_msgs::EpsBatteryLocation::TOP_LEFT;
      present = tl_;
      state = RandomBatteryState(location, header, present);
      pub_batt_tl_.publish(state);
      pub_temp_tl_.publish(temperature);
      break;
    case eps_driver::BATTERY_BOTTOM_LEFT:
      location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT;
      present = bl_;
      state = RandomBatteryState(location, header, present);
      pub_batt_bl_.publish(state);
      pub_temp_bl_.publish(temperature);
      break;
    case eps_driver::BATTERY_TOP_RIGHT:
      location = ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT;
      present = tr_;
      state = RandomBatteryState(location, header, present);
      pub_batt_tr_.publish(state);
      pub_temp_tr_.publish(temperature);
      break;
    case eps_driver::BATTERY_BOTTOM_RIGHT:
      location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT;
      present = br_;
      state = RandomBatteryState(location, header, present);
      pub_batt_br_.publish(state);
      pub_temp_br_.publish(temperature);
      break;
    default:
      location = ff_hw_msgs::EpsBatteryLocation::UNKNOWN;
      present = false;
      break;
    }
  }
}

// Main entry point for application
int main(int argc, char **argv) {
  ros::init(argc, argv, "eps_driver");
  ros::NodeHandle nh("~");
  double rate = 1.0;
  if (!nh.getParam("rate", rate)) ROS_WARN("Could not get parameter TL");
  if (!nh.getParam("present_tl", tl_)) ROS_WARN("Could not get parameter TL");
  if (!nh.getParam("present_tr", tr_)) ROS_WARN("Could not get parameter TR");
  if (!nh.getParam("present_bl", bl_)) ROS_WARN("Could not get parameter BL");
  if (!nh.getParam("present_br", br_)) ROS_WARN("Could not get parameter BR");
  pub_batt_tl_ = nh.advertise < sensor_msgs::BatteryState > (
    name + TOPIC_HARDWARE_EPS_BATTERY_STATE_TL, MESSAGE_QUEUE_SIZE);
  pub_batt_tr_ = nh.advertise < sensor_msgs::BatteryState > (
    name + TOPIC_HARDWARE_EPS_BATTERY_STATE_TR, MESSAGE_QUEUE_SIZE);
  pub_batt_bl_ = nh.advertise < sensor_msgs::BatteryState > (
    name + TOPIC_HARDWARE_EPS_BATTERY_STATE_BL, MESSAGE_QUEUE_SIZE);
  pub_batt_br_ = nh.advertise < sensor_msgs::BatteryState > (
    name + TOPIC_HARDWARE_EPS_BATTERY_STATE_BR, MESSAGE_QUEUE_SIZE);
  pub_temp_tl_ = nh.advertise < sensor_msgs::Temperature > (
    name + TOPIC_HARDWARE_EPS_BATTERY_TEMP_TL, MESSAGE_QUEUE_SIZE);
  pub_temp_tr_ = nh.advertise < sensor_msgs::Temperature > (
    name + TOPIC_HARDWARE_EPS_BATTERY_TEMP_TR, MESSAGE_QUEUE_SIZE);
  pub_temp_bl_ = nh.advertise < sensor_msgs::Temperature > (
    name + TOPIC_HARDWARE_EPS_BATTERY_TEMP_BL, MESSAGE_QUEUE_SIZE);
  pub_temp_br_ = nh.advertise < sensor_msgs::Temperature > (
    name + TOPIC_HARDWARE_EPS_BATTERY_TEMP_BR, MESSAGE_QUEUE_SIZE);
  ros::Timer timer = nh.createTimer(ros::Duration(ros::Rate(rate)), TelemetryCallback, false, true);
  ros::spin();
}
