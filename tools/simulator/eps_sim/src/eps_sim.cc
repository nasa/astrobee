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


#include <eps_sim/eps_sim.h>

namespace eps_sim {
EpsSim::EpsSim() : ff_util::FreeFlyerNodelet(), rate_(10) {
}

EpsSim::~EpsSim() {
}

bool EpsSim::BatteryService(eps_sim::AddRemoveBattery::Request &req,
                            eps_sim::AddRemoveBattery::Response &res) {
  if (req.battery.location == ff_hw_msgs::EpsBatteryLocation::TOP_LEFT) {
    state_tl_ = req.battery;
    if (req.battery.present) {
      tl_orig_charge_ = state_tl_.charge;
      tl_charge_dec_ = req.battery_charge_decrement_value;
      temp_tl_.temperature = req.battery_temperature;
      tl_orig_temp_ = req.battery_temperature;
      tl_temp_inc_ = req.battery_temperature_increment_value;
      state_tl_.percentage = state_tl_.charge / state_tl_.capacity * 100.0;
    }
    battery_state_pub_tl_.publish(state_tl_);
    temp_tl_.header.stamp = ros::Time::now();
    battery_temp_pub_tl_.publish(temp_tl_);
  } else if (req.battery.location ==
                                    ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT) {
    state_tr_ = req.battery;
    if (req.battery.present) {
      tr_orig_charge_ = state_tr_.charge;
      tr_charge_dec_ = req.battery_charge_decrement_value;
      temp_tr_.temperature = req.battery_temperature;
      tr_orig_temp_ = req.battery_temperature;
      tr_temp_inc_ = req.battery_temperature_increment_value;
      state_tr_.percentage = state_tr_.charge / state_tr_.capacity * 100.0;
    }
    battery_state_pub_tr_.publish(state_tr_);
    temp_tr_.header.stamp = ros::Time::now();
    battery_temp_pub_tr_.publish(temp_tr_);
  } else if (req.battery.location ==
                                  ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT) {
    state_bl_ = req.battery;
    if (req.battery.present) {
      bl_orig_charge_ = state_bl_.charge;
      bl_charge_dec_ = req.battery_charge_decrement_value;
      temp_bl_.temperature = req.battery_temperature;
      bl_orig_temp_ = req.battery_temperature;
      bl_temp_inc_ = req.battery_temperature_increment_value;
      state_bl_.percentage = state_bl_.charge / state_bl_.capacity * 100.0;
    }
    battery_state_pub_bl_.publish(state_bl_);
    temp_bl_.header.stamp = ros::Time::now();
    battery_temp_pub_bl_.publish(temp_bl_);
  } else if (req.battery.location ==
                                ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT) {
    state_br_ = req.battery;
    if (req.battery.present) {
      br_orig_charge_ = state_br_.charge;
      br_charge_dec_ = req.battery_charge_decrement_value;
      temp_br_.temperature = req.battery_temperature;
      br_orig_temp_ = req.battery_temperature;
      br_temp_inc_ = req.battery_temperature_increment_value;
      state_br_.percentage = state_br_.charge / state_br_.capacity * 100.0;
    }
    battery_state_pub_br_.publish(state_br_);
    temp_br_.header.stamp = ros::Time::now();
    battery_temp_pub_br_.publish(temp_br_);
  } else {
    ROS_ERROR("Eps sim: Battery location %s not recognized. Battery not added!",
                                                  req.battery.location.c_str());
    return false;
  }

  return true;
}

void EpsSim::Initialize(ros::NodeHandle *nh) {
  battery_state_pub_tl_ = nh->advertise<sensor_msgs::BatteryState>(
      TOPIC_HARDWARE_EPS_BATTERY_STATE_TL, MESSAGE_QUEUE_SIZE);
  battery_state_pub_tr_ = nh->advertise<sensor_msgs::BatteryState>(
      TOPIC_HARDWARE_EPS_BATTERY_STATE_TR, MESSAGE_QUEUE_SIZE);
  battery_state_pub_bl_ = nh->advertise<sensor_msgs::BatteryState>(
      TOPIC_HARDWARE_EPS_BATTERY_STATE_BL, MESSAGE_QUEUE_SIZE);
  battery_state_pub_br_ = nh->advertise<sensor_msgs::BatteryState>(
      TOPIC_HARDWARE_EPS_BATTERY_STATE_BR, MESSAGE_QUEUE_SIZE);

  battery_temp_pub_tl_ = nh->advertise<sensor_msgs::Temperature>(
      TOPIC_HARDWARE_EPS_BATTERY_TEMP_TL, MESSAGE_QUEUE_SIZE);
  battery_temp_pub_tr_ = nh->advertise<sensor_msgs::Temperature>(
      TOPIC_HARDWARE_EPS_BATTERY_TEMP_TR, MESSAGE_QUEUE_SIZE);
  battery_temp_pub_bl_ = nh->advertise<sensor_msgs::Temperature>(
      TOPIC_HARDWARE_EPS_BATTERY_TEMP_BL, MESSAGE_QUEUE_SIZE);
  battery_temp_pub_br_ = nh->advertise<sensor_msgs::Temperature>(
      TOPIC_HARDWARE_EPS_BATTERY_TEMP_BR, MESSAGE_QUEUE_SIZE);

  telem_timer = nh->createTimer(ros::Duration(rate_),
                               &EpsSim::TelemetryCallback, this, false, true);

  battery_srv_ = nh->advertiseService("add_remove_battery",
                                      &EpsSim::BatteryService,
                                      this);

  state_tl_.header.stamp = ros::Time::now();
  state_tl_.present = false;
  state_tl_.location = ff_hw_msgs::EpsBatteryLocation::TOP_LEFT;
  tl_orig_charge_ = 0;
  tl_charge_dec_ = 0;
  battery_state_pub_tl_.publish(state_tl_);
  temp_tl_.header.stamp = ros::Time::now();
  temp_tl_.temperature = -273;
  tl_orig_temp_ = -273;
  tl_temp_inc_ = 0;
  battery_temp_pub_tl_.publish(temp_tl_);

  state_tr_.header.stamp = ros::Time::now();
  state_tr_.present = false;
  state_tr_.location = ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT;
  tr_orig_charge_ = 0;
  tr_charge_dec_ = 0;
  battery_state_pub_tr_.publish(state_tr_);
  temp_tr_.header.stamp = ros::Time::now();
  temp_tr_.temperature = -273;
  tr_orig_temp_ = -273;
  tr_temp_inc_ = 0;
  battery_temp_pub_tr_.publish(temp_tr_);

  state_bl_.header.stamp = ros::Time::now();
  state_bl_.present = false;
  state_bl_.location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT;
  bl_orig_charge_ = 0;
  bl_charge_dec_ = 0;
  battery_state_pub_bl_.publish(state_bl_);
  temp_bl_.header.stamp = ros::Time::now();
  temp_bl_.temperature = -273;
  bl_orig_temp_ = -273;
  bl_temp_inc_ = 0;
  battery_temp_pub_bl_.publish(temp_bl_);

  state_br_.header.stamp = ros::Time::now();
  state_br_.present = false;
  state_br_.location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT;
  br_orig_charge_ = 0;
  br_charge_dec_ = 0;
  battery_state_pub_br_.publish(state_br_);
  temp_br_.header.stamp = ros::Time::now();
  temp_br_.temperature = -273;
  br_orig_temp_ = -273;
  br_temp_inc_ = 0;
  battery_temp_pub_br_.publish(temp_br_);
}

void EpsSim::TelemetryCallback(const ros::TimerEvent &event) {
  if (state_tl_.present) {
    state_tl_.charge -= tl_charge_dec_;
    if (state_tl_.charge < 0) {
      state_tl_.charge = tl_orig_charge_;
    }
    state_tl_.percentage = state_tl_.charge / state_tl_.capacity * 100.0;

    temp_tl_.temperature += tl_temp_inc_;
    if (temp_tl_.temperature >= MAX_TEMPERATURE) {
      temp_tl_.temperature = tl_orig_temp_;
    }
  }

  // Always publish every battery state regardless if it is present or not since
  // the actual eps driver does this
  state_tl_.header.stamp = ros::Time::now();
  battery_state_pub_tl_.publish(state_tl_);
  temp_tl_.header.stamp = ros::Time::now();
  battery_temp_pub_tl_.publish(temp_tl_);

  if (state_tr_.present) {
    state_tr_.charge -= tr_charge_dec_;
    if (state_tr_.charge < 0) {
      state_tr_.charge = tr_orig_charge_;
    }
    state_tr_.percentage = state_tr_.charge / state_tr_.capacity * 100.0;

    temp_tr_.temperature += tr_temp_inc_;
    if (temp_tr_.temperature >= MAX_TEMPERATURE) {
      temp_tr_.temperature = tr_orig_temp_;
    }
  }
  state_tr_.header.stamp = ros::Time::now();
  battery_state_pub_tr_.publish(state_tr_);
  temp_tr_.header.stamp = ros::Time::now();
  battery_temp_pub_tr_.publish(temp_tr_);

  if (state_bl_.present) {
    state_bl_.charge -= bl_charge_dec_;
    if (state_bl_.charge < 0) {
      state_bl_.charge = bl_orig_charge_;
    }
    state_bl_.percentage = state_bl_.charge / state_bl_.capacity * 100.0;

    temp_bl_.temperature += bl_temp_inc_;
    if (temp_bl_.temperature >= MAX_TEMPERATURE) {
      temp_bl_.temperature = bl_orig_temp_;
    }
  }
  state_bl_.header.stamp = ros::Time::now();
  battery_state_pub_bl_.publish(state_bl_);
  temp_bl_.header.stamp = ros::Time::now();
  battery_temp_pub_bl_.publish(temp_bl_);

  if (state_br_.present) {
    state_br_.charge -= br_charge_dec_;
    if (state_br_.charge < 0) {
      state_br_.charge = br_orig_charge_;
    }
    state_br_.percentage = state_br_.charge / state_br_.capacity * 100.0;

    temp_br_.temperature += br_temp_inc_;
    if (temp_br_.temperature >= MAX_TEMPERATURE) {
      temp_br_.temperature = br_orig_temp_;
    }
  }
  state_br_.header.stamp = ros::Time::now();
  battery_state_pub_br_.publish(state_br_);
  temp_br_.header.stamp = ros::Time::now();
  battery_temp_pub_br_.publish(temp_br_);
}

}  // namespace eps_sim

PLUGINLIB_EXPORT_CLASS(eps_sim::EpsSim, nodelet::Nodelet)
