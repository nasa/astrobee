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

#include "dds_ros_bridge/ros_battery_state.h"

ff::RosBatteryStateToRapid::RosBatteryStateToRapid(
                                  const std::string& sub_topic_battery_state_TL,
                                  const std::string& sub_topic_battery_state_TR,
                                  const std::string& sub_topic_battery_state_BL,
                                  const std::string& sub_topic_battery_state_BR,
                                  const std::string& sub_topic_battery_temp_TL,
                                  const std::string& sub_topic_battery_temp_TR,
                                  const std::string& sub_topic_battery_temp_BL,
                                  const std::string& sub_topic_battery_temp_BR,
                                  const std::string& pub_topic,
                                  const ros::NodeHandle &nh,
                                  const unsigned int queue_size)
  : RosSubRapidPub(sub_topic_battery_state_TL, pub_topic, nh, queue_size),
    battery_time_multiple_(10) {
  config_supplier_.reset(new ff::RosBatteryStateToRapid::ConfigSupplier(
        rapid::ext::astrobee::EPS_CONFIG_TOPIC + pub_topic, "",
        "AstrobeeEpsConfigProfile", ""));

  state_supplier_.reset(new ff::RosBatteryStateToRapid::StateSupplier(
       rapid::ext::astrobee::EPS_STATE_TOPIC + pub_topic, "",
      "AstrobeeEpsStateProfile", ""));

  sub_battery_state_tl_ = nh_.subscribe(sub_topic_battery_state_TL,
                                        queue_size,
                                        &RosBatteryStateToRapid::StateCallback,
                                        this);
  sub_battery_state_tr_ = nh_.subscribe(sub_topic_battery_state_TR,
                                        queue_size,
                                        &RosBatteryStateToRapid::StateCallback,
                                        this);
  sub_battery_state_bl_ = nh_.subscribe(sub_topic_battery_state_BL,
                                        queue_size,
                                        &RosBatteryStateToRapid::StateCallback,
                                        this);
  sub_battery_state_br_ = nh_.subscribe(sub_topic_battery_state_BR,
                                        queue_size,
                                        &RosBatteryStateToRapid::StateCallback,
                                        this);

  sub_battery_temp_tl_ = nh_.subscribe(sub_topic_battery_temp_TL,
                                       queue_size,
                                       &RosBatteryStateToRapid::TempTLCallback,
                                       this);
  sub_battery_temp_tr_ = nh_.subscribe(sub_topic_battery_temp_TR,
                                       queue_size,
                                       &RosBatteryStateToRapid::TempTRCallback,
                                       this);
  sub_battery_temp_bl_ = nh_.subscribe(sub_topic_battery_temp_BL,
                                       queue_size,
                                       &RosBatteryStateToRapid::TempBLCallback,
                                       this);
  sub_battery_temp_br_ = nh_.subscribe(sub_topic_battery_temp_BR,
                                       queue_size,
                                       &RosBatteryStateToRapid::TempBRCallback,
                                       this);

  rapid::RapidHelper::initHeader(config_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);

  // Initialize the serial number to be 0, it will be incremented when the
  // config changes i.e. when batteries are added or removed.
  // technically this could have been set in the initHeader function but it is
  // the last argument and I didn't care to specify any of the other arguments
  config_supplier_->event().hdr.serial = 0;
  state_supplier_->event().hdr.serial = 0;

  config_supplier_->sendEvent();
  state_supplier_->sendEvent();
}

void ff::RosBatteryStateToRapid::AddTempToState(
                                        rapid::ext::astrobee::BatterySlot slot,
                                        float temp) {
  // Find where temperature goes in state message. If battery isn't present,
  // temperature isn't added
  for (int i = 0; i < config_supplier_->event().batteries.length(); i++) {
    if (config_supplier_->event().batteries[i].slot == slot) {
      state_supplier_->event().batteries[i].temperature = temp;
      break;
    }
  }
}

rapid::ext::astrobee::BatterySlot ff::RosBatteryStateToRapid::ConvertBatteryLoc(
                                                  std::string const& location) {
  if (location == ff_hw_msgs::EpsBatteryLocation::TOP_LEFT) {
    return rapid::ext::astrobee::SLOT_TOP_LEFT;
  } else if (location == ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT) {
    return rapid::ext::astrobee::SLOT_TOP_RIGHT;
  } else if (location == ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT) {
    return rapid::ext::astrobee::SLOT_BOTTOM_LEFT;
  } else if (location == ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT) {
    return rapid::ext::astrobee::SLOT_BOTTOM_RIGHT;
  } else {
    ROS_ERROR("Executive: Battery location %s is invalid.", location.c_str());
  }

  return rapid::ext::astrobee::SLOT_UNKNOWN;
}

void ff::RosBatteryStateToRapid::SetBatteryTimeMultiple(int multiple) {
  battery_time_multiple_ = multiple;
}

void ff::RosBatteryStateToRapid::StateCallback(sensor_msgs::BatteryStateConstPtr
                                                                const& state) {
  int i = 0, length;
  bool config_updated = false, found = false, present = false;
  int32_t last_estimated_minutes_remaining;
  int32_t current_estimated_minutes_remaining = 0;
  float current_estimated_hours_remaining = 0;
  float total_percentage = 0;

  rapid::ext::astrobee::EpsConfig &config_msg = config_supplier_->event();
  rapid::ext::astrobee::EpsState &state_msg = state_supplier_->event();
  last_estimated_minutes_remaining = state_msg.estimatedMinutesRemaining;

  // We get one battery at a time. Need to check if it was added or removed.
  present = state->present;
  found = false;
  for (i = 0; i < config_msg.batteries.length(); i++) {
    if (config_msg.batteries[i].slot == ConvertBatteryLoc(state->location)) {
      found = true;
      break;
    }
  }

  // Check if found but no longer present
  if (found && !present) {
    // Remove battery from config state since it is no longer present
    // Rapid doesn't allow us to remove element i so put battery at the end in
    // the location of the battery that was removed and then decrement the
    // length.
    int last_loc = config_msg.batteries.length() - 1;

    // If battery to be removed is at the end, just decrement the length
    if (i != last_loc) {
      config_msg.batteries[i].slot = config_msg.batteries[last_loc].slot;
      config_msg.batteries[i].designedCapacity =
                                config_msg.batteries[last_loc].designedCapacity;
      config_msg.batteries[i].currentMaxCapacity =
                              config_msg.batteries[last_loc].currentMaxCapacity;

      state_msg.batteries[i].percentage =
                                      state_msg.batteries[last_loc].percentage;
      state_msg.batteries[i].temperature =
                                      state_msg.batteries[last_loc].temperature;
      state_msg.batteries[i].voltage = state_msg.batteries[last_loc].voltage;
      state_msg.batteries[i].current = state_msg.batteries[last_loc].current;
      state_msg.batteries[i].remainingCapacity =
                                state_msg.batteries[last_loc].remainingCapacity;
    }

    length = config_msg.batteries.length() - 1;

    config_msg.batteries.length(length);
    state_msg.batteries.length(length);
    config_updated = true;
  } else if (!found && present) {  // Check if present but not in config message
    // Add battery to the end of the config and state messages
    length = config_msg.batteries.length() + 1;
    config_msg.batteries.length(length);
    state_msg.batteries.length(length);

    config_msg.batteries[i].slot = ConvertBatteryLoc(state->location);
    config_msg.batteries[i].designedCapacity = state->design_capacity;
    config_msg.batteries[i].currentMaxCapacity = state->capacity;

    config_updated = true;
  }

  // If battery is present, copy values into state at ith location
  if (present) {
    state_msg.batteries[i].percentage = state->percentage;
    state_msg.batteries[i].voltage = state->voltage;
    state_msg.batteries[i].current = state->current;
    state_msg.batteries[i].remainingCapacity = state->charge;
  }

  // Calculate battery total and estimated time remaining
  for (i = 0; i < state_msg.batteries.length(); i++) {
    total_percentage += state_msg.batteries[i].percentage;
    // If battery is charging, don't use current to calculate estimated time
    // remaining
    if (state_msg.batteries[i].current >= 0) {
      current_estimated_hours_remaining +=
                                      state_msg.batteries[i].remainingCapacity;
    } else {
      current_estimated_hours_remaining +=
                                  (state_msg.batteries[i].remainingCapacity/
                                    state_msg.batteries[i].current * -1);
    }
  }

  state_msg.batteryTotal = total_percentage;

  // Convert hours to minutes
  current_estimated_minutes_remaining = current_estimated_hours_remaining * 60;

  // Round estimated time remaining to the nearest tenth
  current_estimated_minutes_remaining = (((current_estimated_minutes_remaining +
                                      (battery_time_multiple_/2)) /
                                      battery_time_multiple_) *
                                      battery_time_multiple_);

  state_msg.estimatedMinutesRemaining = current_estimated_minutes_remaining;

  // If config updated, don't worry about checking if the time estimated changed
  // before publishing the state. Changing the number of batteries is reason
  // enough to publish the state.
  if (config_updated) {
    config_msg.hdr.timeStamp = util::RosTime2RapidTime(state->header.stamp);
    config_msg.hdr.serial += 1;
    config_supplier_->sendEvent();

    state_msg.hdr.timeStamp = util::RosTime2RapidTime(state->header.stamp);
    state_msg.hdr.serial += 1;
    state_supplier_->sendEvent();
  } else if (last_estimated_minutes_remaining !=
                                          state_msg.estimatedMinutesRemaining) {
    // Only publish the state if the time estimated changed since
    state_msg.hdr.timeStamp = util::RosTime2RapidTime(state->header.stamp);
    state_supplier_->sendEvent();
  }
}

void ff::RosBatteryStateToRapid::TempTLCallback(sensor_msgs::TemperatureConstPtr
                                                                  const& temp) {
  AddTempToState(rapid::ext::astrobee::SLOT_TOP_LEFT, temp->temperature);
}

void ff::RosBatteryStateToRapid::TempTRCallback(sensor_msgs::TemperatureConstPtr
                                                                  const& temp) {
  AddTempToState(rapid::ext::astrobee::SLOT_TOP_RIGHT, temp->temperature);
}

void ff::RosBatteryStateToRapid::TempBLCallback(sensor_msgs::TemperatureConstPtr
                                                                  const& temp) {
  AddTempToState(rapid::ext::astrobee::SLOT_BOTTOM_LEFT, temp->temperature);
}

void ff::RosBatteryStateToRapid::TempBRCallback(sensor_msgs::TemperatureConstPtr
                                                                  const& temp) {
  AddTempToState(rapid::ext::astrobee::SLOT_BOTTOM_RIGHT, temp->temperature);
}
