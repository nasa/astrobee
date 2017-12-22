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

// Standard ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_util/ff_nodelet.h>

// Standard messages
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>

// Messages
#include <ff_hw_msgs/EpsBatteryLocation.h>
#include <ff_hw_msgs/EpsChannelState.h>
#include <ff_hw_msgs/EpsDockStateStamped.h>

// Services
#include <ff_hw_msgs/Reset.h>
#include <ff_hw_msgs/ConfigureSystemLeds.h>
#include <ff_hw_msgs/ConfigurePayloadPower.h>
#include <ff_hw_msgs/ConfigureAdvancedPower.h>
#include <ff_hw_msgs/GetBoardInfo.h>
#include <ff_hw_msgs/ClearTerminate.h>
#include <ff_hw_msgs/RingBuzzer.h>
#include <ff_hw_msgs/SetEnabled.h>
#include <ff_hw_msgs/GetBatteryStatus.h>
#include <ff_hw_msgs/GetTemperatures.h>
#include <ff_hw_msgs/Undock.h>

// PMC helper libraries
#include <eps_driver/eps_driver.h>

#include <functional>
#include <cerrno>
#include <cstring>

/**
 * \ingroup hw
 */
namespace eps_driver {

class EpsDriverNode : public ff_util::FreeFlyerNodelet {
 public:
  // Constructor
  EpsDriverNode() : ff_util::FreeFlyerNodelet(NODE_EPS_DRIVER), eps_(nullptr) {}

  // Destructor - make sure we free dynamically allocated memory
  virtual ~EpsDriverNode() {
    if (eps_)
      delete eps_;
  }

 protected:
  virtual void Initialize(ros::NodeHandle *nh) {
    // Add the config file
    config_params.AddFile("hw/eps_driver.config");
    // Try and read the configuration parameters
    if (!GetParams()) {
      ROS_ERROR("Failed to get parameters.");
      Exit(EXIT_FAILURE);
    }
    // Try and initialize the i2c bus
    if (!Init()) {
      ROS_ERROR("Failed to initialize.");
      Exit(EXIT_FAILURE);
    }
    // Setup the ROS interfaces to handle callbacks
    if (en_eps_reset_) {
      srv_eps_reset_ = nh->advertiseService(
        SERVICE_HARDWARE_EPS_RESET, &EpsDriverNode::ResetCallback, this);
    }
    if (en_conf_payload_power_) {
      srv_conf_payload_power_ = nh->advertiseService(
        SERVICE_HARDWARE_EPS_CONF_PAYLOAD_POWER,
        &EpsDriverNode::PayloadConfigureCallback, this);
    }
    if (en_conf_advanced_power_) {
      srv_conf_advanced_power_ = nh->advertiseService(SERVICE_HARDWARE_EPS_CONF_ADVANCED_POWER,
        &EpsDriverNode::AdvancedConfigureCallback, this);
    }
    if (en_conf_led_state_) {
      srv_conf_led_state_ = nh->advertiseService(
        SERVICE_HARDWARE_EPS_CONF_LED_STATE,
        &EpsDriverNode::LedsConfigureCallback, this);
    }
    if (en_ring_buzzer_) {
      srv_ring_buzzer_ = nh->advertiseService(
        SERVICE_HARDWARE_EPS_RING_BUZZER,
        &EpsDriverNode::RingBuzzerCallback, this);
    }
    if (en_enable_pmcs_) {
      srv_enable_pmcs_ = nh->advertiseService(
        SERVICE_HARDWARE_EPS_ENABLE_PMCS,
        &EpsDriverNode::EnablePmcsCallback, this);
    }
    if (en_get_battery_status_) {
      srv_get_battery_status_ = nh->advertiseService(
        SERVICE_HARDWARE_EPS_GET_BATTERY_STATUS,
        &EpsDriverNode::GetBatteryStatusCallback, this);
    }
    if (en_get_temperatures_) {
      srv_get_temperatures_ = nh->advertiseService(
        SERVICE_HARDWARE_EPS_GET_TEMPERATURES,
        &EpsDriverNode::GetTemperaturesCallback, this);
    }
    if (en_undock_) {
      srv_get_temperatures_ = nh->advertiseService(
        SERVICE_HARDWARE_EPS_UNDOCK,
        &EpsDriverNode::UndockCallback, this);
    }
    if (en_get_board_info_) {
      srv_get_board_info_ = nh->advertiseService(SERVICE_HARDWARE_EPS_GET_BOARD_INFO,
        &EpsDriverNode::GetBoardInfoCallback, this);
    }
    if (en_clear_terminate_) {
      srv_clear_terminate_ = nh->advertiseService(SERVICE_HARDWARE_EPS_CLEAR_TERMINATE,
        &EpsDriverNode::ClearTerminateCallback, this);
    }

    // Setup telemetry publishers
    pub_chan_ = nh->advertise < ff_hw_msgs::EpsChannelState > (
      TOPIC_HARDWARE_EPS_CHANNELS, telemetry_queue_size_);
    pub_batt_tl_ = nh->advertise < sensor_msgs::BatteryState > (
      TOPIC_HARDWARE_EPS_BATTERY_STATE_TL, telemetry_queue_size_);
    pub_batt_tr_ = nh->advertise < sensor_msgs::BatteryState > (
      TOPIC_HARDWARE_EPS_BATTERY_STATE_TR, telemetry_queue_size_);
    pub_batt_bl_ = nh->advertise < sensor_msgs::BatteryState > (
      TOPIC_HARDWARE_EPS_BATTERY_STATE_BL, telemetry_queue_size_);
    pub_batt_br_ = nh->advertise < sensor_msgs::BatteryState > (
      TOPIC_HARDWARE_EPS_BATTERY_STATE_BR, telemetry_queue_size_);
    pub_temp_tl_ = nh->advertise < sensor_msgs::Temperature > (
      TOPIC_HARDWARE_EPS_BATTERY_TEMP_TL, telemetry_queue_size_);
    pub_temp_tr_ = nh->advertise < sensor_msgs::Temperature > (
      TOPIC_HARDWARE_EPS_BATTERY_TEMP_TR, telemetry_queue_size_);
    pub_temp_bl_ = nh->advertise < sensor_msgs::Temperature > (
      TOPIC_HARDWARE_EPS_BATTERY_TEMP_BL, telemetry_queue_size_);
    pub_temp_br_ = nh->advertise < sensor_msgs::Temperature > (
      TOPIC_HARDWARE_EPS_BATTERY_TEMP_BR, telemetry_queue_size_);
    pub_dock_state_ = nh->advertise < ff_hw_msgs::EpsDockStateStamped > (
      TOPIC_HARDWARE_EPS_DOCK_STATE, telemetry_queue_size_);
    // Setup a ros timer to publish telemetry at a fixed rate
    timer_telemetry_ = nh->createTimer(ros::Rate(telemetry_pub_rate_),
      &EpsDriverNode::TelemetryCallback, this, false, true);
    timer_dock_check_ = nh->createTimer(ros::Rate(dock_check_rate_),
      &EpsDriverNode::DockCheckCallback, this, false, true);
  }

  // ROS aware sleep callback function to prevent
  void Sleep(uint32_t microseconds) {
    ros::Duration(static_cast<double>(microseconds) / 1e6).sleep();
  }

  // Initialize the i2c bus
  bool Init(void) {
    i2c::Error err;
    auto bus = i2c::Open(i2c_bus_file_, &err);
    if (!bus) {
      ROS_ERROR_STREAM("Unable to open i2c bus ('"
        << i2c_bus_file_ << "'): " << std::strerror(err));
      return false;
    }
    bus->SetRetries(i2c_retries_);
    // Try and contact the slave device
    auto device = bus->DeviceAt(i2c_address_);
    if (!device) {
      ROS_ERROR_STREAM("Unable to contact i2c slave ('"
        << i2c_address_ << "') on bus " << i2c_bus_file_);
      return false;
    }
    // Set or replace the eps interface and
    if (eps_)
      delete eps_;
    eps_ = new eps_driver::EpsDriver(device, std::bind(&EpsDriverNode::Sleep,
      this, std::placeholders::_1));
    // Success
    return true;
  }

  // Read the configuration from the LUA config file
  bool GetParams() {
    if (!config_params.ReadFiles()) {
      ROS_FATAL("EPS: Unable to load LUA parameters!");
      return false;
    }
    if (!config_params.GetStr("device", &i2c_bus_file_)) {
      ROS_FATAL("EPS: Couldn't find device in LUA config");
      return false;
    }
    if (!config_params.GetUInt("address", &i2c_address_)) {
      ROS_FATAL("EPS: couldn't find address in LUA config");
      return false;
    }
    if (!config_params.GetUInt("i2c_retries", &i2c_retries_)) {
      ROS_FATAL("EPS: couldn't find i2c retry count in LUA config");
      return false;
    }
    if (!config_params.GetUInt("i2c_retries", &i2c_retries_)) {
      ROS_FATAL("EPS: couldn't find i2c retry count in LUA config");
      return false;
    }
    if (!config_params.GetBool("en_srv_eps_reset", &en_eps_reset_)) {
      ROS_FATAL("EPS: couldn't get the enable eps reset flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_conf_led_state", &en_conf_led_state_)) {
      ROS_FATAL("EPS: couldn't get the enable led config flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_conf_payload_power", &en_conf_payload_power_)) {
      ROS_FATAL("EPS: couldn't get the enable power state flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_conf_advanced_power", &en_conf_advanced_power_)) {
      ROS_FATAL("EPS: couldn't get the enable power channels flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_ring_buzzer", &en_ring_buzzer_)) {
      ROS_FATAL("EPS: couldn't get the enable ring buzzer flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_enable_pmcs", &en_enable_pmcs_)) {
      ROS_FATAL("EPS: couldn't get the enable pmc flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_get_battery_status", &en_get_battery_status_)) {
      ROS_FATAL("EPS: couldn't get the enable query battery status flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_get_temperatures", &en_get_temperatures_)) {
      ROS_FATAL("EPS: couldn't get the enable query temperatures flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_undock", &en_undock_)) {
      ROS_FATAL("EPS: couldn't get the enable undock flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_get_board_info", &en_get_board_info_)) {
      ROS_FATAL("EPS: couldn't get the enable query boardinfo flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_clear_terminate", &en_clear_terminate_)) {
      ROS_FATAL("EPS: couldn't get the enable clear terminate flag");
      return false;
    }
    if (!config_params.GetBool("en_pub_housekeeping", &en_pub_housekeeping_)) {
      ROS_FATAL("EPS: couldn't get the enable publish houskeeping flag");
      return false;
    }
    if (!config_params.GetBool("en_pub_battery_status", &en_pub_battery_status_)) {
      ROS_FATAL("EPS: couldn't get the enable publish battery status flag");
      return false;
    }
    if (!config_params.GetBool("en_pub_dock_state", &en_pub_dock_state_)) {
      ROS_FATAL("EPS: couldn't get the enable publish dock state flag");
      return false;
    }
    if (!config_params.GetUInt("telemetry_queue_size", &telemetry_queue_size_)) {
      ROS_FATAL("EPS: telemetry queue size not specified!");
      return false;
    }
    if (!config_params.GetPosReal("telemetry_pub_rate", &telemetry_pub_rate_)) {
      ROS_FATAL("EPS: telemetry publish rate not specified!");
      return false;
    }
    if (!config_params.GetPosReal("dock_check_rate", &dock_check_rate_)) {
      ROS_FATAL("EPS: telemetry publish rate not specified!");
      return false;
    }
    return true;
  }

  // Safely exit
  void Exit(int status) {
    exit(status);
  }

  // Convert battery information from EPS to ROS message
  sensor_msgs::BatteryState BatteryStateConversion(
    eps_driver::BatteryStatus const& status, std_msgs::Header const& header) {
    // Populate the battery data
    sensor_msgs::BatteryState state;
    state.header = header;
    // True if the battery is present
    state.present = status.present;
    if (status.present) {
      // Voltage in Volts (Mandatory)
      state.voltage = static_cast<float>(status.voltage) / 1000.0f;
      // Negative when discharging (A)  (If unmeasured NaN)
      state.current = static_cast<float>(status.current) / 1000.0f;
      // Current charge in Ah  (If unmeasured NaN)
      state.charge = static_cast<float>(status.charge) / 1000.0f;
      // Capacity in Ah (last full capacity)  (If unmeasured NaN)
      state.capacity = static_cast<float>(status.capacity) / 1000.0f;
      // Capacity in Ah (design capacity)  (If unmeasured NaN)
      state.design_capacity
        = static_cast<float>(status.design_capacity) / 1000.0f;
      // Charge percentage on 0 to 1 range  (If unmeasured NaN)
      state.percentage = static_cast<float>(status.percentage) / 100.0;
      // An array of individual cell voltages for each cell in the pack
      for (int i = 0; i < 4; i++) {
        state.cell_voltage.push_back(
          static_cast<float>(status.cell_voltage[i]) / 1000.0f);
      }
      state.serial_number = std::to_string(status.serial_number);
      // FIXME: The battery chemistry is statically set to Li-Ion
      state.power_supply_technology
        = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
      state.power_supply_status
        = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
      state.power_supply_health
        = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    } else {
      state.voltage = nanf("");
      state.current = nanf("");
      state.charge = nanf("");
      state.capacity = nanf("");
      state.design_capacity = nanf("");
      state.percentage = nanf("");
      for (int i = 0; i < 4; i++) {
        state.cell_voltage.push_back(nanf(""));
      }
      state.power_supply_technology
        = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
      state.power_supply_status
        = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
      state.power_supply_health
        = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
      state.serial_number = "";
    }
    return state;
  }

  // Convert battery temperature information from EPS to ROS message
  sensor_msgs::Temperature BatteryTemperatureConversion(
    eps_driver::BatteryStatus const& status, std_msgs::Header const& header) {
    sensor_msgs::Temperature temperature;
    temperature.header = header;
    // Temperature in degree Celsius.
    if (status.present) {
      temperature.temperature
        = static_cast <float>(status.temperature) / 10.0 - 273.15;
      temperature.variance = 0.0;
    } else {
      temperature.temperature = nanf("");
      temperature.variance = nanf("");
    }
    return temperature;
  }

  // Callback for pulling telemetry
  void TelemetryCallback(const ros::TimerEvent&) {
    if (!eps_)
      return;

    // Header info
    static std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = GetPlatform();  // Robot name

    // If we should publish the housekeeping data
    if (en_pub_housekeeping_) {
      // Publish the channel information
      static ff_hw_msgs::EpsChannelState msg;
      msg.header = header;
      std::vector < eps_driver::HousekeepingInfo > data;
      if (eps_->ReadHousekeeping(data)) {
        for (std::vector < eps_driver::HousekeepingInfo >::iterator it = data.begin();
          it != data.end(); it++) {
          ff_hw_msgs::ChannelState channel;
          channel.name = it->description;
          channel.value = static_cast<float>(it->value);
          msg.channels.push_back(channel);
        }
      }
      pub_chan_.publish(msg);
    }
    // If we should publish the battery stateus
    if (en_pub_battery_status_) {
      for (int i = 0; i < NUM_BATTERIES; i++) {
        // Get the battery index
        BatteryIndex bid = static_cast < BatteryIndex > (i);
        // Extract the battery data
        eps_driver::BatteryStatus status;
        if (!eps_->GetBatteryStatus(bid, status)) {
          ROS_WARN_STREAM("Could not query the battery status for index " << i);
        } else {
          sensor_msgs::BatteryState battery
              = BatteryStateConversion(status, header);
          sensor_msgs::Temperature temperature
              = BatteryTemperatureConversion(status, header);
          // Choose the temperature publisher based on the battery id
          switch (bid) {
          case eps_driver::BATTERY_TOP_LEFT:
            battery.location = ff_hw_msgs::EpsBatteryLocation::TOP_LEFT;
            pub_batt_tl_.publish(battery);
            pub_temp_tl_.publish(temperature);
            break;
          case eps_driver::BATTERY_BOTTOM_LEFT:
            battery.location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT;
            pub_batt_bl_.publish(battery);
            pub_temp_bl_.publish(temperature);
            break;
          case eps_driver::BATTERY_TOP_RIGHT:
            battery.location = ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT;
            pub_batt_tr_.publish(battery);
            pub_temp_tr_.publish(temperature);
            break;
          case eps_driver::BATTERY_BOTTOM_RIGHT:
            battery.location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT;
            pub_batt_br_.publish(battery);
            pub_temp_br_.publish(temperature);
            break;
          default:
            battery.location = ff_hw_msgs::EpsBatteryLocation::UNKNOWN;
            break;
          }
        }
      }
    }
  }

  // Callback for pulling telemetry
  void DockCheckCallback(const ros::TimerEvent&) {
    if (!eps_)
      return;
    // If we should publish the dock state
    if (en_pub_dock_state_) {
      static ff_hw_msgs::EpsDockStateStamped msg;
      msg.header.frame_id = GetPlatform();
      msg.header.stamp = ros::Time::now();
      msg.state = ff_hw_msgs::EpsDockStateStamped::UNKNOWN;
      // Try and query the state
      eps_driver::ConnectionState state;
      if (eps_->GetConnectionState(state)) {
        switch (state) {
        case eps_driver::CONN_DISCONNECTED:
          msg.state = ff_hw_msgs::EpsDockStateStamped::UNDOCKED;     break;
        case eps_driver::CONN_CONNECTING:
          msg.state = ff_hw_msgs::EpsDockStateStamped::CONNECTING;   break;
        case eps_driver::CONN_CONNECTED:
          msg.state = ff_hw_msgs::EpsDockStateStamped::DOCKED;       break;
        default:
          msg.state = ff_hw_msgs::EpsDockStateStamped::UNKNOWN;      break;
        }
      }
      // Only publish if the state differs
      pub_dock_state_.publish(msg);
    }
  }

  // Callback for resetting the EPS
  bool ResetCallback(ff_hw_msgs::Reset::Request &req,
                     ff_hw_msgs::Reset::Response &res) {
    if (!eps_) return false;
    res.success = eps_->Reset();
    if (!res.success)
      res.status_message = "Could not reset the EPS";
    return true;
  }

  // Callback for configuring the LEDs
  bool LedsConfigureCallback(ff_hw_msgs::ConfigureSystemLeds::Request &req,
                             ff_hw_msgs::ConfigureSystemLeds::Response &res) {
    if (!eps_) return false;
    res.success = true;
    res.success = res.success && eps_->SetLedState(LED_STATUS_1,
      static_cast < PowerState > (req.status1));
    res.success = res.success && eps_->SetLedState(LED_STATUS_2,
      static_cast < PowerState > (req.status2));
    res.success = res.success && eps_->SetLedState(LED_STATUS_3,
      static_cast < PowerState > (req.status3));
    res.success = res.success && eps_->SetLedState(LED_STATUS_4,
      static_cast < PowerState > (req.status4));
    res.success = res.success && eps_->SetLedState(LED_STATUS_5,
      static_cast < PowerState > (req.status5));
    res.success = res.success && eps_->SetLedState(LED_STATUS_6,
      static_cast < PowerState > (req.status6));
    res.success = res.success && eps_->SetLedState(LED_STREAM,
      static_cast < PowerState > (req.streaming));
    res.success = res.success && eps_->SetLedState(LED_CAMERA,
      static_cast < PowerState > (req.camera));
    res.success = res.success && eps_->SetLedState(LED_MIC,
      static_cast < PowerState > (req.microphone));
    if (!res.success)
      res.status = "Could not set all of the LED states";
    else
      res.status = "All LED states set sucessfully";
    return true;
  }

  // Callback for setting the power state
  bool PayloadConfigureCallback(ff_hw_msgs::ConfigurePayloadPower::Request &req,
                                ff_hw_msgs::ConfigurePayloadPower::Response &res) {
    if (!eps_) return false;
    res.success = true;
    res.success = res.success && eps_->SetPayloadState(PAYLOAD_TOP_FRONT,
      static_cast < PowerState > (req.top_front));
    res.success = res.success && eps_->SetPayloadState(PAYLOAD_BOTTOM_FRONT,
      static_cast < PowerState > (req.bottom_front));
    res.success = res.success && eps_->SetPayloadState(PAYLOAD_TOP_AFT,
      static_cast < PowerState > (req.top_aft));
    res.success = res.success && eps_->SetPayloadState(PAYLOAD_BOTTOM_AFT,
      static_cast < PowerState > (req.bottom_aft));
    if (!res.success)
      res.status = "Could not set the payload power";
    else
      res.status = "Payload power sent successfully";
    return true;
  }

  // Callback for setting the power channels
  bool AdvancedConfigureCallback(ff_hw_msgs::ConfigureAdvancedPower::Request &req,
                                ff_hw_msgs::ConfigureAdvancedPower::Response &res) {
    if (!eps_) return false;
    res.success = true;
    res.success = res.success && eps_->SetAdvancedState(ADVANCED_USB,
      static_cast < PowerState > (req.usb));
    res.success = res.success && eps_->SetAdvancedState(ADVANCED_AUX,
      static_cast < PowerState > (req.aux));
    res.success = res.success && eps_->SetAdvancedState(ADVANCED_PMC1,
      static_cast < PowerState > (req.pmc1));
    res.success = res.success && eps_->SetAdvancedState(ADVANCED_PMC2,
      static_cast < PowerState > (req.pmc2));
    if (!res.success)
      res.status = "Could not set the advanced power";
    else
      res.status = "Advanced power channels sent successfully";
    return true;
  }

  // Callback for ringing the buzzer
  bool RingBuzzerCallback(ff_hw_msgs::RingBuzzer::Request &req,
                          ff_hw_msgs::RingBuzzer::Response &res) {
    if (!eps_) return false;
    res.success = eps_->RingBuzzer(req.frequency, req.duration);
    if (!res.success)
      res.status_message = "Could not ring the buzzer";
    return true;
  }

  // Callback for enabling the PMCs
  bool EnablePmcsCallback(ff_hw_msgs::SetEnabled::Request &req,
                          ff_hw_msgs::SetEnabled::Response &res) {
    if (!eps_) return false;
    res.success = eps_->EnablePMCs(req.enabled);
    if (!res.success)
      res.status_message = "Could not enable the PMCs";
    return true;
  }

  // Callback for retrieving the battery status
  bool GetBatteryStatusCallback(ff_hw_msgs::GetBatteryStatus::Request &req,
                                ff_hw_msgs::GetBatteryStatus::Response &res) {
    if (!eps_) return false;
    // Get a battery id from a location
    eps_driver::BatteryIndex bid = NUM_BATTERIES;
    if (req.location == ff_hw_msgs::EpsBatteryLocation::TOP_LEFT)
      bid = eps_driver::BATTERY_TOP_LEFT;
    if (req.location == ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT)
      bid = eps_driver::BATTERY_BOTTOM_LEFT;
    if (req.location == ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT)
      bid = eps_driver::BATTERY_TOP_RIGHT;
    if (req.location == ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT)
      bid = eps_driver::BATTERY_BOTTOM_RIGHT;
    res.success = (bid < eps_driver::NUM_BATTERIES);
    // Only query indexes we know exist
    if (res.success) {
      // Get the battery status
      eps_driver::BatteryStatus status;
      if (!eps_->GetBatteryStatus(bid, status)) {
        ROS_WARN_STREAM("Could not query the battery status for location " << req.location);
      } else {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = GetPlatform();
        res.state = BatteryStateConversion(status, header);
        res.temperature = BatteryTemperatureConversion(status, header);
      }
    }
    return true;
  }

  // Callback for retrieving the temperatures
  bool GetTemperaturesCallback(ff_hw_msgs::GetTemperatures::Request &req,
                               ff_hw_msgs::GetTemperatures::Response &res) {
    if (!eps_) return false;
    switch (req.type) {
    case ff_hw_msgs::GetTemperatures::Request::ANALOG:
      ROS_ERROR("Analog sensor is not supported");
      res.success = false;
      break;
    case ff_hw_msgs::GetTemperatures::Request::DIGITAL: {
      std::vector<double> temps;
      res.success = eps_->ReadTemperatureSensors(temps);

      // FIXME: res.data should be float or double.
      for (size_t i = 0; i < temps.size(); i++)
        res.data[i] = static_cast<uint16_t>(temps[i]);
      break;
    }
    default:
      res.success = false;
    }
    if (!res.success)
      res.status_message = "Could not retrieve the battery status";
    return true;
  }

  // Callback for undocking
  bool UndockCallback(ff_hw_msgs::Undock::Request &req,
                      ff_hw_msgs::Undock::Response &res) {
    // If we don't have a valid i2c device
    if (!eps_) {
      res.value = ff_hw_msgs::Undock::Response::I2C_FAILED;
      return true;
    }
    // If we cant determine the dock state
    eps_driver::ConnectionState state;
    if (!eps_->GetConnectionState(state)) {
      res.value = ff_hw_msgs::Undock::Response::CANNOT_QUERY_STATE;
      return true;
    }
    // Do somethign based on the state
    switch (state) {
    case eps_driver::CONN_DISCONNECTED:
    case eps_driver::CONN_CONNECTING:
      res.value = ff_hw_msgs::Undock::Response::NOT_DOCKED;
      return true;
    case eps_driver::CONN_CONNECTED:
      break;
    }
    // If we get here then we can try and undock
    if (!eps_->Undock()) {
      res.value = ff_hw_msgs::Undock::Response::UNDOCK_FAILED;
      return true;
    }
    // If we get here, undocking succeeded
    res.value = ff_hw_msgs::Undock::Response::SUCCESS;
    return true;
  }

  // Callback for get board information
  bool GetBoardInfoCallback(ff_hw_msgs::GetBoardInfo::Request &req,
                          ff_hw_msgs::GetBoardInfo::Response &res) {
    std::string v;
    std::string s;
    if (!eps_) return false;
    res.success = eps_->GetString(eps_driver::SW_VERSION, v);
    res.success = eps_->GetString(eps_driver::SERIAL, s);
    if (!res.success) {
      res.status_message = "Could get board information";
    } else {
      res.version = v;
      res.serial  = s;
    }
    return true;
  }

  // Callback for clear terminate
  bool ClearTerminateCallback(ff_hw_msgs::ClearTerminate::Request &req,
                          ff_hw_msgs::ClearTerminate::Response &res) {
    if (!eps_) return false;
    res.success = eps_->ClearTerminateEvent();
    if (!res.success)
      res.status_message = "Could not clear terminate";
    return true;
  }


 private:
  config_reader::ConfigReader config_params;    // LUA configuration reader
  std::string i2c_bus_file_;                    // i2c bus
  uint32_t i2c_address_;                        // 7 bit device address
  uint32_t i2c_retries_;                        // number of slave retries
  bool en_eps_reset_;                           // Enable reset
  bool en_conf_led_state_;                      // Enable LEDs
  bool en_conf_payload_power_;                  // Enable set power state
  bool en_conf_advanced_power_;                 // Enable set power channels
  bool en_ring_buzzer_;                         // Enable buzzer
  bool en_enable_pmcs_;                         // Enable PMCs
  bool en_get_battery_status_;                  // Enable get battery status
  bool en_get_temperatures_;                    // Enable get temperatures
  bool en_undock_;                              // Enable undock
  bool en_get_board_info_;                      // Enable get board information
  bool en_clear_terminate_;                     // Enable clear terminate
  bool en_pub_housekeeping_;                    // Enable publish housekeeping
  bool en_pub_battery_status_;                  // Enable publish battery status
  bool en_pub_dock_state_;                      // Enable publish dock state
  unsigned int telemetry_queue_size_;           // Telemetry queue size
  float telemetry_pub_rate_;                    // Telemetry publication rate
  float dock_check_rate_;                       // Dock check rate
  eps_driver::EpsDriver *eps_;                  // Interface class to EPS
  ros::ServiceServer srv_eps_reset_;            // Reset the hardware
  ros::ServiceServer srv_conf_payload_power_;   // Configure LEDs
  ros::ServiceServer srv_conf_advanced_power_;  // Set the power channels
  ros::ServiceServer srv_conf_led_state_;       // Set the power state for a channel
  ros::ServiceServer srv_ring_buzzer_;          // Ring buzzer
  ros::ServiceServer srv_enable_pmcs_;          // Enable PMCs
  ros::ServiceServer srv_get_battery_status_;   // Get battery status
  ros::ServiceServer srv_get_temperatures_;     // Get temperatures
  ros::ServiceServer srv_undock_;               // Undock service
  ros::Timer timer_telemetry_;                  // Telemetry timer
  ros::Timer timer_dock_check_;                 // Dock check timer
  ros::ServiceServer srv_get_board_info_;       // Get board information
  ros::ServiceServer srv_clear_terminate_;      // Clear terminate
  ros::Publisher pub_chan_;                     // Telemetry publishers
  ros::Publisher pub_dock_state_;               // Dock state publisher
  ros::Publisher pub_batt_tl_;                  // Battery: top left
  ros::Publisher pub_batt_tr_;                  // Battery: top right
  ros::Publisher pub_batt_bl_;                  // Battery: bottom left
  ros::Publisher pub_batt_br_;                  // Battery: bottom right
  ros::Publisher pub_temp_tl_;                  // Temperature: top left
  ros::Publisher pub_temp_tr_;                  // Temperature: top right
  ros::Publisher pub_temp_bl_;                  // Temperature: bottom left
  ros::Publisher pub_temp_br_;                  // Temperature: bottom right
};

PLUGINLIB_DECLARE_CLASS(eps_driver, EpsDriverNode,
                        eps_driver::EpsDriverNode, nodelet::Nodelet);

}  // namespace eps_driver
