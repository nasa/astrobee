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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_util/ff_nodelet.h>

// Standard messages
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>

// Messages
#include <ff_hw_msgs/EpsBatteryLocation.h>
#include <ff_hw_msgs/EpsPowerState.h>
#include <ff_hw_msgs/EpsHousekeeping.h>
#include <ff_hw_msgs/EpsDockStateStamped.h>

// Services
#include <ff_hw_msgs/ClearTerminate.h>
#include <ff_hw_msgs/ConfigureAdvancedPower.h>
#include <ff_hw_msgs/ConfigurePayloadPower.h>
#include <ff_hw_msgs/ConfigureSystemLeds.h>
#include <ff_hw_msgs/GetBatteryStatus.h>
#include <ff_hw_msgs/GetBoardInfo.h>
#include <ff_hw_msgs/GetTemperatures.h>
#include <ff_hw_msgs/Reset.h>
#include <ff_hw_msgs/RingBuzzer.h>
#include <ff_hw_msgs/SetEnabled.h>
#include <ff_hw_msgs/Undock.h>

// PMC helper libraries
#include <eps_driver/eps_driver.h>

#include <cerrno>
#include <cstring>
#include <functional>

/**
 * \ingroup hw
 */
namespace eps_driver {

class EpsDriverNode : public ff_util::FreeFlyerNodelet {
 public:
  // Constructor
  EpsDriverNode() : ff_util::FreeFlyerNodelet(NODE_EPS_DRIVER), eps_(nullptr) {}

  // Destructor - make sure we free dynamically allocated memory
  ~EpsDriverNode() {
    if (eps_) delete eps_;
  }

 protected:
  void Initialize(ros::NodeHandle *nh) {
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
      srv_conf_payload_power_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_CONF_PAYLOAD_POWER,
                               &EpsDriverNode::PayloadConfigureCallback, this);
    }
    if (en_conf_advanced_power_) {
      srv_conf_advanced_power_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_CONF_ADVANCED_POWER,
                               &EpsDriverNode::AdvancedConfigureCallback, this);
    }
    if (en_conf_led_state_) {
      srv_conf_led_state_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_CONF_LED_STATE,
                               &EpsDriverNode::LedsConfigureCallback, this);
    }
    if (en_ring_buzzer_) {
      srv_ring_buzzer_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_RING_BUZZER,
                               &EpsDriverNode::RingBuzzerCallback, this);
    }
    if (en_enable_pmcs_) {
      srv_enable_pmcs_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_ENABLE_PMCS,
                               &EpsDriverNode::EnablePmcsCallback, this);
    }
    if (en_get_battery_status_) {
      srv_get_battery_status_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_GET_BATTERY_STATUS,
                               &EpsDriverNode::GetBatteryStatusCallback, this);
    }
    if (en_get_temperatures_) {
      srv_get_temperatures_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_GET_TEMPERATURES,
                               &EpsDriverNode::GetTemperaturesCallback, this);
    }
    if (en_undock_) {
      srv_undock_ = nh->advertiseService(SERVICE_HARDWARE_EPS_UNDOCK,
                                         &EpsDriverNode::UndockCallback, this);
    }
    if (en_get_board_info_) {
      srv_get_board_info_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_GET_BOARD_INFO,
                               &EpsDriverNode::GetBoardInfoCallback, this);
    }
    if (en_clear_terminate_) {
      srv_clear_terminate_ =
          nh->advertiseService(SERVICE_HARDWARE_EPS_CLEAR_TERMINATE,
                               &EpsDriverNode::ClearTerminateCallback, this);
    }

    // Setup telemetry publishers
    pub_power_ = nh->advertise<ff_hw_msgs::EpsPowerState>(
        TOPIC_HARDWARE_EPS_POWER_STATE, telemetry_queue_size_);
    pub_housekeeping_ = nh->advertise<ff_hw_msgs::EpsHousekeeping>(
        TOPIC_HARDWARE_EPS_HOUSEKEEPING, telemetry_queue_size_);
    pub_batt_tl_ = nh->advertise<sensor_msgs::BatteryState>(
        TOPIC_HARDWARE_EPS_BATTERY_STATE_TL, telemetry_queue_size_);
    pub_batt_tr_ = nh->advertise<sensor_msgs::BatteryState>(
        TOPIC_HARDWARE_EPS_BATTERY_STATE_TR, telemetry_queue_size_);
    pub_batt_bl_ = nh->advertise<sensor_msgs::BatteryState>(
        TOPIC_HARDWARE_EPS_BATTERY_STATE_BL, telemetry_queue_size_);
    pub_batt_br_ = nh->advertise<sensor_msgs::BatteryState>(
        TOPIC_HARDWARE_EPS_BATTERY_STATE_BR, telemetry_queue_size_);
    pub_temp_tl_ = nh->advertise<sensor_msgs::Temperature>(
        TOPIC_HARDWARE_EPS_BATTERY_TEMP_TL, telemetry_queue_size_);
    pub_temp_tr_ = nh->advertise<sensor_msgs::Temperature>(
        TOPIC_HARDWARE_EPS_BATTERY_TEMP_TR, telemetry_queue_size_);
    pub_temp_bl_ = nh->advertise<sensor_msgs::Temperature>(
        TOPIC_HARDWARE_EPS_BATTERY_TEMP_BL, telemetry_queue_size_);
    pub_temp_br_ = nh->advertise<sensor_msgs::Temperature>(
        TOPIC_HARDWARE_EPS_BATTERY_TEMP_BR, telemetry_queue_size_);
    pub_dock_state_ = nh->advertise<ff_hw_msgs::EpsDockStateStamped>(
        TOPIC_HARDWARE_EPS_DOCK_STATE, telemetry_queue_size_);

    // Setup a ros timer to publish telemetry at a fixed rate
    timer_telemetry_ =
        nh->createTimer(ros::Rate(telemetry_pub_rate_),
                        &EpsDriverNode::TelemetryCallback, this, false, true);
    timer_dock_check_ =
        nh->createTimer(ros::Rate(dock_check_rate_),
                        &EpsDriverNode::DockCheckCallback, this, false, true);
    timer_fault_check_ =
        nh->createTimer(ros::Rate(fault_check_rate_),
                        &EpsDriverNode::FaultCheckCallback, this, false, true);
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
    if (eps_) delete eps_;
    eps_ = new EPS(
        device, std::bind(&EpsDriverNode::Sleep, this, std::placeholders::_1));
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
    if (!config_params.GetBool("en_srv_eps_reset", &en_eps_reset_)) {
      ROS_FATAL("EPS: couldn't get the enable eps reset flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_conf_led_state", &en_conf_led_state_)) {
      ROS_FATAL("EPS: couldn't get the enable led config flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_conf_payload_power",
                               &en_conf_payload_power_)) {
      ROS_FATAL("EPS: couldn't get the enable power state flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_conf_advanced_power",
                               &en_conf_advanced_power_)) {
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
    if (!config_params.GetBool("en_srv_get_battery_status",
                               &en_get_battery_status_)) {
      ROS_FATAL("EPS: couldn't get the enable query battery status flag");
      return false;
    }
    if (!config_params.GetBool("en_srv_get_temperatures",
                               &en_get_temperatures_)) {
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
    if (!config_params.GetBool("en_srv_clear_terminate",
                               &en_clear_terminate_)) {
      ROS_FATAL("EPS: couldn't get the enable clear terminate flag");
      return false;
    }
    if (!config_params.GetBool("en_pub_housekeeping", &en_pub_housekeeping_)) {
      ROS_FATAL("EPS: couldn't get the enable publish housekeeping flag");
      return false;
    }
    if (!config_params.GetBool("en_pub_power", &en_pub_power_)) {
      ROS_FATAL("EPS: couldn't get the enable publish power flag");
      return false;
    }
    if (!config_params.GetBool("en_pub_battery_status",
                               &en_pub_battery_status_)) {
      ROS_FATAL("EPS: couldn't get the enable publish battery status flag");
      return false;
    }
    if (!config_params.GetBool("en_pub_dock_state", &en_pub_dock_state_)) {
      ROS_FATAL("EPS: couldn't get the enable publish dock state flag");
      return false;
    }
    if (!config_params.GetUInt("telemetry_queue_size",
                               &telemetry_queue_size_)) {
      ROS_FATAL("EPS: telemetry queue size not specified!");
      return false;
    }
    if (!config_params.GetPosReal("telemetry_pub_rate", &telemetry_pub_rate_)) {
      ROS_FATAL("EPS: telemetry publish rate not specified!");
      return false;
    }
    if (!config_params.GetPosReal("dock_check_rate", &dock_check_rate_)) {
      ROS_FATAL("EPS: dock check rate not specified!");
      return false;
    }
    if (!config_params.GetPosReal("fault_check_rate", &fault_check_rate_)) {
      ROS_FATAL("EPS: fault check rate not specified!");
      return false;
    }
    return true;
  }

  // Safely exit
  void Exit(int status) { exit(status); }

  // Convert battery information from EPS to ROS message
  sensor_msgs::BatteryState BatteryStateConversion(
      EPS::BatteryInfo const &info, std_msgs::Header const &header) {
    // Populate the battery data
    sensor_msgs::BatteryState state;
    state.header = header;
    // True if the battery is present
    state.present = info.present;
    if (info.present) {
      // Voltage in Volts (Mandatory)
      state.voltage = static_cast<float>(info.voltage) / 1000.0f;
      // Negative when discharging (A)  (If unmeasured NaN)
      state.current = static_cast<float>(info.current) / 1000.0f;
      // Current charge in Ah  (If unmeasured NaN)
      state.charge = static_cast<float>(info.remaining) / 1000.0f;
      // Capacity in Ah (last full capacity)  (If unmeasured NaN)
      state.capacity = static_cast<float>(info.full) / 1000.0f;
      // Capacity in Ah (design capacity)  (If unmeasured NaN)
      state.design_capacity = static_cast<float>(info.design) / 1000.0f;
      // Charge percentage on 0 to 1 range  (If unmeasured NaN)
      state.percentage = static_cast<float>(info.percentage) / 100.0;
      // An array of individual cell voltages for each cell in the pack
      for (int i = 0; i < 4; i++)
        state.cell_voltage.push_back(static_cast<float>(info.cell[i]) /
                                     1000.0f);
      state.serial_number = std::to_string(info.serial);
      // FIXME: The battery chemistry is statically set to Li-Ion
      state.power_supply_technology =
          sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
      // Converting SmartBattery status to BatteryState.power_supply_status
      // SmartBattery specification status bit
      // BAT_STAT_INITIALIZED 0x0080
      // BAT_STAT_DISCHARGING 0x0040
      // BAT_STAT_FULLY_CHARGED 0x0020
      // BAT_STAT_FULLY_DISCHARGED 0x0010
      if (info.status & 0x0020) {
        state.power_supply_status =
          sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
      } else if (!(info.status & 0x0040)) {
        state.power_supply_status =
          sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      } else {
        state.power_supply_status =
          sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
      }
      state.power_supply_health =
        sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    } else {
      state.voltage = nanf("");
      state.current = nanf("");
      state.charge = nanf("");
      state.capacity = nanf("");
      state.design_capacity = nanf("");
      state.percentage = nanf("");
      for (int i = 0; i < 4; i++) state.cell_voltage.push_back(nanf(""));
      state.power_supply_technology =
          sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
      state.power_supply_status =
          sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
      state.power_supply_health =
          sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
      state.serial_number = "";
    }
    return state;
  }

  // Convert battery temperature information from EPS to ROS message
  sensor_msgs::Temperature BatteryTemperatureConversion(
      EPS::BatteryInfo const &info, std_msgs::Header const &header) {
    sensor_msgs::Temperature temperature;
    temperature.header = header;
    // Temperature in degree Celsius.
    if (info.present) {
      temperature.temperature =
          static_cast<float>(info.temperature) / 10.0 - 273.15;
      temperature.variance = 0.0;
    } else {
      temperature.temperature = nanf("");
      temperature.variance = nanf("");
    }
    return temperature;
  }

  // Callback for pulling telemetry
  void TelemetryCallback(const ros::TimerEvent &) {
    if (!eps_) return;

    // Header info
    static std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = GetPlatform();  // Robot name

    // If we should publish the housekeeping data
    if (en_pub_housekeeping_) {
      // Publish the channel information
      static ff_hw_msgs::EpsHousekeeping msg;
      msg.header = header;
      std::map<EPS::Housekeeping, double> data;
      if (eps_->GetHousekeeping(EPS::EVERYTHING, data)) {
        std::map<std::string, double> keys;
        keys["AGND1_V"] = data[EPS::HK_AGND1_V];
        keys["SUPPLY_IN_V"] = data[EPS::HK_SUPPLY_IN_V];
        keys["PAYLOAD_PWR3_I"] = data[EPS::HK_PAYLOAD_PWR3_I];
        keys["SUBSYS1_1_PWR_V"] = data[EPS::HK_SUBSYS1_1_PWR_V];
        keys["SUBSYS1_2_PWR_V"] = data[EPS::HK_SUBSYS1_2_PWR_V];
        keys["UNREG_V"] = data[EPS::HK_UNREG_V];
        keys["SYSTEM_I"] = data[EPS::HK_SYSTEM_I];
        keys["BAT4V_V"] = data[EPS::HK_BAT4V_V];
        keys["BAT3V_V"] = data[EPS::HK_BAT3V_V];
        keys["BAT2V_V"] = data[EPS::HK_BAT2V_V];
        keys["BAT1V_V"] = data[EPS::HK_BAT1V_V];
        keys["SUPPLY_I"] = data[EPS::HK_SUPPLY_I];
        keys["5VLIVE_V"] = data[EPS::HK_5VLIVE_V];
        keys["AGND2_V"] = data[EPS::HK_AGND2_V];
        keys["FAN_PWR_I"] = data[EPS::HK_FAN_PWR_I];
        keys["AUX_PWR_I"] = data[EPS::HK_AUX_PWR_I];
        keys["PAYLOAD_PWR4_I"] = data[EPS::HK_PAYLOAD_PWR4_I];
        keys["PAYLOAD_PWR2_I"] = data[EPS::HK_PAYLOAD_PWR2_I];
        keys["PAYLOAD_PWR1_I"] = data[EPS::HK_PAYLOAD_PWR1_I];
        keys["5A_REG1_PWR_I"] = data[EPS::HK_5A_REG1_PWR_I];
        keys["MOTOR1_I"] = data[EPS::HK_MOTOR1_I];
        keys["SUBSYS2_PWR_V"] = data[EPS::HK_SUBSYS2_PWR_V];
        keys["MOTOR2_I"] = data[EPS::HK_MOTOR2_I];
        keys["5A_REG2_PWR_I"] = data[EPS::HK_5A_REG2_PWR_I];
        keys["5A_REG3_PWR_I"] = data[EPS::HK_5A_REG3_PWR_I];
        keys["MAIN5_PWR_I"] = data[EPS::HK_MAIN5_PWR_I];
        keys["AUO_PWR_I"] = data[EPS::HK_AUO_PWR_I];
        keys["HLP_I"] = data[EPS::HK_HLP_I];
        keys["USB_PWR_I"] = data[EPS::HK_USB_PWR_I];
        keys["LLP_I"] = data[EPS::HK_LLP_I];
        keys["MLP_I"] = data[EPS::HK_MLP_I];
        keys["ENET_PWR_I"] = data[EPS::HK_ENET_PWR_I];
        std::map<std::string, double>::iterator it;
        // clear previous vector of channels
        msg.values.clear();
        for (it = keys.begin(); it != keys.end(); it++) {
          static ff_hw_msgs::EpsHousekeepingValue keyvalue;
          keyvalue.name = it->first;
          keyvalue.value = it->second;
          msg.values.push_back(keyvalue);
        }
      }
      pub_housekeeping_.publish(msg);
    }
    // If we should publish the housekeeping data
    if (en_pub_power_) {
      // Publish the channel information
      static ff_hw_msgs::EpsPowerState msg;
      msg.header = header;
      std::map<EPS::Channel, bool> data;
      if (eps_->GetChannels(EPS::EVERYTHING, data)) {
        std::map<std::string, double> keys;
        keys["LLP_EN"] = data[EPS::CHANNEL_LLP_EN];
        keys["MLP_EN"] = data[EPS::CHANNEL_MLP_EN];
        keys["HLP_EN"] = data[EPS::CHANNEL_HLP_EN];
        keys["USB_PWR_EN"] = data[EPS::CHANNEL_USB_PWR_EN];
        keys["AUX_PWR_EN"] = data[EPS::CHANNEL_AUX_PWR_EN];
        keys["ENET_PWR_EN"] = data[EPS::CHANNEL_ENET_PWR_EN];
        keys["FAN_EN"] = data[EPS::CHANNEL_FAN_EN];
        keys["SPEAKER_EN"] = data[EPS::CHANNEL_SPEAKER_EN];
        keys["PAYLOAD_EN_TOP_AFT"] = data[EPS::CHANNEL_PAYLOAD_EN_TOP_AFT];
        keys["PAYLOAD_EN_BOT_AFT"] = data[EPS::CHANNEL_PAYLOAD_EN_BOT_AFT];
        keys["PAYLOAD_EN_BOT_FRONT"] = data[EPS::CHANNEL_PAYLOAD_EN_BOT_FRONT];
        keys["PAYLOAD_EN_TOP_FRONT"] = data[EPS::CHANNEL_PAYLOAD_EN_TOP_FRONT];
        keys["MOTOR_EN1"] = data[EPS::CHANNEL_MOTOR_EN1];
        keys["MOTOR_EN2"] = data[EPS::CHANNEL_MOTOR_EN2];
        keys["RESERVED0"] = data[EPS::CHANNEL_RESERVED0];
        keys["RESERVED1"] = data[EPS::CHANNEL_RESERVED1];
        keys["STATUSA2_LED"] = data[EPS::CHANNEL_STATUSA2_LED];
        keys["STATUSA1_LED"] = data[EPS::CHANNEL_STATUSA1_LED];
        keys["STATUSB2_LED"] = data[EPS::CHANNEL_STATUSB2_LED];
        keys["STATUSB1_LED"] = data[EPS::CHANNEL_STATUSB1_LED];
        keys["STATUSC2_LED"] = data[EPS::CHANNEL_STATUSC2_LED];
        keys["STATUSC1_LED"] = data[EPS::CHANNEL_STATUSC1_LED];
        keys["RESERVED2"] = data[EPS::CHANNEL_RESERVED2];
        keys["RESERVED3"] = data[EPS::CHANNEL_RESERVED3];
        keys["VIDEO_LED"] = data[EPS::CHANNEL_VIDEO_LED];
        keys["AUDIO_LED"] = data[EPS::CHANNEL_AUDIO_LED];
        keys["LIVE_LED"] = data[EPS::CHANNEL_LIVE_LED];
        keys["RESERVED4"] = data[EPS::CHANNEL_RESERVED4];
        keys["RESERVED5"] = data[EPS::CHANNEL_RESERVED5];
        keys["RESERVED6"] = data[EPS::CHANNEL_RESERVED6];
        keys["RESERVED7"] = data[EPS::CHANNEL_RESERVED7];
        keys["RESERVED8"] = data[EPS::CHANNEL_RESERVED8];
        std::map<std::string, double>::iterator it;
        // clear previous vector of channels
        msg.values.clear();
        for (it = keys.begin(); it != keys.end(); it++) {
          static ff_hw_msgs::EpsPowerStateValue keyvalue;
          keyvalue.name = it->first;
          keyvalue.value = it->second;
          msg.values.push_back(keyvalue);
        }
      }
      pub_power_.publish(msg);
    }
    // If we should publish the battery stateus
    if (en_pub_battery_status_) {
      std::map<EPS::Battery, EPS::BatteryInfo> data;
      if (eps_->GetBatteries(EPS::EVERYTHING, data)) {
        std::map<EPS::Battery, EPS::BatteryInfo>::iterator it;
        for (it = data.begin(); it != data.end(); it++) {
          sensor_msgs::BatteryState battery =
              BatteryStateConversion(it->second, header);
          sensor_msgs::Temperature temperature =
              BatteryTemperatureConversion(it->second, header);
          switch (it->first) {
            case EPS::BATTERY_TOP_LEFT:
              battery.location = ff_hw_msgs::EpsBatteryLocation::TOP_LEFT;
              pub_batt_tl_.publish(battery);
              pub_temp_tl_.publish(temperature);
              break;
            case EPS::BATTERY_BOTTOM_LEFT:
              battery.location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT;
              pub_batt_bl_.publish(battery);
              pub_temp_bl_.publish(temperature);
              break;
            case EPS::BATTERY_TOP_RIGHT:
              battery.location = ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT;
              pub_batt_tr_.publish(battery);
              pub_temp_tr_.publish(temperature);
              break;
            case EPS::BATTERY_BOTTOM_RIGHT:
              battery.location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT;
              pub_batt_br_.publish(battery);
              pub_temp_br_.publish(temperature);
              break;
            default:
              break;
          }
        }
      }
    }
  }

  // Callback for pulling telemetry
  void DockCheckCallback(const ros::TimerEvent &) {
    if (!eps_) return;
    // If we should publish the dock state
    if (en_pub_dock_state_) {
      static ff_hw_msgs::EpsDockStateStamped msg;
      msg.header.frame_id = GetPlatform();
      msg.header.stamp = ros::Time::now();
      msg.state = ff_hw_msgs::EpsDockStateStamped::UNKNOWN;
      // Try and query the state
      std::map<EPS::State, uint8_t> states;
      if (eps_->GetStates(1 << EPS::STATE_DOCK, states)) {
        switch (states[EPS::STATE_DOCK]) {
          case EPS::DOCK_DISCONNECTED:
            msg.state = ff_hw_msgs::EpsDockStateStamped::UNDOCKED;
            break;
          case EPS::DOCK_CONNECTING:
            msg.state = ff_hw_msgs::EpsDockStateStamped::CONNECTING;
            break;
          case EPS::DOCK_CONNECTED:
            msg.state = ff_hw_msgs::EpsDockStateStamped::DOCKED;
            break;
          default:
            msg.state = ff_hw_msgs::EpsDockStateStamped::UNKNOWN;
            break;
        }
      }
      pub_dock_state_.publish(msg);
    }
  }

  // Callback for fault checking
  void FaultCheckCallback(const ros::TimerEvent &) {
    if (!eps_) return;
    // Check for all faults
    std::map<EPS::Fault, bool> data;
    if (!eps_->GetFaults(EPS::EVERYTHING, data))
      ROS_WARN_STREAM("Could not query faults from EPS");
    // TODO(asymingt) Assert the faults as needed
    /*
    if (data[EPS::FAULT_OC_ENET])         AssertFault("");
    if (data[EPS::FAULT_OT_FLASHLIGHT_1]) AssertFault("");
    if (data[EPS::FAULT_OT_FLASHLIGHT_2]) AssertFault("");
    if (data[EPS::FAULT_OC_FAN])          AssertFault("");
    if (data[EPS::FAULT_OT_MLP])          AssertFault("");
    if (data[EPS::FAULT_OT_LLP])          AssertFault("");
    if (data[EPS::FAULT_OT_HLP])          AssertFault("");
    if (data[EPS::FAULT_OC_USB])          AssertFault("");
    if (data[EPS::FAULT_OC_LLP])          AssertFault("");
    if (data[EPS::FAULT_OC_MLP])          AssertFault("");
    if (data[EPS::FAULT_OC_HLP])          AssertFault("");
    if (data[EPS::FAULT_OC_AUX])          AssertFault("");
    if (data[EPS::FAULT_ST_5A_REG_3])     AssertFault("");
    if (data[EPS::FAULT_OC_5A_REG_2])     AssertFault("");
    if (data[EPS::FAULT_OC_5A_REG_1])     AssertFault("");
    if (data[EPS::FAULT_ST_5A_REG_2])     AssertFault("");
    if (data[EPS::FAULT_OC_PAYLOAD_4])    AssertFault("");
    if (data[EPS::FAULT_ST_5A_REG_1])     AssertFault("");
    if (data[EPS::FAULT_OC_PAYLOAD_1])    AssertFault("");
    if (data[EPS::FAULT_OC_5A_REG_3])     AssertFault("");
    if (data[EPS::FAULT_OC_PAYLOAD_2])    AssertFault("");
    if (data[EPS::FAULT_OC_PAYLOAD_3])    AssertFault("");
    */
  }

  // Callback for resetting the EPS
  bool ResetCallback(ff_hw_msgs::Reset::Request &req,
                     ff_hw_msgs::Reset::Response &res) {
    if (!eps_) return false;
    res.success = eps_->Reboot();
    if (!res.success) res.status_message = "Could not reset the EPS";
    return true;
  }

  // Callback for configuring the LEDs
  bool LedsConfigureCallback(ff_hw_msgs::ConfigureSystemLeds::Request &req,
                             ff_hw_msgs::ConfigureSystemLeds::Response &res) {
    if (!eps_) return false;
    // Convert to batch operations
    std::map<EPS::LedMode, uint32_t> batches;
    for (uint32_t i = 0; i < EPS::NUM_LEDS; i++) {
      uint8_t val = ff_hw_msgs::ConfigureSystemLeds::Request::PERSIST;
      switch (i) {
        case EPS::LED_SA1:
          val = req.status_a1;
          break;
        case EPS::LED_SA2:
          val = req.status_a2;
          break;
        case EPS::LED_SB1:
          val = req.status_b1;
          break;
        case EPS::LED_SB2:
          val = req.status_b2;
          break;
        case EPS::LED_SC1:
          val = req.status_c1;
          break;
        case EPS::LED_SC2:
          val = req.status_c2;
          break;
        case EPS::LED_VIDEO:
          val = req.video;
          break;
        case EPS::LED_AUDIO:
          val = req.audio;
          break;
        case EPS::LED_LIVE:
          val = req.live;
          break;
      }
      switch (val) {
        default:
        case ff_hw_msgs::ConfigureSystemLeds::Request::PERSIST:
          continue;
        case ff_hw_msgs::ConfigureSystemLeds::Request::ON:
          batches[EPS::LED_MODE_ON] |= (1 << i);
          break;
        case ff_hw_msgs::ConfigureSystemLeds::Request::OFF:
          batches[EPS::LED_MODE_OFF] |= (1 << i);
          break;
        case ff_hw_msgs::ConfigureSystemLeds::Request::SLOW:
          batches[EPS::LED_MODE_BLINK_0_5HZ] |= (1 << i);
          break;
        case ff_hw_msgs::ConfigureSystemLeds::Request::MEDIUM:
          batches[EPS::LED_MODE_BLINK_1HZ] |= (1 << i);
          break;
        case ff_hw_msgs::ConfigureSystemLeds::Request::FAST:
          batches[EPS::LED_MODE_BLINK_2HZ] |= (1 << i);
          break;
      }
    }
    // Apply the operation in batches
    std::map<EPS::LedMode, uint32_t>::iterator it;
    for (it = batches.begin(); it != batches.end(); it++) {
      if (!eps_->SetLeds(it->second, it->first)) {
        res.success = false;
        res.status = "Could not set all of the LED states";
        return true;
      }
    }
    // Success!
    res.success = true;
    res.status = "All LED set sucessfully";
    return true;
  }

  // Callback for setting the power state
  bool PayloadConfigureCallback(
      ff_hw_msgs::ConfigurePayloadPower::Request &req,
      ff_hw_msgs::ConfigurePayloadPower::Response &res) {
    if (!eps_) return false;
    // Batch the request
    std::map<bool, uint32_t> bat;
    uint8_t const &on = ff_hw_msgs::ConfigurePayloadPower::Request::ON;
    uint8_t const &off = ff_hw_msgs::ConfigurePayloadPower::Request::OFF;
    if (req.top_front == on)
      bat[true] |= (1 << EPS::CHANNEL_PAYLOAD_EN_TOP_FRONT);
    if (req.top_front == off)
      bat[false] |= (1 << EPS::CHANNEL_PAYLOAD_EN_TOP_FRONT);
    if (req.bottom_front == on)
      bat[true] |= (1 << EPS::CHANNEL_PAYLOAD_EN_BOT_FRONT);
    if (req.bottom_front == off)
      bat[false] |= (1 << EPS::CHANNEL_PAYLOAD_EN_BOT_FRONT);
    if (req.top_aft == on)
      bat[true] |= (1 << EPS::CHANNEL_PAYLOAD_EN_TOP_AFT);
    if (req.top_aft == off)
      bat[false] |= (1 << EPS::CHANNEL_PAYLOAD_EN_TOP_AFT);
    if (req.bottom_aft == on)
      bat[true] |= (1 << EPS::CHANNEL_PAYLOAD_EN_BOT_AFT);
    if (req.bottom_aft == off)
      bat[false] |= (1 << EPS::CHANNEL_PAYLOAD_EN_BOT_AFT);
    // Apply the operation in batches
    std::map<bool, uint32_t>::iterator it;
    for (it = bat.begin(); it != bat.end(); it++) {
      if (!eps_->SetChannels(it->second, it->first)) {
        res.success = false;
        res.status = "Could not set one of the payload powers";
        return true;
      }
    }
    // Success!
    res.success = true;
    res.status = "All payload power set sucessfully";
    return true;
  }

  // Callback for setting the power channels
  bool AdvancedConfigureCallback(
      ff_hw_msgs::ConfigureAdvancedPower::Request &req,
      ff_hw_msgs::ConfigureAdvancedPower::Response &res) {
    if (!eps_) return false;
    // Batch the request
    std::map<bool, uint32_t> bat;
    uint8_t const &on = ff_hw_msgs::ConfigureAdvancedPower::Request::ON;
    uint8_t const &off = ff_hw_msgs::ConfigureAdvancedPower::Request::OFF;
    if (req.usb == on) bat[true] |= (1 << EPS::CHANNEL_USB_PWR_EN);
    if (req.usb == off) bat[false] |= (1 << EPS::CHANNEL_USB_PWR_EN);
    if (req.aux == on) bat[true] |= (1 << EPS::CHANNEL_AUX_PWR_EN);
    if (req.aux == off) bat[false] |= (1 << EPS::CHANNEL_AUX_PWR_EN);
    if (req.pmc1 == on) bat[true] |= (1 << EPS::CHANNEL_MOTOR_EN1);
    if (req.pmc1 == off) bat[false] |= (1 << EPS::CHANNEL_MOTOR_EN1);
    if (req.pmc2 == on) bat[true] |= (1 << EPS::CHANNEL_MOTOR_EN2);
    if (req.pmc2 == off) bat[false] |= (1 << EPS::CHANNEL_MOTOR_EN2);
    std::map<bool, uint32_t>::iterator it;
    for (it = bat.begin(); it != bat.end(); it++) {
      if (!eps_->SetChannels(it->second, it->first)) {
        res.success = false;
        res.status = "Could not set one of the advanced powers";
        return true;
      }
    }
    // Success!
    res.success = true;
    res.status = "All advanced power set sucessfully";
    return true;
  }

  // Callback for ringing the buzzer
  bool RingBuzzerCallback(ff_hw_msgs::RingBuzzer::Request &req,
                          ff_hw_msgs::RingBuzzer::Response &res) {
    if (!eps_) return false;
    if (eps_->RingBuzzer(req.frequency, req.duration)) {
      res.success = true;
      res.status_message = "Buzzer rung successfully";
    } else {
      res.success = false;
      res.status_message = "Could not ring the buzzer";
    }
    return true;
  }

  // Callback for enabling the PMCs
  bool EnablePmcsCallback(ff_hw_msgs::SetEnabled::Request &req,
                          ff_hw_msgs::SetEnabled::Response &res) {
    if (!eps_) return false;
    uint32_t mask =
        (1 << EPS::CHANNEL_MOTOR_EN1) | (1 << EPS::CHANNEL_MOTOR_EN2);
    if (eps_->SetChannels(mask, req.enabled)) {
      res.success = true;
      res.status_message = "PMSs toggled successfuly";
    } else {
      res.success = false;
      res.status_message = "Could not toggle PMCs";
    }
    return true;
  }

  // Callback for retrieving the battery status
  bool GetBatteryStatusCallback(ff_hw_msgs::GetBatteryStatus::Request &req,
                                ff_hw_msgs::GetBatteryStatus::Response &res) {
    if (!eps_) return false;
    EPS::Battery bid;
    if (req.location == ff_hw_msgs::EpsBatteryLocation::TOP_LEFT)
      bid = EPS::BATTERY_TOP_LEFT;
    if (req.location == ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT)
      bid = EPS::BATTERY_BOTTOM_LEFT;
    if (req.location == ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT)
      bid = EPS::BATTERY_TOP_RIGHT;
    if (req.location == ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT)
      bid = EPS::BATTERY_BOTTOM_RIGHT;
    // Get the battery data
    std::map<EPS::Battery, EPS::BatteryInfo> data;
    if (eps_->GetBatteries(1 << bid, data)) {
      static std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = GetPlatform();
      res.state = BatteryStateConversion(data[bid], header);
      res.temperature = BatteryTemperatureConversion(data[bid], header);
      res.success = true;
      res.status_message = "Successfully queried battery state";
    } else {
      res.success = false;
      res.status_message = "Could not query the battery state";
    }
    return true;
  }

  // Callback for retrieving the temperatures
  bool GetTemperaturesCallback(ff_hw_msgs::GetTemperatures::Request &req,
                               ff_hw_msgs::GetTemperatures::Response &res) {
    if (!eps_) return false;
    std::map<EPS::Temp, EPS::TempInfo> data;
    if (eps_->GetTemps(EPS::EVERYTHING, data)) {
      res.top = data[EPS::TEMP_TOP].temp;
      res.bottom = data[EPS::TEMP_BOTTOM].temp;
      res.connector = data[EPS::TEMP_CONNECTOR].temp;
      res.success = true;
      res.status_message = "Successfully retreiecved the temperatures";
    } else {
      res.success = false;
      res.status_message = "Could not retrieve the battery status";
    }
    return true;
  }

  // Callback for undocking
  bool UndockCallback(ff_hw_msgs::Undock::Request &req,
                      ff_hw_msgs::Undock::Response &res) {
    if (!eps_) return false;
    if (eps_->Undock())
      res.value = ff_hw_msgs::Undock::Response::SUCCESS;
    else
      res.value = ff_hw_msgs::Undock::Response::UNDOCK_FAILED;
    return true;
  }

  // Callback for get board information
  bool GetBoardInfoCallback(ff_hw_msgs::GetBoardInfo::Request &req,
                            ff_hw_msgs::GetBoardInfo::Response &res) {
    if (!eps_) return false;
    std::map<EPS::String, std::string> data;
    if (eps_->GetStrings(EPS::EVERYTHING, data)) {
      res.sw_version = data[EPS::STRING_SW_VERSION];
      res.build = data[EPS::STRING_BUILD];
      res.serial = data[EPS::STRING_SERIAL];
      res.success = true;
      res.status_message = "Board information acquired";
    } else {
      res.success = false;
      res.status_message = "Could get board information";
    }
    return true;
  }

  // Callback for clear terminate
  bool ClearTerminateCallback(ff_hw_msgs::ClearTerminate::Request &req,
                              ff_hw_msgs::ClearTerminate::Response &res) {
    if (!eps_) return false;
    if (eps_->Unterminate()) {
      res.success = true;
      res.status_message = "Terminate cleared successfully";
    } else {
      res.success = false;
      res.status_message = "Could not clear terminate";
    }
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
  bool en_pub_power_;                           // Enable publish power states
  bool en_pub_battery_status_;                  // Enable publish battery status
  bool en_pub_dock_state_;                      // Enable publish dock state
  unsigned int telemetry_queue_size_;           // Telemetry queue size
  float telemetry_pub_rate_;                    // Telemetry publication rate
  float dock_check_rate_;                       // Dock check rate
  float fault_check_rate_;                      // Fault check rate
  EPS *eps_;                                    // Interface class to the EPS
  ros::ServiceServer srv_eps_reset_;            // Reset the hardware
  ros::ServiceServer srv_conf_payload_power_;   // Configure LEDs
  ros::ServiceServer srv_conf_advanced_power_;  // Set the power channels
  ros::ServiceServer srv_conf_led_state_;  // Set the power state for a channel
  ros::ServiceServer srv_ring_buzzer_;     // Ring buzzer
  ros::ServiceServer srv_enable_pmcs_;     // Enable PMCs
  ros::ServiceServer srv_get_battery_status_;  // Get battery status
  ros::ServiceServer srv_get_temperatures_;    // Get temperatures
  ros::ServiceServer srv_undock_;              // Undock service
  ros::Timer timer_telemetry_;                 // Telemetry timer
  ros::Timer timer_dock_check_;                // Dock check timer
  ros::Timer timer_fault_check_;               // Fault check timer
  ros::ServiceServer srv_get_board_info_;      // Get board information
  ros::ServiceServer srv_clear_terminate_;     // Clear terminate
  ros::Publisher pub_power_;                   // Power channel states
  ros::Publisher pub_housekeeping_;            // Housekeeping telemetry
  ros::Publisher pub_dock_state_;              // Dock state publisher
  ros::Publisher pub_batt_tl_;                 // Battery: top left
  ros::Publisher pub_batt_tr_;                 // Battery: top right
  ros::Publisher pub_batt_bl_;                 // Battery: bottom left
  ros::Publisher pub_batt_br_;                 // Battery: bottom right
  ros::Publisher pub_temp_tl_;                 // Temperature: top left
  ros::Publisher pub_temp_tr_;                 // Temperature: top right
  ros::Publisher pub_temp_bl_;                 // Temperature: bottom left
  ros::Publisher pub_temp_br_;                 // Temperature: bottom right
};

PLUGINLIB_EXPORT_CLASS(eps_driver::EpsDriverNode, nodelet::Nodelet);

}  // namespace eps_driver
