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

#ifndef SMART_DOCK_SMART_DOCK_H_
#define SMART_DOCK_SMART_DOCK_H_

#include <i2c/i2c_new.h>

#include <eps_driver/eps_driver.h>

#include <vector>
#include <string>
#include <map>

namespace smart_dock {

using EPS = eps_driver::EPS;

// Class to arbitrate access to the Smart Dock
class SmartDock {
 public:
  static constexpr uint8_t OFF = 0;
  static constexpr uint8_t ON  = 1;

  // Berth connection state
  typedef enum {
    CONN_DISCONNECTED = 0x00,   // when ASx_LOOP is high
    CONN_CONNECTING   = 0x01,   // when ASx_LOOP goes low
    CONN_CONNECTED    = 0x02,   // Auto CONNECTING to CONNECTED after 5s
    NUM_CONNECTION_STATES
  } ConnectionState;

  // Berth actuator state
  typedef enum {
    ACT_RETRACT     = 0x00,
    ACT_RETRACTING  = 0x01,
    ACT_DEPLOYED    = 0x03
  } ActuatorState;

  // The DockState is the state of the whole dock system
  typedef struct {
    uint8_t serial_number[6];
    uint8_t subsys_1_pwr;
    uint8_t subsys_2_pwr;
    uint8_t led_state;
    uint8_t actuator_state[2];
    uint8_t conn_state[2];
    uint8_t loop_back[2];
    uint32_t flags;
  } DockState;

  // We reuse the various contants from the EPS driver to ensure that we keep
  // the EPS and smart dock consistent with each other.
  typedef struct {
    uint8_t power_state;                                // EPS::DockStateValue
    uint8_t serial[6];                                  // Serial number
    uint8_t terminate;                                  // Terminate button
    uint32_t channel_mask;                              // EPS::Channel
    uint8_t charge_mask;                                // EPS::Charger
    uint8_t dock_state;                                 // EPS::DockStateVal
    EPS::BatteryInfo batteries[EPS::NUM_BATTERIES];     // Battery states
    uint32_t fault_mask;                                // EPS::Fault mask
  } BerthState;

  // All possible EPS commands
  enum BerthCommand {
    COMMAND_UNKNOWN,
    COMMAND_SET_POWER_MODE_HIBERNATE,
    COMMAND_SET_POWER_MODE_AWAKE_NOMINAL,
    COMMAND_SET_POWER_MODE_AWAKE_SAFE,
    COMMAND_SET_POWER_MODE_CRITICAL_FAULT,
    COMMAND_CLEAR_TERMINATE,
    COMMAND_CLEAR_FAULTS,
    COMMAND_ENABLE_ALL_PAYLOADS,
    COMMAND_DISABLE_ALL_PAYLOADS,
    COMMAND_ENABLE_ALL_PMCS,
    COMMAND_DISABLE_ALL_PMCS,
    COMMAND_REBOOT,
    NUM_COMMANDS
  };

  // Berths
  enum Berth {
    BERTH_1,
    BERTH_2,
    NUM_BERTHS
  };

  // String type
  enum String : uint32_t {
    STRING_SW_VERSION,
    STRING_BUILD,
    STRING_SERIAL,
    NUM_STRINGS
  };

  // Channel indexes
  enum Channel : uint32_t {
    CHANNEL_DEV_EN,
    CHANNEL_EC_PWR_EN,
    CHANNEL_DEV2_PWR_EN,
    CHANNEL_DEV1_PWR_EN,
    CHANNEL_RESERVED0,
    CHANNEL_RESERVED1,
    CHANNEL_RESERVED2,
    CHANNEL_RESERVED3,
    CHANNEL_ACT_SIGOUT2,
    CHANNEL_ACT_SIGOUT1,
    CHANNEL_AS2_PWR_EN,
    CHANNEL_AS1_PWR_EN,
    CHANNEL_FAN_EN,
    CHANNEL_RESERVED4,
    CHANNEL_RESERVED5,
    CHANNEL_RESERVED6,
    CHANNEL_LED_1,
    CHANNEL_LED_2,
    CHANNEL_LED_3,
    CHANNEL_LED_4,
    CHANNEL_LED_5,
    CHANNEL_LED_6,
    CHANNEL_RESERVED7,
    CHANNEL_RESERVED8,
    CHANNEL_RESERVED9,
    CHANNEL_RESERVED10,
    CHANNEL_RESERVED11,
    CHANNEL_RESERVED12,
    CHANNEL_RESERVED13,
    CHANNEL_RESERVED14,
    CHANNEL_RESERVED15,
    CHANNEL_RESERVED16,
    NUM_CHANNELS
  };

  // Faults
  enum Fault : uint32_t {
    FAULT_OC_BERTH_1,
    FAULT_OC_BERTH_2,
    FAULT_OC_SYSTEM,
    FAULT_OC_DOCK_PROCESSOR,
    FAULT_OT_CHARGER,
    FAULT_OT_ACTUATOR_1,
    FAULT_OT_ACTUATOR_2,
    FAULT_WDT1_EPS_REBOOT,
    FAULT_WDT2_DOCKCTL_REBOOT,
    FAULT_WDT3_DOCKPC_REBOOT,
    NUM_FAULTS
  };

  // Housekeeping
  enum Housekeeping : uint32_t {
    HK_FAN_MAG_I,
    HK_CHR_V_V,
    HK_CHR_T_PROTECT,
    HK_VLIVE_I,
    HK_MAIN5_PWR_I,
    HK_DEV_I,
    HK_EC_PWR_I,
    HK_A_GND_V1,
    HK_AS2_I,
    HK_AS1_I,
    HK_A_GND_V2,
    HK_DEV1_T_PROTECT,
    HK_DEV2_T_PROTECT,
    HK_A_GND_V3,
    HK_DEV2_I,
    HK_DEV1_I,
    NUM_HOUSEKEEPING
  };

  // LEDs
  enum Led : uint32_t {
    LED_1,
    LED_2,
    LED_3,
    LED_4,
    LED_5,
    LED_6,
    NUM_LEDS
  };

  // LED Mode
  enum LedMode : uint8_t {
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_BLINK_2HZ,
    LED_MODE_BLINK_1HZ,
    LED_MODE_BLINK_0_5HZ,
    NUM_LED_MODES
  };

  // Constructor requires a vaid i2c device and sleep() callback function
  explicit SmartDock(const i2c::Device &i2c_dev);

  // Destructor
  ~SmartDock(void);

  // One-shot
  bool ClearFaults(void);
  bool Reboot(void);
  bool SendBerthCommand(uint32_t mask, BerthCommand const value);

  // Getters
  bool GetStrings(uint32_t mask, std::map<String, std::string> & data);
  bool GetHousekeeping(uint32_t mask, std::map<Housekeeping, double> & data);
  bool GetFaults(uint32_t mask, std::map<Fault, bool> & data);
  bool GetChannels(uint32_t mask, std::map<Channel, bool> & data);
  bool GetBerthStates(uint32_t mask, std::map<Berth, BerthState> & data);
  bool GetSystemState(DockState & data);

  // Setters
  bool SetLeds(uint32_t mask, LedMode const value);
  bool SetChannels(uint32_t mask, bool const value);

 protected:
  static constexpr size_t  I2C_BUF_MAX_LEN                  = 256;
  static constexpr uint8_t I2C_RESP_ACK                     = 0xEE;
  static constexpr uint8_t I2C_RESP_NACK                    = 0xFF;
  static constexpr uint8_t I2C_CMD_NONE                     = 0x00;
  static constexpr uint8_t I2C_CMD_GET_SW_VERSION           = 0x02;
  static constexpr uint8_t I2C_CMD_GET_BUILD_TIME           = 0x03;
  static constexpr uint8_t I2C_CMD_SW_ON                    = 0x04;
  static constexpr uint8_t I2C_CMD_SW_OFF                   = 0x05;
  static constexpr uint8_t I2C_CMD_GET_SW_STATES            = 0x06;
  static constexpr uint8_t I2C_CMD_GET_SYSTEM_STATE         = 0x07;
  static constexpr uint8_t I2C_CMD_SET_LED_MODES            = 0x08;
  static constexpr uint8_t I2C_CMD_GET_HK                   = 0x09;
  static constexpr uint8_t I2C_CMD_GET_HW_EXCEPTIONS        = 0x0A;
  static constexpr uint8_t I2C_CMD_CLR_HW_EXCEPTIONS        = 0x0B;
  static constexpr uint8_t I2C_CMD_GET_DIGITAL_TEMPS        = 0x0C;
  static constexpr uint8_t I2C_CMD_GET_SERIAL_NUMBER        = 0x0D;
  static constexpr uint8_t I2C_CMD_REBOOT                   = 0x0F;
  static constexpr uint8_t I2C_CMD_ENTER_BOOTLOADER         = 0x0E;
  static constexpr uint8_t I2C_CMD_GET_BATTERY_STATUS       = 0x40;
  static constexpr uint8_t I2C_CMD_RING_BUZZER              = 0x41;
  static constexpr uint8_t I2C_CMD_CLR_TERMINATE_EVT        = 0x42;
  static constexpr uint8_t I2C_CMD_UNDOCK                   = 0x43;
  static constexpr uint8_t I2C_CMD_GET_CONNECTION_STATE     = 0x44;
  static constexpr uint8_t I2C_CMD_SET_CHARGE_STATE         = 0x45;
  static constexpr uint8_t I2C_CMD_GET_CHARGE_STATE         = 0x46;
  static constexpr uint8_t I2C_CMD_SET_EPS_POWER_MODE       = 0x47;
  static constexpr uint8_t I2C_CMD_GET_EPS_POWER_MODE       = 0x48;
  static constexpr uint8_t I2C_CMD_GET_CONNECTED_EPS_STATE  = 0x71;
  static constexpr uint8_t I2C_CMD_GET_CONN_STATE           = 0x72;
  static constexpr uint8_t I2C_CMD_GET_EPS_CMD              = 0x75;
  static constexpr uint8_t I2C_CMD_SET_EPS_CMD              = 0x76;
  static constexpr uint8_t I2C_CMD_SEND_EPS_STATE_TO_DOCK   = 0xD0;
  static constexpr uint8_t DOCK_BERTH_ID_MIN                = 1;
  static constexpr uint8_t DOCK_BERTH_ID_MAX                = 2;
  static constexpr size_t kDockStateLength                  = 19;

  // Sleep function easy since ROS is not used
  void Sleep(uint32_t microseconds);

  // Read an i2c frame + 1 byte checksum
  uint16_t Read(uint8_t *buff);

  // Read an i2c frame size of len.
  uint16_t Read(uint8_t *buff, uint16_t len);

  // Write an i2c frame + 1 byte checksum
  uint16_t Write(uint8_t *buff, uint16_t len);

  // Compute checksum
  uint8_t ComputeChecksum(uint8_t *buf, size_t size);

 private:
  i2c::Device i2c_dev_;                             // Device
};

}  // namespace smart_dock

#endif  // SMART_DOCK_SMART_DOCK_H_
