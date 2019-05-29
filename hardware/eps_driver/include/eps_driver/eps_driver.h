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

#ifndef EPS_DRIVER_EPS_DRIVER_H_
#define EPS_DRIVER_EPS_DRIVER_H_

#include <i2c/i2c_new.h>

#include <functional>
#include <vector>
#include <string>
#include <map>

namespace eps_driver {

// Class to arbitrate access to the Electrical Power System
class EPS {
 public:
  // Toggle values
  static constexpr uint32_t EVERYTHING = 0xFFFFFF;
  static constexpr uint8_t OFF = 0;
  static constexpr uint8_t ON  = 1;

  // String type
  enum String : uint32_t {
    STRING_SW_VERSION,
    STRING_BUILD,
    STRING_SERIAL,
    NUM_STRINGS
  };

  // Channel indexes
  enum Channel : uint32_t {
    CHANNEL_LLP_EN,
    CHANNEL_MLP_EN,
    CHANNEL_HLP_EN,
    CHANNEL_USB_PWR_EN,
    CHANNEL_AUX_PWR_EN,
    CHANNEL_ENET_PWR_EN,
    CHANNEL_FAN_EN,
    CHANNEL_SPEAKER_EN,
    CHANNEL_PAYLOAD_EN_TOP_AFT,
    CHANNEL_PAYLOAD_EN_BOT_AFT,
    CHANNEL_PAYLOAD_EN_BOT_FRONT,
    CHANNEL_PAYLOAD_EN_TOP_FRONT,
    CHANNEL_MOTOR_EN1,
    CHANNEL_MOTOR_EN2,
    CHANNEL_RESERVED0,
    CHANNEL_RESERVED1,
    CHANNEL_STATUSA2_LED,
    CHANNEL_STATUSA1_LED,
    CHANNEL_STATUSB2_LED,
    CHANNEL_STATUSB1_LED,
    CHANNEL_STATUSC2_LED,
    CHANNEL_STATUSC1_LED,
    CHANNEL_RESERVED2,
    CHANNEL_RESERVED3,
    CHANNEL_VIDEO_LED,
    CHANNEL_AUDIO_LED,
    CHANNEL_LIVE_LED,
    CHANNEL_RESERVED4,
    CHANNEL_RESERVED5,
    CHANNEL_RESERVED6,
    CHANNEL_RESERVED7,
    CHANNEL_RESERVED8,
    NUM_CHANNELS
  };

  // States
  enum State : uint32_t {
    STATE_POWER,
    STATE_DOCK,
    NUM_STATES
  };

  // Powe state values
  enum PowerStateValue : uint8_t {
    POWER_STATE_UNKNOWN,
    POWER_STATE_HIBERNATE,
    POWER_STATE_AWAKE_NOMINAL,
    POWER_STATE_AWAKE_SAFE,
    POWER_STATE_CRITICAL_FAULT,
    NUM_POWER_STATES
  };

  // Dock state values
  enum DockStateValue : uint8_t {
    DOCK_DISCONNECTED,
    DOCK_CONNECTING,
    DOCK_CONNECTED,
    NUM_DOCK_STATES
  };

  // Faults
  enum Fault : uint32_t {
    FAULT_OC_ENET,
    FAULT_OT_FLASHLIGHT_1,
    FAULT_OT_FLASHLIGHT_2,
    FAULT_OC_FAN,
    FAULT_RESERVED0,
    FAULT_RESERVED1,
    FAULT_RESERVED2,
    FAULT_RESERVED3,
    FAULT_RESERVED4,
    FAULT_OT_MLP,
    FAULT_OT_LLP,
    FAULT_OT_HLP,
    FAULT_RESERVED5,
    FAULT_RESERVED6,
    FAULT_RESERVED7,
    FAULT_RESERVED8,
    FAULT_OC_USB,
    FAULT_OC_LLP,
    FAULT_OC_MLP,
    FAULT_OC_HLP,
    FAULT_OC_AUX,
    FAULT_ST_5A_REG_3,
    FAULT_OC_5A_REG_2,
    FAULT_OC_5A_REG_1,
    FAULT_ST_5A_REG_2,
    FAULT_OC_PAYLOAD_4,
    FAULT_RESERVED9,
    FAULT_ST_5A_REG_1,
    FAULT_OC_PAYLOAD_1,
    FAULT_OC_5A_REG_3,
    FAULT_OC_PAYLOAD_2,
    FAULT_OC_PAYLOAD_3,
    NUM_FAULTS
  };

  // Housekeeping
  enum Housekeeping : uint32_t {
    HK_AGND1_V,
    HK_SUPPLY_IN_V,
    HK_PAYLOAD_PWR3_I,
    HK_SUBSYS1_1_PWR_V,
    HK_SUBSYS1_2_PWR_V,
    HK_UNREG_V,
    HK_SYSTEM_I,
    HK_BAT4V_V,
    HK_BAT3V_V,
    HK_BAT2V_V,
    HK_BAT1V_V,
    HK_SUPPLY_I,
    HK_5VLIVE_V,
    HK_AGND2_V,
    HK_FAN_PWR_I,
    HK_AUX_PWR_I,
    HK_PAYLOAD_PWR4_I,
    HK_PAYLOAD_PWR2_I,
    HK_PAYLOAD_PWR1_I,
    HK_5A_REG1_PWR_I,
    HK_MOTOR1_I,
    HK_SUBSYS2_PWR_V,
    HK_MOTOR2_I,
    HK_5A_REG2_PWR_I,
    HK_5A_REG3_PWR_I,
    HK_MAIN5_PWR_I,
    HK_AUO_PWR_I,
    HK_HLP_I,
    HK_USB_PWR_I,
    HK_LLP_I,
    HK_MLP_I,
    HK_ENET_PWR_I,
    NUM_HOUSEKEEPING
  };

  // Temperatures
  enum Temp : uint32_t {
    TEMP_BOTTOM,
    TEMP_TOP,
    TEMP_CONNECTOR,
    NUM_TEMPERATURES
  };

  // Battery
  enum Battery : uint32_t {
    BATTERY_TOP_RIGHT,
    BATTERY_BOTTOM_RIGHT,
    BATTERY_TOP_LEFT,
    BATTERY_BOTTOM_LEFT,
    NUM_BATTERIES
  };

  // Charge state
  enum Charger : uint8_t {
    CHARGER_TOP_RIGHT,
    CHARGER_BOTTOM_RIGHT,
    CHARGER_TOP_LEFT,
    CHARGER_BOTTOM_LEFT,
    NUM_CHARGERS
  };

  // LEDs
  enum Led : uint32_t {
    LED_SA1,
    LED_SA2,
    LED_SB1,
    LED_SB2,
    LED_SC1,
    LED_SC2,
    LED_VIDEO,
    LED_AUDIO,
    LED_LIVE,
    NUM_LEDS
  };

  // LED mode
  enum LedMode : uint8_t {
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_BLINK_2HZ,
    LED_MODE_BLINK_1HZ,
    LED_MODE_BLINK_0_5HZ,
    NUM_LED_MODES
  };

  // Structure for holding battery information
  struct BatteryInfo {
    uint8_t chan;                 // Channel
    bool present;                 // Is it plegged in?
    uint16_t voltage;             // Voltage in mV
    int16_t current;              // Current draw mA
    uint16_t full;                // Full mAh
    uint16_t remaining;           // Remaining mAh
    uint16_t design;              // Design mAh
    uint16_t percentage;          // [0..100] %
    uint16_t cell[4];             // Cell voltage mV
    uint16_t status;              // bit mask
    uint16_t temperature;         // Cell temp 0.1 K
    uint16_t serial;              // Serial number
  };

  // Temnperature information struct
  struct TempInfo {
    uint8_t addr;                 // i2c address
    double temp;                  // current value
  };

  // Constructor requires a vaid i2c device and sleep() callback function
  explicit EPS(
    const i2c::Device &i2c_dev, std::function<void(uint32_t)> usleep_cb);

  // Destructor
  ~EPS(void);

  // Commands
  bool RingBuzzer(uint16_t freq, uint8_t secs);
  bool EnterBootloader(void);
  bool ClearFaults(void);
  bool Unterminate(void);
  bool Undock(void);
  bool Reboot(void);

  // Getters
  bool GetStrings(uint32_t mask, std::map<String, std::string> & data);
  bool GetStates(uint32_t mask, std::map<State, uint8_t> & data);
  bool GetChannels(uint32_t mask, std::map<Channel, bool> & data);
  bool GetChargers(uint32_t mask, std::map<Charger, bool> & data);
  bool GetFaults(uint32_t mask, std::map<Fault, bool> & data);
  bool GetBatteries(uint32_t mask, std::map<Battery, BatteryInfo> & data);
  bool GetHousekeeping(uint32_t mask, std::map<Housekeeping, double> & data);
  bool GetTemps(uint32_t mask, std::map<Temp, TempInfo> & data);

  // Setters
  bool SetLeds(uint32_t mask, LedMode const value);
  bool SetChargers(uint32_t mask, bool const value);
  bool SetChannels(uint32_t mask, bool const value);
  bool SetPowerState(PowerStateValue const value);

  // Helpers
  static std::string SerialToString(uint8_t serial[6]);

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
  std::function<void(uint32_t)> usleep_cb_;         // Fake usleep callback
};

}  // namespace eps_driver

#endif  // EPS_DRIVER_EPS_DRIVER_H_
