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

// Ack/Nack
#define I2C_RESP_ACK 0xEE
#define I2C_RESP_NACK 0xFF

// Common commands for EPS and Dock (0x01 - 0x3F)
#define I2C_CMD_NONE 0x00
#define I2C_CMD_GET_HW_VERSION 0x01
#define I2C_CMD_GET_SW_VERSION 0x02
#define I2C_CMD_GET_BUILD_TIME 0x03
#define I2C_CMD_SW_ON 0x04
#define I2C_CMD_SW_OFF 0x05
#define I2C_CMD_GET_SW_STATES 0x06
// 0x07 RESERVED
// 0x08 RESERVED
#define I2C_CMD_GET_HK 0x09
#define I2C_CMD_GET_HW_EXCEPTIONS 0x0A
#define I2C_CMD_CLEAR_HW_EXCEPTIONS 0x0B
#define I2C_CMD_GET_DIGITAL_TEMPS 0x0C
#define I2C_CMD_GET_SERIAL_NUMBER 0x0D
#define I2C_CMD_REBOOT 0x0F
#define I2C_CMD_ENTER_BOOTLOADER 0x0E

// EPS-specific commands (0x40 - 0x6F)
#define I2C_CMD_GET_BATTERY_STATUS 0x40
#define I2C_CMD_RING_BUZZER 0x41
#define I2C_CMD_CLR_TERMINATE_EVT 0x42
#define I2C_CMD_UNDOCK 0x43
#define I2C_CMD_GET_CONNECTION_STATE 0x44

// Dock-specific commands (0x70 - 0x9F)
#define I2C_CMD_GET_DOCK_STATE 0x70
#define I2C_CMD_GET_CONNECTED_EPS_STATE 0x71
#define I2C_CMD_GET_CONN_STATE 0x72
#define I2C_CMD_GET_EPS_CMD 0x75
#define I2C_CMD_SET_EPS_CMD 0x76

// EPS-to-Dock I2C commands (0xA0 - 0xCF)
#define I2C_CMD_SEND_EPS_STATE_TO_DOCK 0xD0

/*
#define I2C_ACK 0xAA
#define I2C_NACK 0xBB
#define I2C_CMD_SW_ON 0x01
#define I2C_CMD_SW_OFF 0x02
#define I2C_CMD_GET_SERIAL_NUMBER 0x08
#define I2C_CMD_REBOOT 0x0E
#define I2C_CMD_ENTER_BOOTLOADER 0x0F
#define I2C_CMD_GET_HK 0x11
#define I2C_CMD_GET_SW_STATES 0x13
#define I2C_CMD_GET_VERSION 0x90
#define I2C_CMD_GET_BUILD_TIME 0x91
#define I2C_CMD_GET_BATTERY_STATUS 0x92
#define I2C_CMD_GET_DIGITAL_TEMPS 0x93
#define I2C_CMD_RING_BUZZER 0x95
*/

#define EPS_NUM_PWR_CHANS               32
#define EPS_NUM_HOUSEKEEPING            32
#define EPS_FIRMWARE_NUM_FILE_RETRIES   10
#define EPS_FIRMWARE_BLOCK_SIZE_BYTES   128
#define EPS_MIN_BUZZER_FREQUENCY        1000
#define EPS_MAX_BUZZER_FREQUENCY        2000
#define EPS_MIN_BUZZER_DURATION         1
#define EPS_MAX_BUZZER_DURATION         10

namespace eps_driver {

// Structure for holding battery information
/*
struct BatteryStatus {
  uint16_t chan;
  int16_t percentage;
  int16_t voltage;
  int16_t current;
  float temperature;
};
*/
struct BatteryStatus {
  uint8_t chan;
  bool present;
  uint16_t voltage;             // mV
  int16_t current;              // mA
  uint16_t charge;              // mAh
  uint16_t capacity;            // mAh
  uint16_t design_capacity;     // mAh
  uint16_t percentage;          // [0..100] %
  uint16_t cell_voltage[4];     // mV
  uint16_t status;              // bit mask
  uint16_t temperature;         // 0.1 K
  uint16_t serial_number;
};

// Structure for holding channel information
struct HousekeepingInfo {
  int channel;
  const char *description;
  double scale;
  double offset;
  double value;
};

// String type
enum StringType {
  HW_VERSION = I2C_CMD_GET_HW_VERSION,
  SW_VERSION = I2C_CMD_GET_SW_VERSION,
  BUILD      = I2C_CMD_GET_BUILD_TIME,
  SERIAL     = I2C_CMD_GET_SERIAL_NUMBER
};

// Battery index
enum BatteryIndex {
  BATTERY_TOP_LEFT     = 0,
  BATTERY_BOTTOM_LEFT  = 1,
  BATTERY_TOP_RIGHT    = 2,
  BATTERY_BOTTOM_RIGHT = 3,
  NUM_BATTERIES        = 4
};

// LED index
enum LedIndex {
  LED_STATUS_1 = 0,
  LED_STATUS_2 = 1,
  LED_STATUS_3 = 2,
  LED_STATUS_4 = 3,
  LED_STATUS_5 = 4,
  LED_STATUS_6 = 5,
  LED_STREAM   = 6,
  LED_CAMERA   = 7,
  LED_MIC      = 8,
  NUM_LEDS     = 9
};

enum PayloadIndex {
  PAYLOAD_TOP_FRONT    = 0,
  PAYLOAD_BOTTOM_FRONT = 1,
  PAYLOAD_TOP_AFT      = 2,
  PAYLOAD_BOTTOM_AFT   = 3,
  NUM_PAYLOADS         = 4
};

enum AdvancedIndex {
  ADVANCED_USB         = 0,
  ADVANCED_AUX         = 1,
  ADVANCED_PMC1        = 2,
  ADVANCED_PMC2        = 3,
  NUM_ADVANCED         = 4
};

// Power state
enum PowerState {
  PERSIST  = 0x00,
  ENABLED  = 0x01,
  DISABLED = 0x02
};

// Digital and analog enumerations
enum TemperatureSensorType {
  DIGITAL  = 0x93,
  ANALOG   = 0x12
};

// Firmware file actions
enum FirmwareFileState {
  OPEN     = 0x03,
  CLOSE    = 0x05,
  READY    = 0x07
};

enum ConnectionState {
  CONN_DISCONNECTED = 0x00,
  CONN_CONNECTING   = 0x01,
  CONN_CONNECTED    = 0x02
};

class EpsDriver {
 public:
  enum ChannelIndex {
    CHANNEL_LLP_EN      = 0,
    CHANNEL_MLP_EN      = 1,
    CHANNEL_HLP_EN      = 2,
    CHANNEL_ENET_PWR_EN = 3,
    CHANNEL_USB_PWR_EN  = 4,
    CHANNEL_AUX_PWR_EN  = 5,
    CHANNEL_RESERVED1   = 6,
    CHANNEL_RESERVED2   = 7,
    CHANNEL_PAYLOAD_EN1 = 8,
    CHANNEL_PAYLOAD_EN2 = 9,
    CHANNEL_PAYLOAD_EN3 = 10,
    CHANNEL_PAYLOAD_EN4 = 11,
    CHANNEL_MOTOR_EN1   = 12,
    CHANNEL_MOTOR_EN2   = 13,
    CHANNEL_RESERVED3   = 14,
    CHANNEL_RESERVED4   = 15,
    CHANNEL_STATUS_LED1 = 16,
    CHANNEL_STATUS_LED2 = 17,
    CHANNEL_STATUS_LED3 = 18,
    CHANNEL_STATUS_LED4 = 19,
    CHANNEL_STATUS_LED5 = 20,
    CHANNEL_STATUS_LED6 = 21,
    CHANNEL_RESERVED5   = 22,
    CHANNEL_RESERVED6   = 23,
    CHANNEL_STREAM_LED  = 24,
    CHANNEL_CAMERA_LED  = 25,
    CHANNEL_MIC_LED     = 26,
    CHANNEL_RESERVED7   = 27,
    CHANNEL_RESERVED8   = 28,
    CHANNEL_RESERVED9   = 29,
    CHANNEL_RESERVED10  = 30,
    CHANNEL_RESERVED11  = 31,
    NUM_CHANNELS        = 32
  };

  // Constructor
  explicit EpsDriver(const i2c::Device &i2c_dev, std::function<void(uint32_t)> usleep_cb);

  // Destructor
  ~EpsDriver(void);

  // Set the version string
  bool GetString(StringType type, std::string & data);

  // Reset the EPS
  bool Reset();

  // Ring the buzzer
  bool RingBuzzer(uint16_t freq, uint8_t secs);

  // Enable the PMCs
  bool EnablePMCs(bool enable);

  // Read the analog temperature sensors
  bool ReadTemperatureSensors(std::vector<double>& data);

  // Set specific power channel on (not a mask, but an integer channel!)
  bool SetPayloadState(PayloadIndex payload, PowerState state);

  // Set specific power channel on (not a mask, but an integer channel!)
  bool SetAdvancedState(AdvancedIndex advanced, PowerState state);

  // Set specific power channel on (not a mask, but an integer channel!)
  bool SetLedState(LedIndex led, PowerState state);

  // Get the battery status for a particular channel
  bool GetBatteryStatus(BatteryIndex battery, BatteryStatus &data);

  // Read housekeeping data for each power channel
  bool ReadHousekeeping(std::vector<HousekeepingInfo> &data);

  // Send UNDOCK command to the dock.
  bool Undock(void);

  // Get Connection State
  bool GetConnectionState(ConnectionState &state);

  // Clear TERMINATE event.
  bool ClearTerminateEvent(void);

  // Write new firmware to the EPS
  bool WriteFirmware(std::string const& hexfile);

  // Set all power switch states
  bool SetAllPowerChannelState(PowerState state);

  // Get all power switch states
  bool GetAllPowerChannelState(std::vector<PowerState> &states);

  // Low level power channel state
  bool SetPowerChannelState(uint8_t channel, PowerState state);

  std::vector<std::string>& GetPowerChannelNames();

 protected:
  // Close the firware file
  bool ChangeFirmwareFileState(FirmwareFileState state);

  // Read an i2c frame + 1 byte checksum
  uint16_t Read(uint8_t *buff);

  // Read an i2c frame size of len.
  uint16_t Read(uint8_t *buff, uint16_t len);

  // Write an i2c frame + 1 byte checksum
  uint16_t Write(uint8_t *buff, uint16_t len);

  // Compute checksum
  uint8_t ComputeChecksum(uint8_t *buf, size_t size);

 private:
  i2c::Device i2c_dev_;                                      // Device
  std::function<void(uint32_t)> usleep_cb_;                  // Fake usleep callback
  HousekeepingInfo hk_info_[EPS_NUM_HOUSEKEEPING];  // Housekeeping info
  std::vector<std::string> power_chan_names_;
};

}  // namespace eps_driver

#endif  // EPS_DRIVER_EPS_DRIVER_H_
