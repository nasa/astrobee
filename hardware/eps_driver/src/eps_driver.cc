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

#include <eps_driver/eps_driver.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <fstream>

#define MAX_BUFFER_SIZE 256

/**
 * \ingroup hw
 */
namespace eps_driver {

// Constructor sets up the i2c bus, usleep callback and channel mux
EpsDriver::EpsDriver(const i2c::Device &i2c_dev, std::function<void(uint32_t)> usleep_cb)
  : i2c_dev_(i2c_dev)
  , usleep_cb_(usleep_cb)
  , hk_info_({
    { 0, "AGND1_V", 1.0, 0.0, },
    { 1, "SUPPLY_IN_V", ((160.0+20.0)/20.0*3.3/1023.0), 0.0, },
    { 2, "PAYLOAD_PWR3_I", (3.3/1023.0/100.0/0.01), 0, },
    { 3, "SUBSYS1_1_PWR_V", ((160.0+39.2)/39.2*3.3/1023.0), 0.0, },
    { 4, "SUBSYS1_2_PWR_V", ((160.0+39.2)/39.2*3.3/1023.0), 0.0, },
    { 5, "UNREG_V", ((160.0+39.2)/39.2*3.3/1023.0), 0.0, },
    { 6, "SYSTEM_I", (3.3/1023.0/100.0/0.001), 0, },
    { 7, "BAT4V_V", ((160.0+39.2)/39.2*3.3/1023.0), 0.0, },
    { 8, "BAT3V_V", ((160.0+39.2)/39.2*3.3/1023.0), 0.0, },
    { 9, "BAT2V_V", ((160.0+39.2)/39.2*3.3/1023.0), 0.0, },
    { 10, "BAT1V_V", ((160.0+39.2)/39.2*3.3/1023.0), 0.0, },
    { 11, "SUPPLY_I", ((27.0+39.2)/39.2*3.3/1023.0/100.0/0.01), 0, },
    { 12, "5VLIVE_V", ((39.2+39.2)/39.2*3.3/1023.0), 0.0, },
    { 13, "AGND2_V", 1.0, 0.0, },
    { 14, "FAN_PWR_I", (3.3/1023.0/100.0/0.03), 0, },
    { 15, "AUX_PWR_I", (3.3/1023.0/100.0/0.01), 0, },
    { 16, "PAYLOAD_PWR4_I", (3.3/1023.0/100.0/0.01), 0, },
    { 17, "PAYLOAD_PWR2_I", (3.3/1023.0/100.0/0.01), 0, },
    { 18, "PAYLOAD_PWR1_I", (3.3/1023.0/100.0/0.01), 0, },
    { 19, "5A_REG1_PWR_I", ((27.0+39.2)/39.2*3.3/1023.0/100.0/0.01), 0, },
    { 20, "MOTOR1_I", (3.3/1023.0/100.0/0.01), 0, },
    { 21, "SUBSYS2_PWR_V", ((160.0+39.2)/39.2*3.3/1023.0), 0.0, },
    { 22, "MOTOR2_I", (3.3/1023.0/100.0/0.01), 0, },
    { 23, "5A_REG2_PWR_I", ((27.0+39.2)/39.2*3.3/1023.0/100.0/0.01), 0, },
    { 24, "5A_REG3_PWR_I", ((27.0+39.2)/39.2*3.3/1023.0/100.0/0.01), 0, },
    { 25, "MAIN5_PWR_I", ((27.0+39.2)/39.2*3.3/1023.0/100.0/0.01), 0, },
    { 26, "AUO_PWR_I", (3.3/1023.0/100.0/0.03), 0, },
    { 27, "HLP_I", (3.3/1023.0/100.0/0.01), 0, },
    { 28, "USB_PWR_I", (3.3/1023.0/100.0/0.015), 0, },
    { 29, "LLP_I", (3.3/1023.0/100.0/0.03), 0, },
    { 30, "MLP_I", (3.3/1023.0/100.0/0.01), 0, },
    { 31, "ENET_PWR_I", (3.3/1023.0/100.0/0.05), 0, }
  })
  , power_chan_names_({
      // Subsystem power switch
      "LLP_EN",
      "MLP_EN",
      "HLP_EN",
      "USB_PWR_EN",
      "AUX_PWR_EN",
      "ENET_PWR_EN",
      "FAN_EN",
      "SPEAKER_EN",
      // Reset switch
      "PAYLOAD_EN1",
      "PAYLOAD_EN2",
      "PAYLOAD_EN3",
      "PAYLOAD_EN4",
      "MOTOR_EN1",
      "MOTOR_EN2",
      "** RESERVED **",
      "** RESERVED **",
      // LED1 switch
      "(LJ1) LED1",
      "(LJ1) LED2",
      "(LJ1) LED3",
      "(LJ1) LED4",
      "(LJ1) LED5",
      "(LJ1) LED6",
      "** RESERVED **",
      "** RESERVED **",
      // LED2, 3 switch
      "(LJ2/LJ3) LED1",
      "(LJ2/LJ3) LED2",
      "(LJ2/LJ3) LED3",
      "(LJ3) LED4",
      "(LJ2) LED4",
      "(LJ2) LED5",
      "** RESERVED **",
      "** RESERVED **"}) {}

// Destructor
EpsDriver::~EpsDriver(void) {}


std::vector<std::string>& EpsDriver::GetPowerChannelNames() {
  return power_chan_names_;
}

// Set the version string
bool EpsDriver::GetString(StringType type, std::string & data) {
  uint8_t cmd[4] = {type, 1, 0, 0};
  if (Write(cmd, 4) != 4)
    return false;
  // Based on the size of the returned result, change the data size
  uint8_t inbuf[256];
  uint16_t size = Read(inbuf);
  if (!size)  // We are expecting a result
    return false;
  // Copy result to the outut string
  data = std::string(reinterpret_cast<char*>(inbuf), size);
  // Return success
  return true;
}

// Reset the EPS
bool EpsDriver::Reset() {
  uint8_t cmd[4] = {I2C_CMD_REBOOT, 1, 0, 0};
  return (Write(cmd, 4) == 4);
}

// Set specific power channel on (not a mask, but an integer channel!)
bool EpsDriver::SetPayloadState(PayloadIndex payload, PowerState state) {
  ChannelIndex channel = CHANNEL_PAYLOAD_EN1;
  switch (payload) {
  case PAYLOAD_TOP_FRONT:     channel = CHANNEL_PAYLOAD_EN1;  break;
  case PAYLOAD_BOTTOM_FRONT:  channel = CHANNEL_PAYLOAD_EN2;  break;
  case PAYLOAD_TOP_AFT:       channel = CHANNEL_PAYLOAD_EN3;  break;
  case PAYLOAD_BOTTOM_AFT:    channel = CHANNEL_PAYLOAD_EN4;  break;
  default:
    return false;
  }
  switch (state) {
  case PERSIST:  return true;
  case ENABLED:  return SetPowerChannelState(channel, ENABLED);
  case DISABLED: return SetPowerChannelState(channel, DISABLED);
  default:
    break;
  }
  return false;
}

// Set specific power channel on (not a mask, but an integer channel!)
bool EpsDriver::SetAdvancedState(AdvancedIndex advanced, PowerState state) {
  ChannelIndex channel = CHANNEL_USB_PWR_EN;
  switch (advanced) {
  case ADVANCED_USB:          channel = CHANNEL_USB_PWR_EN;   break;
  case ADVANCED_AUX:          channel = CHANNEL_AUX_PWR_EN;   break;
  case ADVANCED_PMC1:         channel = CHANNEL_MOTOR_EN1;    break;
  case ADVANCED_PMC2:         channel = CHANNEL_MOTOR_EN2;    break;
  default:
    return false;
  }
  switch (state) {
  case PERSIST:  return true;
  case ENABLED:
    if (channel == CHANNEL_USB_PWR_EN || channel == CHANNEL_AUX_PWR_EN)
      return SetPowerChannelState(channel - 1, ENABLED);
    else if (channel == CHANNEL_MOTOR_EN1 || channel == CHANNEL_MOTOR_EN2)
      return SetPowerChannelState(channel, ENABLED);
  case DISABLED:
    if (channel == CHANNEL_USB_PWR_EN || channel == CHANNEL_AUX_PWR_EN)
      return SetPowerChannelState(channel - 1, DISABLED);
    else if (channel == CHANNEL_MOTOR_EN1 || channel == CHANNEL_MOTOR_EN2)
      return SetPowerChannelState(channel, DISABLED);
  default:
    break;
  }
  return false;
}

// Set specific power channel on (not a mask, but an integer channel!)
bool EpsDriver::SetLedState(LedIndex led, PowerState state) {
  ChannelIndex channel = CHANNEL_STATUS_LED1;
  switch (led) {
  case LED_STATUS_1:  channel = CHANNEL_STATUS_LED1;  break;
  case LED_STATUS_2:  channel = CHANNEL_STATUS_LED2;  break;
  case LED_STATUS_3:  channel = CHANNEL_STATUS_LED3;  break;
  case LED_STATUS_4:  channel = CHANNEL_STATUS_LED4;  break;
  case LED_STATUS_5:  channel = CHANNEL_STATUS_LED5;  break;
  case LED_STATUS_6:  channel = CHANNEL_STATUS_LED6;  break;
  case LED_STREAM:    channel = CHANNEL_STREAM_LED;   break;
  case LED_CAMERA:    channel = CHANNEL_CAMERA_LED;   break;
  case LED_MIC:       channel = CHANNEL_MIC_LED;      break;
  default:
    return false;
  }
  switch (state) {
  case PERSIST:  return true;
  case ENABLED:  return SetPowerChannelState(channel, ENABLED);
  case DISABLED: return SetPowerChannelState(channel, DISABLED);
  default:
    break;
  }
  return false;
}

// Ring the buzzer
bool EpsDriver::RingBuzzer(uint16_t freq, uint8_t secs) {
  if (freq < EPS_MIN_BUZZER_FREQUENCY || freq > EPS_MAX_BUZZER_FREQUENCY)
    return false;
  if (secs < EPS_MIN_BUZZER_DURATION || secs > EPS_MAX_BUZZER_DURATION)
    return false;

  uint8_t cmd[7] = {  I2C_CMD_RING_BUZZER,  // Opcode
                      4, 0,  // Length (2 Bytes), LSB
                      (uint8_t)((freq >> 0) & 0x00FF),
                      (uint8_t)((freq >> 8) & 0x00FF),
                      secs,
                      0
                    };
  return (Write(cmd, 7) == 7);
}

// Enable the PMCs
bool EpsDriver::EnablePMCs(bool enable) {
  uint8_t cmd[8] = {
    (uint8_t)(enable ? I2C_CMD_SW_ON : I2C_CMD_SW_OFF),
    5, 0, 0x00, 0x30, 0x00, 0x00, 0};
  return (Write(cmd, 8) == 8);
}

bool EpsDriver::Undock(void) {
  uint8_t cmd[4] = { I2C_CMD_UNDOCK, 1, 0, 0 };
  return (Write(cmd, 4) == 4);
}

bool EpsDriver::GetConnectionState(ConnectionState &state) {
  uint8_t cmd[4] = {I2C_CMD_GET_CONNECTION_STATE, 1, 0, 0};
  if (Write(cmd, 4) != 4)
    return false;
  // Based on the size of the returned result, change the data size
  uint8_t inbuf[16];
  uint16_t size = Read(inbuf);

  if (size != 1)
    return false;

  switch (inbuf[0]) {
  case CONN_DISCONNECTED:
    state = CONN_DISCONNECTED;
    return true;
  case CONN_CONNECTING:
    state = CONN_CONNECTING;
    return true;
  case CONN_CONNECTED:
    state = CONN_CONNECTED;
    return true;
  default:
    std::cerr << "Undefined State" << std::endl;
    return false;
  }

  return true;
}

bool EpsDriver::ClearTerminateEvent(void) {
  uint8_t cmd[4] = { I2C_CMD_CLR_TERMINATE_EVT, 1, 0, 0 };
  return (Write(cmd, 4) == 4);
}

// Read the analog temperature sensors
bool EpsDriver::ReadTemperatureSensors(std::vector<double>& data) {
  uint8_t cmd[4] = {I2C_CMD_GET_DIGITAL_TEMPS, 1, 0, 0};
  if (Write(cmd, 4) != 4)
    return false;
  // Based on the size of the returned result, change the data size
  uint8_t inbuf[256];
  uint16_t size = Read(inbuf);
  data.resize(size / 3);
  // Extract each value one by one
  for (size_t i = 0; i < data.size(); i++) {
    uint8_t addr = inbuf[i * 3];
    int16_t temp = (inbuf[i * 3 + 1] & 0x00FF) | (inbuf[i * 3 + 2]) << 8;
    data[i] = static_cast<double>(temp) / 100.0;

    // Debug
    printf("Temp @ 0x%02X: %f\n", addr, data[i]);
  }
  // Return success
  return true;
}

// Set all power switches on
bool EpsDriver::SetAllPowerChannelState(PowerState state) {
  uint8_t cmd[8] = {state, 5, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

  if (state == ENABLED)
    cmd[0] = I2C_CMD_SW_ON;
  else if (state == DISABLED)
    cmd[0] = I2C_CMD_SW_OFF;
  else
    return false;

  return (Write(cmd, 8) == 8);
}

bool EpsDriver::GetAllPowerChannelState(std::vector<PowerState> &states) {
  uint8_t cmd[4] = {I2C_CMD_GET_SW_STATES, 0x01, 0x00, 0x00};

  if (Write(cmd, 4) != 4)
    return false;

  uint8_t inbuf[8];
  if (Read(inbuf) != 4)
    return false;

  states.clear();

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 8; j++) {
      if ((inbuf[i] & 0x01) == 0x00)
        states.push_back(DISABLED);
      else
        states.push_back(ENABLED);

      inbuf[i] >>= 1;
    }
  }

  return true;
}


// Set specific power channel on (not a mask, but an integer channel!)
bool EpsDriver::SetPowerChannelState(uint8_t channel, PowerState state) {
  // Only accept valid channels
  if (channel >= NUM_CHANNELS)
    return false;

  uint8_t cmd[8] = {state, 5, 0,
    0, 0, 0, 0,
    0
  };

  if (state == ENABLED)
    cmd[0] = I2C_CMD_SW_ON;
  else if (state == DISABLED)
    cmd[0] = I2C_CMD_SW_OFF;
  else
    return false;

  // Set the power channel
  uint32_t mask = (0x1 << channel);
  cmd[3] = (mask)       & 0xFF;
  cmd[4] = (mask >> 8)  & 0xFF;
  cmd[5] = (mask >> 16) & 0xFF;
  cmd[6] = (mask >> 24) & 0xFF;

  return (Write(cmd, 8) == 8);
}

// Get the battery status for a particular channel
bool EpsDriver::GetBatteryStatus(BatteryIndex battery, BatteryStatus &data) {
  // Only accept valid batteries
  switch (battery) {
  case BATTERY_TOP_LEFT:
  case BATTERY_BOTTOM_LEFT:
  case BATTERY_TOP_RIGHT:
  case BATTERY_BOTTOM_RIGHT:
    break;
  default:
    return false;
  }
  uint8_t cmd[5] = {I2C_CMD_GET_BATTERY_STATUS, 2, 0, battery, 0};
  if (Write(cmd, 5) != 5)
    return false;
  // Extract a response from the i2c bus
  uint8_t inbuf[256];
  uint16_t size = Read(inbuf, 28);
  if (size != 28)  // Avoid a buffer overrun
    return false;

  // Populate the battery data
  data.chan = (inbuf[0]);
  data.present = (inbuf[1] == 0x00 ? false : true);
  data.voltage = (inbuf[2] & 0x00FF) | (inbuf[3]  << 8);
  data.current = (inbuf[4] & 0x00FF) | (inbuf[5] << 8);
  data.charge = (inbuf[6] & 0x00FF) | (inbuf[7]  << 8);
  data.capacity = (inbuf[8] & 0x00FF) | (inbuf[9]  << 8);
  data.design_capacity = (inbuf[10] & 0x00FF) | (inbuf[11]  << 8);
  data.percentage = (inbuf[12] & 0x00FF) | (inbuf[13]  << 8);
  for (int i = 0; i < 4; i++)
    data.cell_voltage[i] = (inbuf[14 + (i * 2)] & 0x00FF)
                          | (inbuf[14 + (i * 2) + 1]  << 8);
  data.status = (inbuf[22] & 0x00FF) | (inbuf[23]  << 8);
  data.temperature = (inbuf[24] & 0x00FF) | (inbuf[25] << 8);
  data.serial_number = (inbuf[26] & 0x00FF) | (inbuf[27] << 8);

  return true;
}

// Read housekeeping data for each power channel
bool EpsDriver::ReadHousekeeping(std::vector<HousekeepingInfo> &data) {
  uint8_t cmd[4] = {I2C_CMD_GET_HK, 1, 0, 0};
  if (Write(cmd, 4) != 4)
    return false;
  // Extract a response from the i2c bus
  uint8_t inbuf[256];
  uint16_t size = Read(inbuf);
  uint16_t num_chans = size / 2;
  // Avoid a buffer overrun
  if (num_chans != EPS_NUM_HOUSEKEEPING)
    return false;
  // Extract the data for each channel
  data.resize(num_chans);
  for (size_t i = 0; i < num_chans; i++) {
    uint16_t raw =
      (uint16_t)((inbuf[i * 2] & 0x00FF) | (inbuf[(i * 2) + 1] << 8));
    hk_info_[i].value = static_cast<double>(raw) / 1000.0;

    data[i] = hk_info_[i];
  }
  // Success
  return true;
}

// Write new firmware to the EPS
bool EpsDriver::WriteFirmware(std::string const& hexfile) {
  // Firs, try and open a binary file stream to the hexfile
  std::ifstream file(hexfile.c_str(), std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Could not open the local firmware hex file";
    return false;
  }
  // Try and open the firmware file remotely
  if (!ChangeFirmwareFileState(OPEN)) {
    std::cerr << "Could not open the remote firmware file";
    return false;
  }
  // Keep iterating in blocks
  uint8_t cmd[256] = {4, 0, 0, };
  uint8_t ackbuf[3], retries = 0;
  bool ack_success = 0;
  while (!file.eof()) {
    // Read a block of data into the command
    file.read(reinterpret_cast<char*>(cmd) + 3, EPS_FIRMWARE_BLOCK_SIZE_BYTES);
    // Write the number of bits read into byte 2
    cmd[1] = file.gcount() + 1;  // Include the checksum byte :)
    // Write and check acknowledgment was received
    retries = EPS_FIRMWARE_NUM_FILE_RETRIES;
    ack_success = false;
    do {
      if (Write(cmd, cmd[1] + 4) != cmd[1] + 4)
        continue;
      if (Read(ackbuf) != 3)  // Avoid a buffer overrun
        continue;
      if (ackbuf[0] == 0xAA && ackbuf[1] == 0 && ackbuf[2] == 0)
        ack_success = true;
    } while (retries-- > 0 && !ack_success);
    // Check that we actually received the ack
    if (!ack_success) {
      std::cerr << "One of the firmware blocks could not be written";
      return false;
    }
  }
  // Close the file handle
  file.close();
  // Try and open the firmware file remotely
  if (!ChangeFirmwareFileState(CLOSE)) {
    std::cerr << "Could not close the remote firmware file";
    return false;
  }
  // Try and open the firmware file remotely
  if (!ChangeFirmwareFileState(READY)) {
    std::cerr << "Could not tag the remote firmware as ready";
    return false;
  }
  // Sleep for 5ms
  usleep_cb_(5000);
  // Reset the EPS
  Reset();
  // Success
  return true;
}

////////////////////////////////////////////////
// Internal read, write and checksum routines //
////////////////////////////////////////////////

// Close the firware file
bool EpsDriver::ChangeFirmwareFileState(FirmwareFileState state) {
  uint8_t cmd[4] = {state, 1, 0, 0};
  uint8_t ackbuf[256];
  uint8_t retries = EPS_FIRMWARE_NUM_FILE_RETRIES;
  do {
    // Write the command
    if (Write(cmd, 4) != 4)
      continue;
    // Read the acknowledgment
    if (Read(ackbuf) != 3)  // Avoid a buffer overrun
      continue;
    // Check the contents of the acknowledgment
    if (ackbuf[0] == 0xAA && ackbuf[1] == 0 && ackbuf[2] == 0)
      return true;
  // Keep looping indefinitely
  } while (retries-- > 0);
  // We should never reach this point
  return false;
}

// Read len bytes from the I2C bus.
uint16_t EpsDriver::Read(uint8_t *buff, uint16_t len) {
  i2c::Error err;
  uint8_t tmp[MAX_BUFFER_SIZE];
  // Op code, length L, length H, data (len), checksum
  if (i2c_dev_.Read(tmp, len + 4, &err) != (len + 4)) {
    std::cerr << "Failed to read data over I2C: ";
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }

  uint16_t size = (tmp[1] & 0xFF) | ((tmp[2] & 0xFF) << 8);

  if (size != len + 1) {
    std::cerr << "I2C packet size mismatched: " << std::endl;
    std::cerr << size << " != " << len + 1 << std::endl;
    return 0;
  }

  // Check that the checksum matches
  if (ComputeChecksum(tmp, size + 3) != 0x0) {
    std::cerr << "Checksums did not match: ";
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Copy the data into the buffer
  memcpy(buff, tmp + 3, size - 1);
  // Return the payload + header size (NO CHECKSUM)
  return size - 1;
}

// Read an i2c frame. This removed the header and the checksum, and the size returned
// is ONLY of the payload. The buff argument contins ONLY the payload.
uint16_t EpsDriver::Read(uint8_t *buff) {
  i2c::Error err;
  uint8_t tmp[MAX_BUFFER_SIZE];
  // Read the size of the output buffer
  if (i2c_dev_.Read(tmp, 3, &err) != 3) {
    std::cerr << "Failed to read the output buffer size over I2C: ";
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Calculate the size of the output buffer
  uint16_t size = (tmp[1] & 0xFF) | ((tmp[2] & 0xFF) << 8);
  // Read the size of the output buffer why + 3 ?
  if (i2c_dev_.Read(tmp, size + 3, &err) != size + 3) {
    std::cerr << "Failed to read data over I2C: ";
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Check that the checksum matches
  if (ComputeChecksum(tmp, size + 3) != 0x0) {
    std::cerr << "Checksums did not match: ";
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Copy the data into the buffer
  memcpy(buff, tmp + 3, size - 1);
  // Return the payload + header size (NO CHECKSUM)
  return size - 1;
}

// Write an i2c frame + 1 byte checksum at end
uint16_t EpsDriver::Write(uint8_t *buff, uint16_t len) {
  buff[len-1] = ComputeChecksum(buff, len-1);
  i2c::Error err;
  int size = i2c_dev_.Write(buff, len, &err);
  if (size < 0) {
    std::cerr << "Failed to write " << len << " Bytes over I2C: ";
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  return static_cast<uint16_t>(size);
  // Sleep for 50ms
  usleep_cb_(50000);
}

// Checksum a buffer
uint8_t EpsDriver::ComputeChecksum(uint8_t *buf, size_t size) {
  uint8_t checksum = 0xFF;
  for (size_t i = 0; i < size; i++)
    checksum ^= buf[i];
  return checksum;
}

}  // namespace eps_driver
