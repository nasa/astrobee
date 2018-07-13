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
#include <iomanip>
#include <fstream>
#include <sstream>

/**
 * \ingroup hw
 */
namespace eps_driver {

EPS::EPS(const i2c::Device &i2c_dev, std::function<void(uint32_t)> usleep_cb)
  : i2c_dev_(i2c_dev), usleep_cb_(usleep_cb) {}

EPS::~EPS(void) {}

bool EPS::RingBuzzer(uint16_t freq, uint8_t secs) {
  if (freq < 1000 || freq > 2000 || secs > 10) return false;
  uint8_t cmd[7] = {
    I2C_CMD_RING_BUZZER,                // Command
      4, 0,                             // Length (2 Bytes), LSB
      (uint8_t)((freq >> 0) & 0x00FF),  // Low freq bits
      (uint8_t)((freq >> 8) & 0x00FF),  // High freq bits
      secs,                             // Duration
      0                                 // Checksum
    };
  return (Write(cmd, 7) == 7);
}

bool EPS::EnterBootloader(void) {
  uint8_t cmd[4] = { I2C_CMD_ENTER_BOOTLOADER, 1, 0, 0 };
  return (Write(cmd, 4) == 4);
}

bool EPS::ClearFaults(void) {
  uint8_t cmd[4] = { I2C_CMD_CLR_HW_EXCEPTIONS, 1, 0, 0 };
  return (Write(cmd, 4) == 4);
}

bool EPS::Unterminate(void) {
  uint8_t cmd[4] = { I2C_CMD_CLR_TERMINATE_EVT, 1, 0, 0 };
  return (Write(cmd, 4) == 4);
}

bool EPS::Undock(void) {
  uint8_t cmd[4] = {I2C_CMD_UNDOCK, 1, 0, 0 };
  return (Write(cmd, 4) == 4);
}

bool EPS::Reboot(void) {
  uint8_t cmd[4] = {I2C_CMD_REBOOT, 1, 0, 0};
  return (Write(cmd, 4) == 4);
}

bool EPS::GetStrings(uint32_t mask, std::map<String, std::string> & data) {
  uint8_t cmd[4] = {0, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  for (uint32_t i = 0; i < NUM_STRINGS; i++) {
    if ((mask & (1 << i)) == 0) continue;
    switch (i) {
    case STRING_SW_VERSION: cmd[0] = I2C_CMD_GET_SW_VERSION;          break;
    case STRING_BUILD:      cmd[0] = I2C_CMD_GET_BUILD_TIME;          break;
    case STRING_SERIAL:     cmd[0] = I2C_CMD_GET_SERIAL_NUMBER;       break;
    default:
      return false;
    }
    if (Write(cmd, 4) != 4)
      return false;
    size_t size = Read(buf);
    if (!size || size > I2C_BUF_MAX_LEN)
      return false;
    if (i == STRING_SERIAL) {
      data[static_cast<String>(i)] = SerialToString(buf);
    } else {
      data[static_cast<String>(i)] =
        std::string(reinterpret_cast<char*>(buf), size);
    }
  }
  return true;
}

bool EPS::GetStates(uint32_t mask, std::map<State, uint8_t> & data) {
  uint8_t cmd[4] = {0, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  for (uint32_t i = 0; i < NUM_STATES; i++) {
    if ((mask & (1 << i)) == 0) continue;
    switch (i) {
     case STATE_POWER: cmd[0] = I2C_CMD_GET_EPS_POWER_MODE;   break;
     case STATE_DOCK:  cmd[0] = I2C_CMD_GET_CONNECTION_STATE; break;
     default:
      return false;
    }
    if (Write(cmd, 4) != 4)
      return false;
    if (Read(buf) != 1)
      return false;
    data[static_cast<State>(i)] = buf[0];
  }
  return true;
}

bool EPS::GetChannels(uint32_t mask, std::map<Channel, bool> & data) {
  uint8_t cmd[4] = {I2C_CMD_GET_SW_STATES, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Write(cmd, 4) != 4)
    return false;
  if (Read(buf) != 4)
    return false;
  uint32_t values = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
  for (uint32_t i = 0; i < NUM_CHANNELS; i++)
    if ((mask & (1 << i)))
      data[static_cast<Channel>(i)] = (((values >> i) & 0x1) == 0x1);
  return true;
}

bool EPS::GetChargers(uint32_t mask, std::map<Charger, bool> & data) {
  uint8_t cmd[4] = {I2C_CMD_GET_CHARGE_STATE, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Write(cmd, 4) != 4)
    return false;
  if (Read(buf) != 1)
    return false;
  uint8_t values = buf[0];
  for (uint32_t i = 0; i < NUM_CHARGERS; i++)
    if ((mask & (1 << i)))
      data[static_cast<Charger>(i)] = (((values >> i) & 0x1) == 0x1);
  return true;
}

bool EPS::GetFaults(uint32_t mask, std::map<Fault, bool> & data) {
  uint8_t cmd[4] = {I2C_CMD_GET_HW_EXCEPTIONS, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Write(cmd, 4) != 4)
    return false;
  if (Read(buf) != 4)
    return false;
  uint32_t values = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
  for (uint32_t i = 0; i < NUM_FAULTS; i++)
    if (mask & (1 << i))
      data[static_cast<Fault>(i)] = ((values >> i) & 0x1);
  return true;
}

bool EPS::GetBatteries(uint32_t mask, std::map<Battery, BatteryInfo> & data) {
  uint8_t cmd[5] = {I2C_CMD_GET_BATTERY_STATUS, 2, 0, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  for (uint32_t i = 0; i < NUM_BATTERIES; i++) {
    if ((mask & (1 << i)) == 0) continue;
    cmd[3] = i;
    if (Write(cmd, 5) != 5)
      return false;
    if (Read(buf, 28) != 28)
      return false;
    BatteryInfo & info = data[static_cast<Battery>(i)];
    info.chan = (buf[0]);
    info.present = (buf[1] == 0x00 ? false : true);
    info.voltage = (buf[2] & 0x00FF) | (buf[3]  << 8);
    info.current = (buf[4] & 0x00FF) | (buf[5] << 8);
    info.remaining = (buf[6] & 0x00FF) | (buf[7]  << 8);
    info.full = (buf[8] & 0x00FF) | (buf[9]  << 8);
    info.design= (buf[10] & 0x00FF) | (buf[11]  << 8);
    info.percentage = (buf[12] & 0x00FF) | (buf[13]  << 8);
    for (int i = 0; i < 4; i++)
      info.cell[i] = (buf[14 + (i * 2)] & 0x00FF)
                            | (buf[14 + (i * 2) + 1]  << 8);
    info.status = (buf[22] & 0x00FF) | (buf[23]  << 8);
    info.temperature = (buf[24] & 0x00FF) | (buf[25] << 8);
    info.serial = (buf[26] & 0x00FF) | (buf[27] << 8);
  }
  return true;
}

bool EPS::GetHousekeeping(uint32_t mask, std::map<Housekeeping, double> & data) {
  uint8_t cmd[4] = {I2C_CMD_GET_HK, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Write(cmd, 4) != 4)
    return false;
  if (Read(buf, 64) != 64)
    return false;
  for (uint32_t i = 0; i < NUM_HOUSEKEEPING; i++) {
    if ((mask & (1 << i)) == 0) continue;
    uint16_t raw = (uint16_t)((buf[(i * 2) + 0] & 0x00FF)
                 | (uint16_t)((buf[(i * 2) + 1] << 8)));
    data[static_cast<Housekeeping>(i)] = static_cast<double>(raw) / 1000.0;
  }
  return true;
}

bool EPS::GetTemps(uint32_t mask, std::map<Temp, TempInfo> & data) {
  uint8_t cmd[4] = {I2C_CMD_GET_DIGITAL_TEMPS, 1, 0, 0};
  if (Write(cmd, 4) != 4)
    return false;
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Read(buf) != 9)
    return false;
  for (uint32_t i = 0; i < NUM_TEMPERATURES; i++) {
    if ((mask & (1 << i)) == 0) continue;
    int16_t temp = (buf[i * 3 + 1] & 0x00FF) | (buf[i * 3 + 2]) << 8;
    TempInfo & info = data[static_cast<Temp>(i)];
    info.addr = buf[i * 3];
    info.temp = static_cast<double>(temp) / 100.0;
  }
  return true;
}

bool EPS::SetLeds(uint32_t mask, LedMode const value) {
  uint32_t chanmask = 0x0;
  if (mask & (1 << LED_S2A)) chanmask |= (1 << CHANNEL_FWD_LED_4);
  if (mask & (1 << LED_S1A)) chanmask |= (1 << CHANNEL_FWD_LED_5);
  if (mask & (1 << LED_S2B)) chanmask |= (1 << CHANNEL_FWD_LED_6);
  if (mask & (1 << LED_S1B)) chanmask |= (1 << CHANNEL_FWD_LED_7);
  if (mask & (1 << LED_S2C)) chanmask |= (1 << CHANNEL_FWD_LED_8);
  if (mask & (1 << LED_S1C)) chanmask |= (1 << CHANNEL_FWD_LED_9);
  if (mask & (1 << LED_STR)) chanmask |= (1 << CHANNEL_FWD_LED_1);
  if (mask & (1 << LED_CAM)) chanmask |= (1 << CHANNEL_FWD_LED_2);
  if (mask & (1 << LED_MIC)) chanmask |= (1 << CHANNEL_FWD_LED_3);
  uint8_t cmd[9] = {
    I2C_CMD_SET_LED_MODES,                          // Command
    6, 0,                                           // Length
    0,                                              // Channel mask
    0,                                              // Channel mask
    static_cast<uint8_t>((chanmask >> 16) & 0xff),  // Channel mask
    static_cast<uint8_t>((chanmask >> 24) & 0xff),  // Channel mask
    static_cast<uint8_t>(value),                    // New value
    0};                                             // Checksum
  return (Write(cmd, 9) == 9);
}

bool EPS::SetChargers(uint32_t mask, bool const value) {
  uint8_t cmd[6] = {
    I2C_CMD_SET_CHARGE_STATE,                       // Command
    3, 0,                                           // Length
    static_cast<uint8_t>(value ? 1 : 0),            // Enable or disable?
    static_cast<uint8_t>(mask & 0xf),               // Mask
    0};                                             // Checksum
  return (Write(cmd, 6) == 6);
}

bool EPS::SetChannels(uint32_t mask, bool const value) {
  uint8_t cmd[8] = {
    value ? I2C_CMD_SW_ON : I2C_CMD_SW_OFF,         // Command
    5, 0,                                           // Length
    static_cast<uint8_t>(mask & 0xff),              // Mask LSB
    static_cast<uint8_t>((mask >> 8)  & 0xff),      // Mask
    static_cast<uint8_t>((mask >> 16) & 0xff),      // Mask
    static_cast<uint8_t>((mask >> 24) & 0xff),      // Mask MSB
    0};                                             // Checksum
  return (Write(cmd, 8) == 8);
}

bool EPS::SetPowerState(PowerStateValue const value) {
  uint8_t cmd[9] = {
    I2C_CMD_SET_EPS_POWER_MODE,                     // Command
    2, 0,                                           // Length
    static_cast<uint8_t>(value),                    // New power state
    0};                                             // Checksum
  return (Write(cmd, 5) == 5);
}

std::string EPS::SerialToString(uint8_t serial[6]) {
  std::ostringstream oss;
  oss << "0x";
  for (uint8_t k = 0; k < 6; k++)
    oss << std::setfill('0') << std::setw(2) << std::hex << std::uppercase
        << static_cast<int>(serial[k]);
  return oss.str();
}

// PRIVATE FUNCTIONS

uint16_t EPS::Read(uint8_t *buff, uint16_t len) {
  i2c::Error err;
  uint8_t tmp[I2C_BUF_MAX_LEN];
  // Op code, length L, length H, data (len), checksum
  if (i2c_dev_.Read(tmp, len + 4, &err) != (len + 4)) {
    std::cerr << "Failed to read data over I2C: " << std::endl;
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
    std::cerr << "Checksums did not match: " << std::endl;
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Copy the data into the buffer
  memcpy(buff, tmp + 3, size - 1);
  // Return the payload + header size (NO CHECKSUM)
  return size - 1;
}

uint16_t EPS::Read(uint8_t *buff) {
  i2c::Error err;
  uint8_t tmp[I2C_BUF_MAX_LEN];
  // Read the size of the output buffer
  if (i2c_dev_.Read(tmp, 3, &err) != 3) {
    std::cerr << "Failed to read the output buffer size over I2C: ";
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Calculate the size of the output buffer
  uint16_t size = (tmp[1] & 0xFF) | ((tmp[2] & 0xFF) << 8);
  if (!size) {
    std::cerr << "Didn't read any bytes" << std::endl;
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Read the size of the output buffer why + 3 ?
  if (i2c_dev_.Read(tmp, size + 3, &err) != size + 3) {
    std::cerr << "Failed to read data over I2C: " << std::endl;
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Check that the checksum matches
  if (ComputeChecksum(tmp, size + 3) != 0x0) {
    std::cerr << "Checksums did not match: " << std::endl;
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  // Copy the data into the buffer
  memcpy(buff, tmp + 3, size - 1);
  // Return the payload + header size (NO CHECKSUM)
  return size - 1;
}

uint16_t EPS::Write(uint8_t *buff, uint16_t len) {
  buff[len-1] = ComputeChecksum(buff, len-1);
  i2c::Error err;
  int size = i2c_dev_.Write(buff, len, &err);
  if (size < 0) {
    std::cerr << "Failed to write " << len << " Bytes over I2C: " << std::endl;
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  return static_cast<uint16_t>(size);
  // Sleep for 50ms
  usleep_cb_(50000);
}

uint8_t EPS::ComputeChecksum(uint8_t *buf, size_t size) {
  uint8_t checksum = 0xFF;
  for (size_t i = 0; i < size; i++)
    checksum ^= buf[i];
  return checksum;
}


}  // namespace eps_driver
