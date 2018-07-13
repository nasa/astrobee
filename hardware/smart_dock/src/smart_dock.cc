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

#include <smart_dock/smart_dock.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

/**
 * \ingroup hw
 */
namespace smart_dock {

SmartDock::SmartDock(const i2c::Device &i2c_dev) : i2c_dev_(i2c_dev) {}

SmartDock::~SmartDock(void) {}

// PUBLIC

// CHECKED
bool SmartDock::Reboot(void) {
  uint8_t cmd[4] = {I2C_CMD_REBOOT, 1, 0, 0};
  return (Write(cmd, 4) == 4);
}

// CHECKED
bool SmartDock::ClearFaults(void) {
  uint8_t cmd[4] = {I2C_CMD_CLR_HW_EXCEPTIONS, 1, 0, 0};
  return (Write(cmd, 4) == 4);
}

// CHECKED
bool SmartDock::SendBerthCommand(uint32_t mask, BerthCommand const value) {
  // First, get information about the berths
  std::map<Berth, BerthState> data;
  if (!GetBerthStates(mask, data)) return false;
  // Now, iterate over each berth in the mask
  bool success = true;
  for (uint8_t i = 0; i < NUM_BERTHS; i++) {
    if ((mask & (1 << i)) == 0) continue;
    // We can't send a command to an empty berth, as there is no freeflyer
    if (data[static_cast<Berth>(i)].dock_state != EPS::DOCK_CONNECTED) {
      success &= false;
      continue;
    }
    // If we get there then we do in fact have an EPS attached to the berth
    switch (value) {
      case COMMAND_SET_POWER_MODE_HIBERNATE:
      case COMMAND_SET_POWER_MODE_AWAKE_NOMINAL:
      case COMMAND_SET_POWER_MODE_AWAKE_SAFE:
      case COMMAND_SET_POWER_MODE_CRITICAL_FAULT: {
        // Get the mode assignement
        uint8_t mode = EPS::POWER_STATE_UNKNOWN;
        if (value == COMMAND_SET_POWER_MODE_HIBERNATE)
          mode = EPS::POWER_STATE_HIBERNATE;
        else if (value == COMMAND_SET_POWER_MODE_AWAKE_NOMINAL)
          mode = EPS::POWER_STATE_AWAKE_NOMINAL;
        else if (value == COMMAND_SET_POWER_MODE_AWAKE_SAFE)
          mode = EPS::POWER_STATE_AWAKE_SAFE;
        else if (value == COMMAND_SET_POWER_MODE_CRITICAL_FAULT)
          mode = EPS::POWER_STATE_CRITICAL_FAULT;
        // Send the i2c transaction
        uint8_t cmd[7] = {I2C_CMD_SET_EPS_CMD,  // SET
                          4,
                          0,                           // Size
                          i,                           // Berth
                          I2C_CMD_SET_EPS_POWER_MODE,  // Command
                          mode,                        // Power mode
                          0};                          // CRC
        success &= (Write(cmd, 7) == 7);
        break;
      }
      // Unterminate
      case COMMAND_CLEAR_TERMINATE: {
        uint8_t cmd[6] = {I2C_CMD_SET_EPS_CMD,  // SET
                          3,
                          0,                          // Size
                          i,                          // Berth
                          I2C_CMD_CLR_TERMINATE_EVT,  // Command
                          0};                         // CRC
        success &= (Write(cmd, 6) == 6);
        break;
      }
      // Clear faults
      case COMMAND_CLEAR_FAULTS: {
        uint8_t cmd[6] = {I2C_CMD_SET_EPS_CMD,  // SET
                          3,
                          0,                          // Size
                          i,                          // Berth
                          I2C_CMD_CLR_HW_EXCEPTIONS,  // Command
                          0};                         // CRC
        success &= (Write(cmd, 6) == 6);
        break;
      }
      // Set select channel states
      case COMMAND_ENABLE_ALL_PAYLOADS:
      case COMMAND_DISABLE_ALL_PAYLOADS:
      case COMMAND_ENABLE_ALL_PMCS:
      case COMMAND_DISABLE_ALL_PMCS: {
        // Work out if this is a turn on or turn off command
        uint8_t command = I2C_CMD_SW_ON;
        if (value == COMMAND_DISABLE_ALL_PMCS ||
            value == COMMAND_DISABLE_ALL_PAYLOADS)
          command = I2C_CMD_SW_OFF;
        // Work out if this is a payload or PMC command
        uint32_t mask =
            (1 << EPS::CHANNEL_PAYLOAD_EN1) | (1 << EPS::CHANNEL_PAYLOAD_EN2) |
            (1 << EPS::CHANNEL_PAYLOAD_EN3) | (1 << EPS::CHANNEL_PAYLOAD_EN4);
        if (value == COMMAND_ENABLE_ALL_PMCS ||
            value == COMMAND_DISABLE_ALL_PMCS)
          mask = (1 << EPS::CHANNEL_MOTOR_EN1) | (1 << EPS::CHANNEL_MOTOR_EN2);
        // Creat the irc command
        uint8_t cmd[10] = {
            I2C_CMD_SET_EPS_CMD,  // SET
            7,
            0,                                          // Size
            i,                                          // Berth
            command,                                    // Command
            static_cast<uint8_t>((mask)&0xff),          // Mask LSB
            static_cast<uint8_t>((mask >> 8) & 0xff),   // Mask
            static_cast<uint8_t>((mask >> 16) & 0xff),  // Mask
            static_cast<uint8_t>((mask >> 24) & 0xff),  // Mask MSB
            0};                                         // CRC
        success &= (Write(cmd, 10) == 10);
        break;
      }
      // Reboot
      case COMMAND_REBOOT: {
        uint8_t cmd[6] = {I2C_CMD_SET_EPS_CMD,  // SET
                          3,
                          0,               // Size
                          i,               // berth
                          I2C_CMD_REBOOT,  // Command
                          0};              // CRC
        success &= (Write(cmd, 6) == 6);
        break;
      }
      default:
        success &= false;
        break;
    }
  }
  // Did ALL commands succeed?
  return success;
}

// CHECKED
bool SmartDock::GetStrings(uint32_t mask, std::map<String, std::string> &data) {
  uint8_t cmd[4] = {0, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  for (uint32_t i = 0; i < NUM_STRINGS; i++) {
    if ((mask & (1 << i)) == 0) continue;
    switch (i) {
      case STRING_SW_VERSION:
        cmd[0] = I2C_CMD_GET_SW_VERSION;
        break;
      case STRING_BUILD:
        cmd[0] = I2C_CMD_GET_BUILD_TIME;
        break;
      case STRING_SERIAL:
        cmd[0] = I2C_CMD_GET_SERIAL_NUMBER;
        break;
      default:
        return false;
    }
    if (Write(cmd, 4) != 4) return false;
    size_t size = Read(buf);
    if (!size || size > I2C_BUF_MAX_LEN) return false;
    if (i == STRING_SERIAL) {
      data[static_cast<String>(i)] = EPS::SerialToString(buf);
    } else {
      data[static_cast<String>(i)] =
          std::string(reinterpret_cast<char *>(buf), size);
    }
  }
  return true;
}

// CHECKED
bool SmartDock::GetChannels(uint32_t mask, std::map<Channel, bool> &data) {
  uint8_t cmd[4] = {I2C_CMD_GET_SW_STATES, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Write(cmd, 4) != 4) return false;
  if (Read(buf) != 4) return false;
  uint32_t values = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
  for (uint32_t i = 0; i < NUM_CHANNELS; i++)
    if ((mask & (1 << i)))
      data[static_cast<Channel>(i)] = (((values >> i) & 0x1) == 0x1);
  return true;
}

// CHECKED
bool SmartDock::GetFaults(uint32_t mask, std::map<Fault, bool> &data) {
  uint8_t cmd[4] = {I2C_CMD_GET_HW_EXCEPTIONS, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Write(cmd, 4) != 4) return false;
  if (Read(buf) != 4) return false;
  uint32_t values = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
  for (uint32_t i = 0; i < NUM_FAULTS; i++)
    if (mask & (1 << i)) data[static_cast<Fault>(i)] = ((values >> i) & 0x1);
  return true;
}

// CHECKED
bool SmartDock::GetBerthStates(uint32_t mask,
                               std::map<Berth, BerthState> &data) {
  uint8_t cmd[5] = {I2C_CMD_GET_CONN_STATE, 2, 0, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  for (uint32_t i = 0; i < NUM_BERTHS; i++) {
    if ((mask & (1 << i)) == 0) continue;
    cmd[3] = i;
    if (Write(cmd, 5) != 5) return false;
    if (Read(buf, 131) != 131) return false;
    int len = 0;
    BerthState &state = data[static_cast<Berth>(i)];
    state.dock_state = buf[len++];  // loopback state debounced
    for (size_t j = 0; j < 6; j++)
      state.serial[5 - j] = buf[len++];
    state.power_state = buf[len++];
    state.terminate = buf[len++];
    state.channel_mask = buf[len++];
    state.channel_mask |= (buf[len++] << 8);
    state.channel_mask |= (buf[len++] << 16);
    state.channel_mask |= (buf[len++] << 24);
    state.charge_mask = buf[len++];
    len++;  // Swallow dock_state byte that may be stale
    for (size_t j = 0; j < EPS::NUM_BATTERIES; j++) {
      EPS::BatteryInfo &info = state.batteries[j];
      info.chan = buf[len++];
      info.present = buf[len++];
      info.voltage = (buf[len++] & 0x00FF);
      info.voltage |= (buf[len++] << 8);
      info.current = (buf[len++] & 0x00FF);
      info.current |= (buf[len++] << 8);
      info.remaining = (buf[len++] & 0x00FF);
      info.remaining |= (buf[len++] << 8);
      info.full = (buf[len++] & 0x00FF);
      info.full |= (buf[len++] << 8);
      info.design = (buf[len++] & 0x00FF);
      info.design |= (buf[len++] << 8);
      info.percentage = (buf[len++] & 0x00FF);
      info.percentage |= (buf[len++] << 8);
      for (size_t k = 0; k < 4; k++) {
        info.cell[k] = (buf[len++] & 0x00FF);
        info.cell[k] |= (buf[len++] << 8);
      }
      info.status = (buf[len++] & 0x00FF);
      info.status |= (buf[len++] << 8);
      info.temperature = (buf[len++] & 0x00FF);
      info.temperature |= (buf[len++] << 8);
      info.serial = (buf[len++] & 0x00FF);
      info.serial |= (buf[len++] << 8);
    }
    state.fault_mask = buf[len++] & 0x000000FF;
    state.fault_mask |= ((buf[len++] << 8) & 0x0000FF00);
    state.fault_mask |= ((buf[len++] << 16) & 0x00FF0000);
    state.fault_mask |= buf[len++] << 24;
  }
  return true;
}

// TO BE CHECKED BY LORENZO
bool SmartDock::GetSystemState(DockState &data) {
  uint8_t cmd[4] = {I2C_CMD_GET_SYSTEM_STATE, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Write(cmd, 4) != 4) return false;
  if (Read(buf, kDockStateLength) != kDockStateLength) return false;
  memcpy(&data, buf, kDockStateLength);
  return true;
}

// CHECKED
bool SmartDock::GetHousekeeping(uint32_t mask,
                                std::map<Housekeeping, double> &data) {
  uint8_t cmd[4] = {I2C_CMD_GET_HK, 1, 0, 0};
  uint8_t buf[I2C_BUF_MAX_LEN];
  if (Write(cmd, 4) != 4) return false;
  if (Read(buf, 32) != 32) return false;
  for (uint32_t i = 0; i < NUM_HOUSEKEEPING; i++) {
    if ((mask & (1 << i)) == 0) continue;
    uint16_t raw = (uint16_t)((buf[(i * 2) + 0] & 0x00FF) |
                              (uint16_t)((buf[(i * 2) + 1] << 8)));
    data[static_cast<Housekeeping>(i)] = static_cast<double>(raw) / 1000.0;
  }
  return true;
}

// CHECKED
bool SmartDock::SetLeds(uint32_t mask, LedMode const value) {
  uint32_t chanmask = 0x0;
  if (mask & (1 << LED_1)) chanmask |= (1 << CHANNEL_LED_1);
  if (mask & (1 << LED_2)) chanmask |= (1 << CHANNEL_LED_2);
  if (mask & (1 << LED_3)) chanmask |= (1 << CHANNEL_LED_3);
  if (mask & (1 << LED_4)) chanmask |= (1 << CHANNEL_LED_4);
  if (mask & (1 << LED_5)) chanmask |= (1 << CHANNEL_LED_5);
  if (mask & (1 << LED_6)) chanmask |= (1 << CHANNEL_LED_6);
  uint8_t cmd[9] = {
      I2C_CMD_SET_LED_MODES,  // Command
      6,
      0,                                              // Length
      0,                                              // Channel mask
      0,                                              // Channel mask
      static_cast<uint8_t>((chanmask >> 16) & 0xff),  // Channel mask
      static_cast<uint8_t>((chanmask >> 24) & 0xff),  // Channel mask
      static_cast<uint8_t>(value),                    // New value
      0x00};                                          // Checksum
  return (Write(cmd, 9) == 9);
}

// CHECKED
bool SmartDock::SetChannels(uint32_t mask, bool const value) {
  uint8_t cmd[8] = {value ? I2C_CMD_SW_ON : I2C_CMD_SW_OFF,  // Command
                    5,
                    0,                                          // Length
                    static_cast<uint8_t>(mask & 0xff),          // Mask LSB
                    static_cast<uint8_t>((mask >> 8) & 0xff),   // Mask
                    static_cast<uint8_t>((mask >> 16) & 0xff),  // Mask
                    static_cast<uint8_t>((mask >> 24) & 0xff),  // Mask MSB
                    0};                                         // Checksum
  return (Write(cmd, 8) == 8);
}

// PRIVATE

// Sleep function required by EPS driver
void SmartDock::Sleep(uint32_t microseconds) {
  struct timespec req, rem;
  req.tv_sec = microseconds / 1000000;
  req.tv_nsec = (microseconds % 1000000) * 100;
  while ((req.tv_sec != 0) || (req.tv_nsec != 0)) {
    if (nanosleep(&req, &rem) == 0) break;
    if (errno == EINTR) {
      req.tv_sec = rem.tv_sec;
      req.tv_nsec = rem.tv_nsec;
      continue;
    }
    std::cout << "Warning: nanosleep terminated prematurely" << std::endl;
    break;
  }
}

uint16_t SmartDock::Read(uint8_t *buff, uint16_t len) {
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

uint16_t SmartDock::Read(uint8_t *buff) {
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

uint16_t SmartDock::Write(uint8_t *buff, uint16_t len) {
  buff[len - 1] = ComputeChecksum(buff, len - 1);
  i2c::Error err;
  int size = i2c_dev_.Write(buff, len, &err);
  if (size < 0) {
    std::cerr << "Failed to write " << len << " Bytes over I2C: " << std::endl;
    std::cerr << std::strerror(err) << std::endl;
    return 0;
  }
  return static_cast<uint16_t>(size);
  // Sleep for 50ms
  Sleep(50000);
}

uint8_t SmartDock::ComputeChecksum(uint8_t *buf, size_t size) {
  uint8_t checksum = 0xFF;
  for (size_t i = 0; i < size; i++) checksum ^= buf[i];
  return checksum;
}

}  // namespace smart_dock
