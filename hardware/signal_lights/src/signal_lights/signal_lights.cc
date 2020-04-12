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

#include <custom_i2c/i2c.h>
#include <signal_lights/signal_lights.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace signal_lights {

Device::Device(const char *bus, uint8_t addr) {
  handle = custom_i2c::i2c_open(bus, addr);
}

int Device::read(uint8_t *buf, unsigned int length) {
  return custom_i2c::i2c_read(handle, buf, length);
}

int Device::write(uint8_t *buf, unsigned int length) {
  return custom_i2c::i2c_write(handle, buf, length);
}

// CONSTRUCTOR and DESTRUCTOR

SignalLights::SignalLights(const Device &i2c_dev)
    : i2c_dev_(i2c_dev), block_index_(0), metadata_index_(0) {
  for (size_t b = 0; b < NUM_BLOCKS; b++) {
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
      // on launch of FSW, all pixels should be a faint white
      // the maximum number is 31 and the minimum is 0
      command_[b].pixels[i].red = 1;
      command_[b].pixels[i].green = 1;
      command_[b].pixels[i].blue = 1;
      command_[b].pixels[i].ignore = 0;
    }
  }
}

SignalLights::~SignalLights() {}

// Configure and return data
bool SignalLights::Set(uint8_t pos, uint8_t red, uint8_t green, uint8_t blue) {
  uint8_t b = pos / BLOCK_SIZE;
  uint8_t i = pos % BLOCK_SIZE;
  if (b < NUM_BLOCKS && i < BLOCK_SIZE) {
    command_[b].pixels[i].red = red;
    command_[b].pixels[i].green = green;
    command_[b].pixels[i].blue = blue;
    command_[b].pixels[i].ignore = 0;
    return true;
  }
  return false;
}

void SignalLights::SetAll(uint8_t red, uint8_t green, uint8_t blue) {
  for (size_t b = 0; b < NUM_BLOCKS; b++) {
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
      command_[b].pixels[i].red = red;
      command_[b].pixels[i].green = green;
      command_[b].pixels[i].blue = blue;
      command_[b].pixels[i].ignore = 0;
    }
  }
}

// Get the polling frequency for a desired control rate
double SignalLights::GetPollDuration(double rate) {
  return (1.0 / rate - kRenderTime_secs_) / static_cast<double>(NUM_BLOCKS);
}

// Write a given block
bool SignalLights::Poll() {
  // Set the header of the block
  command_[block_index_].instruction.range = 0;  // 0 - 255
  command_[block_index_].instruction.mode = 1;   // Nominal
  command_[block_index_].instruction.render =
      (block_index_ == (NUM_BLOCKS - 1));
  command_[block_index_].instruction.mask = (1 << block_index_);
  // Increment the block number
  // Write the current command
  if (Write(reinterpret_cast<uint8_t *>(&command_[block_index_]),
            sizeof(LEDC_COMMAND)) != sizeof(LEDC_COMMAND))
    return false;

  block_index_ = (block_index_ + 1) % NUM_BLOCKS;

  // Only proceed if the current block is zero (we've just ended a frame)
  if (block_index_) return false;

  // Receive telemetry after a block is sent
  if (Read(reinterpret_cast<uint8_t *>(&telemetry_), sizeof(LEDC_TELEMETRY)) !=
      sizeof(LEDC_TELEMETRY))
    return false;

  // If the telemetry control bit is 1 then this is the start of new metadata
  bool metadata_found = false;
  if (telemetry_.status.control) {
    switch (metadata_.type) {
      case 0x1:
        // Check the CRC of the metadata block
        if (ComputeChecksum(reinterpret_cast<uint8_t *>(&metadata_),
                            sizeof(METADATA_VERSION)) == 0) {
          // Convert the hash
          char buf[128];
          for (size_t i = 0; i < sizeof(metadata_.hash); i++)
            snprintf(&buf[2 * i], 128 - 2 * i, "%02x", metadata_.hash[i]);
          hash_ = buf;
          // Convert the time
          time_t t = static_cast<time_t>(metadata_.time);
          char str[128];
          strftime(str, sizeof(str), "%Y-%m-%d %H:%M:%S %z", std::gmtime(&t));
          time_ = str;
          // Mark metadata as found
          metadata_found = true;
        }
        break;
    }
    // Reset once
    metadata_index_ = 0;
    memset(&metadata_, 0x0, sizeof(METADATA_VERSION));
  }

  // In all cases, shift on some metadata
  reinterpret_cast<uint8_t *>(&metadata_)[metadata_index_ / 8] |=
      (telemetry_.status.metadata << (metadata_index_ % 8));

  // Increment the metadata index
  metadata_index_++;

  if (metadata_index_ >= 26) metadata_index_ = 0;

  // Return if metadata was found
  return metadata_found;
}

// INTERNAL

// Write
uint16_t SignalLights::Read(uint8_t *buff, uint16_t len) {
  uint8_t tmp[128];
  if (i2c_dev_.read(buff, len) < 0) {
    std::cerr << "Failed to read data over I2C." << std::endl;
    return 0;
  }
  memcpy(buff, tmp, 9);
  return len;
}

// Read
uint16_t SignalLights::Write(uint8_t *buff, uint16_t len) {
  buff[len - 1] = ComputeChecksum(buff, len - 1);
  int size = i2c_dev_.write(buff, len);
  if (size < 0) {
    std::cerr << "Failed to write " << len << " bytes over I2C." << std::endl;
    return 0;
  }
  return static_cast<uint16_t>(size);
}

// Checksum
uint8_t SignalLights::ComputeChecksum(uint8_t *buf, size_t size) {
  uint8_t checksum = 0xFF;
  for (size_t i = 0; i < size; i++) checksum ^= buf[i];
  return checksum;
}

}  // namespace signal_lights
