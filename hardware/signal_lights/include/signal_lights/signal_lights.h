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

#ifndef SIGNAL_LIGHTS_SIGNAL_LIGHTS_H_
#define SIGNAL_LIGHTS_SIGNAL_LIGHTS_H_


#include <functional>
#include <string>

#include "../custom_i2c/i2c.h"

namespace signal_lights {

// I2C COMMUNICATION PROTOCOL

#define NUM_LEDS 44
#define NUM_BLOCKS 4
#define BLOCK_SIZE 11

typedef enum {
  LEDC_MODE_SHUTDOWN = 0,
  LEDC_MODE_NOMINAL,
  LEDC_MODE_INIT
} LEDC_MODE;

typedef enum {
  LEDC_STATE_NO_MODE_CHANGE = 0,
  LEDC_STATE_OOR_STATE,
  LEDC_STATE_MODE_CHANGE_TO_SHUTDOWN,
  LEDC_STATE_MODE_CHANGE_TO_NOMINAL,
  LEDC_STATE_TIMEOUT_INDUCED_SHUTDOWN,
  LEDC_STATE_RESERVED,
  LEDC_STATE_PIC_STARTUP,
  LEDC_STATE_ERROR
} LEDC_STATE;

typedef enum {
  LEDC_ERROR_NONE = 0,
  LEDC_ERROR_PACKET_CRC,
  LEDC_ERROR_PACKET_DROPPED,
  LEDC_ERROR_OVERRUN,
} LEDC_ERROR;

typedef union {
  uint16_t raw;
  struct {
    uint8_t red : 5;     // Red bits
    uint8_t green : 5;   // Green bits
    uint8_t blue : 5;    // Blue bits
    uint8_t ignore : 1;  // Ignore this pixel
  } __attribute__((packed));
} LEDC_COMMAND_COLOR;

typedef union {
  uint8_t raw;
  struct {
    uint8_t render : 1;  // Render the LEDs after this packet
    uint8_t mask : 4;    // Blocks - 0: 0:10, 0: 11:21, 0: 22:32, 3: 33:43
    uint8_t range : 2;   // Range k (0: 255, 1: 128, 2: 64: 3: 32)
    uint8_t mode : 1;    // Operating mode
  } __attribute__((packed));
} LEDC_COMMAND_INSTRUCTION;

typedef struct __attribute__((packed)) {
  LEDC_COMMAND_INSTRUCTION instruction;   // Instruction (1)
  LEDC_COMMAND_COLOR pixels[BLOCK_SIZE];  // LED block values (22)
  uint8_t checksum;                       // Checksum (1)
} LEDC_COMMAND;

typedef union {
  uint8_t raw;
  struct {
    uint8_t control : 1;   // Firmware version reset bit
    uint8_t metadata : 1;  // Firmware version data bit
    uint8_t error : 3;     // Packet errors - DROP, OVERRUN, CRC
    uint8_t state : 3;     // Current state
  } __attribute__((packed));
} LEDC_TELEMETRY_STATUS;

typedef struct __attribute__((packed)) {
  LEDC_TELEMETRY_STATUS status;
  uint8_t checksum;
} LEDC_TELEMETRY;

typedef enum {
  METADATA_TYPE_VERSION = 0x1,
  METADATA_TYPE_INVALID = 0x0
} METADATA_TYPE;

typedef struct __attribute__((packed)) {
  uint8_t type;      // 0x1 for METADATA_FIRMWARE
  uint8_t hash[20];  // 20 byte hash
  uint32_t time;     // Unix timestamp
  uint8_t chksum;
} METADATA_VERSION;

class Device {
 public:
  Device(const char* bus, uint8_t addr);
  int read(uint8_t *buf, unsigned int length);
  int write(uint8_t *buf, unsigned int length);

 private:
  int handle;
};

// A single signal light system
class SignalLights {
 public:
  static constexpr double kRenderTime_secs_ = 50e-6;

  // Constructor
  explicit SignalLights(const Device &i2c_dev);

  // Destructor
  virtual ~SignalLights();

  // Configure and return data
  bool Set(uint8_t pos, uint8_t red, uint8_t green, uint8_t blue);

  // Get the polling frequency for a desired control rate
  double GetPollDuration(double rate);

  // Poll a given block and return if metadata was received
  bool Poll();

  // Get the firmware hash ans build time
  std::string GetHash() { return hash_; }
  std::string GetTime() { return time_; }

 private:
  uint16_t Read(uint8_t *buff, uint16_t len);
  uint16_t Read(uint8_t *buff);
  uint16_t Write(uint8_t *buff, uint16_t len);
  uint8_t ComputeChecksum(uint8_t *buf, size_t size);

  // Local parameters
  Device i2c_dev_;
  LEDC_COMMAND command_[NUM_BLOCKS];
  LEDC_TELEMETRY telemetry_;
  METADATA_VERSION metadata_;
  size_t block_index_, metadata_index_;
  std::string hash_, time_;
};

}  // namespace signal_lights

#endif  // SIGNAL_LIGHTS_SIGNAL_LIGHTS_H_
