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

#ifndef PMC_ACTUATOR_PMC_ACTUATOR_H_
#define PMC_ACTUATOR_PMC_ACTUATOR_H_

#include <i2c/i2c_new.h>

#include <string>
#include <atomic>
#include <mutex>
#include <thread>

namespace pmc_actuator {

enum CmdMode { SHUTDOWN = 0, NORMAL = 1, RESTART  = 2 };

////////////////////////////////////////////////////////////////////////////////
// This must be kept in sync with the protocol declared in the PMC firmware   //
////////////////////////////////////////////////////////////////////////////////

constexpr int kNumNozzles = 6;
constexpr size_t kCommandMsgLength = 10;
constexpr size_t kTemperatureSensorsCount = 7;
constexpr size_t kTelemetryMsgLength = 16;
constexpr size_t kGitHashLength = 40;
constexpr size_t kDateLength = 17;
constexpr size_t kMaxMetadataLength = 64;
constexpr uint8_t kMetadataTypePrototype = 0x1;
constexpr uint8_t kMetadataTypeFlight = 0x2;

typedef struct __attribute__((packed)) {
  uint8_t motor_speed;
  uint8_t nozzle_positions[kNumNozzles];
  uint8_t mode;
  uint8_t command_id;
  uint8_t checksum;
} Command;

typedef union {
  uint8_t asUint8;
  struct {
    uint8_t mode          :2;
    uint8_t stateStatus   :3;
    uint8_t mcDisabled    :1;
    uint8_t errBadCRC     :1;
    uint8_t errNozCmdOOR  :1;
  };
} Status1;

typedef union {
  uint8_t asUint8;
  struct {
    uint8_t errOvrCurrent :1;
    uint8_t speedCamState :1;
    uint8_t control       :1;
    uint8_t metadata      :1;
    uint8_t reserved      :4;
  };
} Status2;

typedef struct __attribute__((packed)) {
  uint8_t motor_speed;
  uint8_t motor_current;
  uint8_t v6_current;
  uint16_t pressure;
  uint8_t temperatures[kTemperatureSensorsCount];
  Status1 status_1;
  Status2 status_2;
  uint8_t command_id;
  uint8_t checksum;
} Telemetry;

// A metadata frame containing the git hash and time of compilation
typedef struct  __attribute__((packed)) {
    uint8_t type;        // 0x1 for METADATA_FIRMWARE
    uint8_t hash[20];    // 20 byte hash
    uint32_t time;       // Unix timestamp
    uint8_t checksum;
} MetadataVersion;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * Base class for the PMC module
 */
class PmcActuatorBase {
 public:
  // Sends the command packet.
  virtual bool SendCommand(const Command &command) = 0;

  // Gets the telemetry packet.
  virtual bool GetTelemetry(Telemetry *telemetry) = 0;

  virtual i2c::Address GetAddress() = 0;

  // Get the firmware version
  virtual bool GetFirmwareHash(std::string & hash) = 0;

  // Get the firmware date
  virtual bool GetFirmwareTime(std::string & date) = 0;

  // Get the firmware type
  virtual bool GetFirmwareType(uint8_t & type) = 0;

  // Print out telemetry
  void PrintTelemetry(std::ostream &out, const Telemetry &telem);
};

/**
 * Class represents the physical PMC module.
 */
class PmcActuator : public PmcActuatorBase {
 private:
  i2c::Device i2c_dev_;
  bool metadata_received_;                        // Do we have valid metadata?
  uint16_t metadata_index_;                       // Current metadata bit
  uint8_t metadata_buffer_[kMaxMetadataLength];   // Buffer for metadata
  MetadataVersion metadata_;                      // Last complete metadata

 public:
  explicit PmcActuator(const i2c::Device &i2c_dev);
  virtual ~PmcActuator(void);

  // Sends the command packet to the PMC over I2C.
  bool SendCommand(const Command &command);

  // Gets the telemetry packet from the PMC over I2C.
  bool GetTelemetry(Telemetry *telemetry);

  // Get the firmware version
  bool GetFirmwareHash(std::string & hash);

  // Get the firmware date
  bool GetFirmwareTime(std::string & date);

  // Get the firmware type
  bool GetFirmwareType(uint8_t & type);

  // Get the i2c address
  i2c::Address GetAddress();

 private:
  uint8_t ComputeChecksum(const uint8_t *buf, size_t size);
};

class PmcActuatorStub : public PmcActuatorBase {
 private:
  i2c::Address addr_;
  std::fstream &cmds_output_;
  std::fstream &telem_output_;
  Command command_;
  Telemetry telemetry_;

 public:
  explicit PmcActuatorStub(int addr, std::fstream &cmds_out,
                           std::fstream &telem_out);
  virtual ~PmcActuatorStub();

  // Sends the command packet to the PMC over I2C.
  bool SendCommand(const Command &command);

  // Gets the telemetry packet from the PMC over I2C.
  bool GetTelemetry(Telemetry *telemetry);

  // Get the firmware version
  bool GetFirmwareHash(std::string & hash);

  // Get the firmware date
  bool GetFirmwareTime(std::string & date);

  // Get the firmware date
  bool GetFirmwareType(uint8_t & type);

  i2c::Address GetAddress();
};

}  // namespace pmc_actuator

#endif  // PMC_ACTUATOR_PMC_ACTUATOR_H_
