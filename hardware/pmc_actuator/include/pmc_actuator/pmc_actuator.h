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

#include <atomic>
#include <mutex>
#include <thread>

namespace pmc_actuator {

enum CmdMode { SHUTDOWN = 0, NORMAL = 1, RESTART = 2 };

// Number of nozzles in each PMC.
constexpr int kNumNozzles = 6;

/**
 * Data structure represents the command packet - LLP to PMC.
 */
typedef struct _Command {
  uint8_t motor_speed;                    // Speed of the motor
  uint8_t nozzle_positions[kNumNozzles];  // Nozzle positions (0 - 255)
  uint8_t mode;                           // Mode
  uint8_t command_id;                     // Command ID
  uint8_t checksum;                       // Checksum
} Command;

constexpr size_t kCommandMsgLength = 10;

constexpr size_t kTemperatureSensorsCount = 7;

/**
 * Data structure represents the telemetry packet - PMC to LLP.
 */
typedef struct _Telemetry {
  // Speed of the motor.
  // Unitless. 0-255
  uint8_t motor_speed;

  // Current consumption.
  uint8_t motor_current;

  // V6 ? current
  uint8_t v6_current;

  // Pressure
  // Unitless?
  uint16_t pressure;

  // Temperatures
  uint8_t temperatures[kTemperatureSensorsCount];

  // Status 1
  uint8_t status_1;

  // Status 2
  uint8_t status_2;

  // Command ID
  uint8_t command_id;

  // Checksum_
  uint8_t checksum;
} Telemetry;

constexpr size_t kTelemetryMsgLength = 16;

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

  void PrintTelemetry(std::ostream &out, const Telemetry &telem);
};

/**
 * Class represents the physical PMC module.
 */
class PmcActuator : public PmcActuatorBase {
 private:
  i2c::Device i2c_dev_;

 public:
  explicit PmcActuator(const i2c::Device &i2c_dev);
  virtual ~PmcActuator(void);

  // Sends the command packet to the PMC over I2C.
  bool SendCommand(const Command &command);

  // Gets the telemetry packet from the PMC over I2C.
  bool GetTelemetry(Telemetry *telemetry);

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

  bool SendCommand(const Command &command);
  bool GetTelemetry(Telemetry *telemetry);
  i2c::Address GetAddress();
};

}  // namespace pmc_actuator

#endif  // PMC_ACTUATOR_PMC_ACTUATOR_H_
