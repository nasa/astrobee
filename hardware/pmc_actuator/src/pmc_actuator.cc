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

#include <pmc_actuator/pmc_actuator.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>

#include <fstream>

namespace pmc_actuator {

PmcActuator::PmcActuator(const i2c::Device &i2c_dev) : i2c_dev_(i2c_dev) {}

PmcActuator::~PmcActuator(void) {}

bool PmcActuator::SendCommand(const Command &command) {
  uint8_t buf[sizeof(Command)];
  i2c::Error err;
  int pos = 0;

  buf[pos++] = command.motor_speed;

  for (int i = 0; i < kNumNozzles; i++) {
    buf[pos++] = command.nozzle_positions[i];
  }
  buf[pos++] = command.mode;
  buf[pos++] = command.command_id;
  buf[pos++] = ComputeChecksum(buf, kCommandMsgLength - 1);

  if (i2c_dev_.Write(buf, kCommandMsgLength, &err) != kCommandMsgLength) {
    std::cerr << "Failed to write " << kCommandMsgLength << " Bytes over I2C: ";
    std::cerr << std::strerror(err) << std::endl;
    return false;
  }

  return true;
}

bool PmcActuator::GetTelemetry(Telemetry *telemetry) {
  uint8_t buf[sizeof(Telemetry)];
  i2c::Error err;

  if (i2c_dev_.Read(buf, kTelemetryMsgLength, &err) != kTelemetryMsgLength) {
    std::cerr << "Failed to read " << kTelemetryMsgLength
              << " Bytes over I2C: ";
    std::cerr << std::strerror(err) << std::endl;
    return false;
  }

  if (ComputeChecksum(buf, kTelemetryMsgLength) != 0) {
    std::cerr << "Telemetry data checksum failed" << std::endl;
    return false;
  }

  // Copy the data.
  size_t pos = 0;
  telemetry->motor_speed = buf[pos++];
  telemetry->motor_current = buf[pos++];
  telemetry->v6_current = buf[pos++];
  // PMC is little endian.
  telemetry->pressure = buf[pos + 1] << 8 | (buf[pos] & 0x00FF);
  pos += 2;
  for (size_t i = 0; i < kTemperatureSensorsCount; i++) {
    telemetry->temperatures[i] = buf[pos++];
  }
  telemetry->status_1 = buf[pos++];
  telemetry->status_2 = buf[pos++];
  telemetry->command_id = buf[pos++];
  telemetry->checksum = buf[pos++];

  return true;
}

uint8_t PmcActuator::ComputeChecksum(const uint8_t *buf, size_t size) {
  uint8_t checksum = 0xFF;
  for (size_t i = 0; i < size; i++) checksum ^= buf[i];
  return checksum;
}

i2c::Address PmcActuator::GetAddress() {
  return i2c_dev_.addr();
}

PmcActuatorStub::PmcActuatorStub(int addr, std::fstream &cmds_out,
                                 std::fstream &telem_out)
    : addr_(addr), cmds_output_(cmds_out), telem_output_(telem_out) {
  std::cout << "OPENING:" << addr_ << std::endl;
}

PmcActuatorStub::~PmcActuatorStub() {
  std::cout << "CLOSING: " << addr_ << std::endl;
}

bool PmcActuatorStub::SendCommand(const Command &command) {
  command_ = command;
  cmds_output_ << "addr=0x" << std::hex << addr_ << std::dec;
  cmds_output_.fill('0');
  cmds_output_ << " id=" << std::setw(3)
               << static_cast<unsigned int>(command.command_id);
  cmds_output_ << " mode=" << std::setw(1)
               << static_cast<unsigned int>(command.mode);
  cmds_output_ << " speed=" << std::setw(3)
               << static_cast<unsigned int>(command.motor_speed);
  cmds_output_ << " nozzles=";
  for (int i = 0; i < kNumNozzles; i++) {
    cmds_output_ << std::setw(3)
                 << static_cast<unsigned int>(command.nozzle_positions[i])
                 << " ";
  }
  cmds_output_ << std::endl << std::flush;
  cmds_output_.copyfmt(std::ios(NULL));
  return true;
}

bool PmcActuatorStub::GetTelemetry(Telemetry *telemetry) {
  telemetry->motor_speed = command_.motor_speed;
  telemetry->motor_current = command_.motor_speed / 2;
  float current = 0;
  for (int i = 0; i < kNumNozzles; i++) {
    current += command_.nozzle_positions[i];
  }
  telemetry->v6_current = static_cast<uint8_t>(current / 8.0);
  telemetry->pressure = 2 * static_cast<uint16_t>(command_.motor_speed);
  telemetry->command_id = command_.command_id;
  telemetry->status_1 = command_.mode;
  telemetry_ = *telemetry;
  PrintTelemetry(telem_output_, telemetry_);
  return true;
}

i2c::Address PmcActuatorStub::GetAddress() {
  return addr_;
}

void PmcActuatorBase::PrintTelemetry(std::ostream &out,
                                     const Telemetry &telem) {
  out << "addr=0x" << std::hex << GetAddress() << std::dec;
  out.fill('0');
  out << " id=" << std::setw(3) << static_cast<unsigned int>(telem.command_id);
  out << " status_1=" << std::setw(1)
      << static_cast<unsigned int>(telem.status_1);
  out << " status_2=" << std::setw(1)
      << static_cast<unsigned int>(telem.status_2);
  out << " speed=" << std::setw(3)
      << static_cast<unsigned int>(telem.motor_speed);
  out << " motor_current=" << std::setw(3)
      << static_cast<unsigned int>(telem.motor_current);
  out << " v6_current=" << std::setw(3)
      << static_cast<unsigned int>(telem.v6_current);
  out << " temps=";
  for (unsigned int i = 0; i < kTemperatureSensorsCount; i++) {
    out << std::setw(3) << static_cast<unsigned int>(telem.temperatures[i])
        << " ";
  }
  out << std::endl << std::flush;
  out.copyfmt(std::ios(NULL));
}

}  // namespace pmc_actuator
