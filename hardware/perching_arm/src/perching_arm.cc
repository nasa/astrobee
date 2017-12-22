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

#include <perching_arm/perching_arm.h>

namespace perching_arm {


// By default the arm is uninitialized
PerchingArm::PerchingArm() :
  serial_(std::bind(&PerchingArm::Read, this, std::placeholders::_1,
    std::placeholders::_2)), cb_raw_data_({}), cb_sleep_ms_({}),
    rx_state_(PROTOCOL_WF_HEADER_1), rx_ptr_(0), rx_datalen_(0) {
  serial_.SetTimeoutCallback(std::bind(&PerchingArm::Timeout, this), 5000);
  serial_.SetShutdownCallback(std::bind(&PerchingArm::Shutdown, this));
}

// Connect to the arm
PerchingArmResult PerchingArm::Connect(std::string const& port,
  uint32_t baud, PerchingArmSleepMsCallback cb_sleep_ms,
  PerchingArmRawDataCallback cb_raw_data) {
  // Check the serial opens and return failure if not
  if (!serial_.Open(port, baud))
    return RESULT_PORT_NOT_OPEN;
  // Make sure the DTR pin is set low be default
  serial_.SetResetPinState(serial::RESET_PIN_LOW);
  // Save the callbacks
  if (cb_raw_data) cb_raw_data_ = cb_raw_data;
  if (cb_sleep_ms) cb_sleep_ms_ = cb_sleep_ms;
  // Success!
  return RESULT_SUCCESS;
}

// Dosconnect from the arm
void PerchingArm::Disconnect() {
  serial_.Close();
  cb_raw_data_ = nullptr;
  cb_sleep_ms_ = nullptr;
}

bool PerchingArm::ResultToString(PerchingArmResult result, std::string & msg) {
  bool success = (result == RESULT_SUCCESS);
  switch (result) {
  case RESULT_SUCCESS:            msg = "Success";                  break;
  case RESULT_PORT_NOT_OPEN:      msg = "Port not open";            break;
  case RESULT_COMMAND_REJECTED:   msg = "Command rejected";         break;
  case RESULT_INVALID_COMMAND:    msg = "Invalid command";          break;
  case RESULT_PORT_WRITE_FAILURE: msg = "Port write failure";       break;
  case RESULT_PORT_INIT_FAILURE:  msg = "Port initialize failure";  break;
  case RESULT_FIRMWARE_ERROR:     msg = "Firmware error";           break;
  case RESULT_RESPONSE_TIMEOUT:   msg = "Response timeout";         break;
  case RESULT_OUT_OF_BOUNDS:      msg = "Out of bounds";            break;
  case RESULT_NOT_CALIBRATED:     msg = "Calibrate arm first";      break;
  }
  return success;
}

// Set the velocity of the proximal joint in revolutions per minute
PerchingArmResult PerchingArm::SetProximalVelocity(int16_t rpm) {
  if (rpm < PROX_VEL_MIN || rpm > PROX_VEL_MAX)
    return RESULT_OUT_OF_BOUNDS;
  return SendCommand(TARGET_PROXIMAL, ADDRESS_VELOCITY, rpm);
}

// Set the position of the proximal joint in whole degrees
PerchingArmResult PerchingArm::SetProximalPosition(int16_t deg) {
  if (deg < PROX_POS_MIN || deg > PROX_POS_MAX)
    return RESULT_OUT_OF_BOUNDS;
  return SendCommand(TARGET_PROXIMAL, ADDRESS_POSITION, deg);
}

// Enable or disabled the distal motor
PerchingArmResult PerchingArm::SetProximalEnabled(bool enabled) {
  return SendCommand(TARGET_PROXIMAL, ADDRESS_ENABLE, (enabled ? 1 : 0));
}

// Set the velocity of the distal joint in revolutions per minute
PerchingArmResult PerchingArm::SetDistalVelocity(int16_t rpm) {
  if (rpm < DIST_VEL_MIN || rpm > DIST_VEL_MAX)
    return RESULT_OUT_OF_BOUNDS;
  return SendCommand(TARGET_DISTAL, ADDRESS_VELOCITY, rpm);
}

// Set the position of the distal joint in whole degrees
PerchingArmResult PerchingArm::SetDistalPosition(int16_t deg) {
  if (deg < DIST_POS_MIN || deg > DIST_POS_MAX)
    return RESULT_OUT_OF_BOUNDS;
  return SendCommand(TARGET_DISTAL, ADDRESS_POSITION, deg);
}

// Enable or disabled the distal motor
PerchingArmResult PerchingArm::SetDistalEnabled(bool enabled) {
  return SendCommand(TARGET_DISTAL, ADDRESS_ENABLE, (enabled ? 1 : 0));
}

// Set the gripper position (in percentage from 0 to 100)
PerchingArmResult PerchingArm::CalibrateGripper() {
  return SendCommand(TARGET_GRIPPER, ADDRESS_GRIPPER_CALIBRATE, 0);
}

// Set the gripper position
PerchingArmResult PerchingArm::SetGripperPosition(int16_t perc) {
  // Special case: -100 is a goal for calibration. We do this so that
  // a simple joint state publisher can use this value to forcefully
  // recalibrate the gripper without a bespoke service.
  if (perc == GRIP_CALIBRATE)
    return CalibrateGripper();
  // We can only accept commands if we are calibrated!
  if (raw_.grip.maximum > 0) {
    if (perc < GRIP_POS_MIN || perc > GRIP_POS_MAX)
      return RESULT_OUT_OF_BOUNDS;
    // Get the number of encoder ticks as a percentage of the max
    double ticks = static_cast<double>(perc) / 100.0
                 * static_cast<double>(raw_.grip.maximum);
    // Send the command to open to this encoder tick value
    return SendCommand(TARGET_GRIPPER, ADDRESS_GRIPPER_SET,
      static_cast<int16_t>(ticks));
  }
  return RESULT_NOT_CALIBRATED;
}

// Calibrate the gripper position
PerchingArmResult PerchingArm::OpenGripper() {
  // We can only accept commands if we are calibrated!
  if (raw_.grip.maximum > 0)
    return SendCommand(TARGET_GRIPPER, ADDRESS_GRIPPER_OPEN, 0);
  return RESULT_NOT_CALIBRATED;
}

// Calibrate the gripper position
PerchingArmResult PerchingArm::CloseGripper() {
  // We can only accept commands if we are calibrated!
  if (raw_.grip.maximum > 0)
    return SendCommand(TARGET_GRIPPER, ADDRESS_GRIPPER_CLOSE, 0);
  return RESULT_NOT_CALIBRATED;
}

// Reset the software
PerchingArmResult PerchingArm::SoftReset() {
  return SendCommand(TARGET_RESET, ADDRESS_RESET, 0);
}

// Reset the hardware
PerchingArmResult PerchingArm::HardReset() {
  // Check serial is open
  if (!serial_.IsOpen())
    return RESULT_PORT_NOT_OPEN;
  // Reset the board
  serial_.SetResetPinState(serial::RESET_PIN_HIGH);
  cb_sleep_ms_(500);
  serial_.SetResetPinState(serial::RESET_PIN_LOW);
  return RESULT_SUCCESS;
}

// Blocking write
PerchingArmResult PerchingArm::SendCommand(
  uint8_t target, int16_t address, int16_t data) {
  if (!serial_.IsOpen())
    return RESULT_PORT_NOT_OPEN;
  uint8_t txBuffer[9] = {0xff, 0xff, 6};
  txBuffer[3] = target;
  txBuffer[4] = (address & 0xff);
  txBuffer[5] = ((address >> 8) & 0xff);
  txBuffer[6] = (data & 0xff);
  txBuffer[7] = ((data >> 8) & 0xff);
  txBuffer[8] = Checksum(txBuffer, sizeof(txBuffer));
  if (serial_.Write(txBuffer, sizeof(txBuffer)) != sizeof(txBuffer))
    return RESULT_PORT_WRITE_FAILURE;
  return RESULT_SUCCESS;
}

// Process a packet
void PerchingArm::Process(const uint8_t* buf, size_t len) {
  // Prevent a buffer overrun
  if (len != 34) return;
  // Convert the raw byte stream to a struct
  raw_.prox.load        = (static_cast<int16_t>(buf[ 1]) << 8)   // FEEDBACK[00]
                        |  static_cast<int16_t>(buf[ 0]);
  raw_.prox.velocity    = (static_cast<int32_t>(buf[ 5]) << 24)  // FEEDBACK[01]
                        | (static_cast<int32_t>(buf[ 4]) << 16)
                        | (static_cast<int32_t>(buf[ 3]) << 8)
                        |  static_cast<int32_t>(buf[ 2]);
  raw_.prox.position    = (static_cast<int32_t>(buf[ 9]) << 24)  // FEEDBACK[02]
                        | (static_cast<int32_t>(buf[ 8]) << 16)
                        | (static_cast<int32_t>(buf[ 7]) << 8)
                        |  static_cast<int32_t>(buf[ 6]);
  raw_.dist.load        = (static_cast<int16_t>(buf[11]) << 8)   // FEEDBACK[03]
                        |  static_cast<int16_t>(buf[10]);
  raw_.dist.velocity    = (static_cast<int32_t>(buf[15]) << 24)  // FEEDBACK[04]
                        | (static_cast<int32_t>(buf[14]) << 16)
                        | (static_cast<int32_t>(buf[13]) << 8)
                        |  static_cast<int32_t>(buf[12]);
  raw_.dist.position    = (static_cast<int32_t>(buf[19]) << 24)  // FEEDBACK[05]
                        | (static_cast<int32_t>(buf[18]) << 16)
                        | (static_cast<int32_t>(buf[17]) << 8)
                        |  static_cast<int32_t>(buf[16]);
  raw_.grip.position    = (static_cast<int16_t>(buf[21]) << 8)   // FEEDBACK[06]
                        |  static_cast<int16_t>(buf[20]);
  raw_.grip.load        = (static_cast<int16_t>(buf[23]) << 8)   // FEEDBACK[07]
                        |  static_cast<int16_t>(buf[22]);
  raw_.current_11v      = (static_cast<int16_t>(buf[25]) << 8)   // FEEDBACK[08]
                        |  static_cast<int16_t>(buf[24]);
  raw_.current_5v       = (static_cast<int16_t>(buf[27]) << 8)   // FEEDBACK[09]
                        |  static_cast<int16_t>(buf[26]);
  raw_.loop_time        = (static_cast<int16_t>(buf[29]) << 8)   // FEEDBACK[10]
                        |  static_cast<int16_t>(buf[28]);
  raw_.board_temp       = (static_cast<int16_t>(buf[31]) << 8)   // FEEDBACK[11]
                        |  static_cast<int16_t>(buf[30]);
  raw_.grip.maximum     = (static_cast<int16_t>(buf[33]) << 8)   // FEEDBACK[06]
                        |  static_cast<int16_t>(buf[32]);
  // Sign adjustments
  raw_.prox.position -= 2048;
  raw_.dist.position -= 2048;
  // Push the raw data
  if (cb_raw_data_)
    cb_raw_data_(raw_);
}

// Callback with serial data
void PerchingArm::Read(const uint8_t* buf, size_t len) {
  for (size_t i = 0; i < len; i++)
    FiniteStateMachine(buf[i]);
}

// Called when we time out
void PerchingArm::Timeout(void) {
  std::cout << "Timeout" << std::endl;
}

// Called when we time out
void PerchingArm::Shutdown(void) {
  std::cout << "Shutdown" << std::endl;
}

// Process a single character as part of the protocol state machine
void PerchingArm::FiniteStateMachine(uint8_t const& c) {
  // If we are waiting for header 1, then the packet pointer is reset
  if (rx_state_ == PROTOCOL_WF_HEADER_1)
    rx_ptr_ = 0;
  // Copy the data
  if (rx_ptr_ < MAX_PACKET_SIZE)
    rx_buf_[rx_ptr_++] = c;
  // Advance the state machine
  switch (rx_state_) {
  // Wait for header 1
  case PROTOCOL_WF_HEADER_1:
    if (c != PROTOCOL_HEADER_1) {
      rx_state_ = PROTOCOL_WF_HEADER_1;
      return;
    }
    rx_state_ = PROTOCOL_WF_HEADER_2;
    return;
  // Wait for header 2
  case PROTOCOL_WF_HEADER_2:
    if (c != PROTOCOL_HEADER_2) {
      rx_state_ = PROTOCOL_WF_HEADER_1;
      return;
    }
    rx_state_ = PROTOCOL_WF_LENGTH;
    return;
  // Wait for packet length
  case PROTOCOL_WF_LENGTH:
    rx_datalen_ = c - 1;
    rx_state_ = PROTOCOL_WF_DATA;
    return;
  // Wait until all data rx
  case PROTOCOL_WF_DATA:
    if (--rx_datalen_)
      return;
    rx_state_ = PROTOCOL_WF_CHECKSUM;
    return;
  // Wait for checksum
  case PROTOCOL_WF_CHECKSUM:
    if (Checksum(rx_buf_, rx_ptr_) == rx_buf_[rx_ptr_ - 1])
      Process(&rx_buf_[3], rx_ptr_ - 4);
    rx_state_ = PROTOCOL_WF_HEADER_1;
    return;
  }
}

// Checksum calculation
uint8_t PerchingArm::Checksum(const uint8_t* buf, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 2; i < len - 1; i++)
    sum += buf[i];
  return ~sum;
}

}  // namespace perching_arm
