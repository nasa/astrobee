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

// Protocol header definition
#define PROTOCOL_HEADER_1                   0xFF
#define PROTOCOL_HEADER_2                   0xFF

// Set tilt motor
#define PROTOCOL_TILT_TARGET                0
#define PROTOCOL_TILT_ADDRESS_POSITION      1
#define PROTOCOL_TILT_ADDRESS_VELOCITY      2

// Set pan motor
#define PROTOCOL_PAN_TARGET                 1
#define PROTOCOL_PAN_ADDRESS_POSITION       1
#define PROTOCOL_PAN_ADDRESS_VELOCITY       2

// Enable or disable motors (unused for now)
#define PROTOCOL_TOGGLE_TARGET              2
#define PROTOCOL_TOGGLE_VALUE_DISABLE       0
#define PROTOCOL_TOGGLE_VALUE_ENABLE        1

// Torque limits (unused for now)
#define PROTOCOL_TORQUE_LIMIT_TARGET        3
#define PROTOCOL_TORQUE_LIMIT_ADDRESS       34

// Gripper protocol defines
#define PROTOCOL_GRIPPER_TARGET             4
#define PROTOCOL_GRIPPER_ADDRESS_CALIBRATE  51
#define PROTOCOL_GRIPPER_ADDRESS_OPEN       52
#define PROTOCOL_GRIPPER_ADDRESS_CLOSE      53
#define PROTOCOL_GRIPPER_ADDRESS_SET        54
#define PROTOCOL_GRIPPER_ADDRESS_RESET      55

// System reset defines
#define PROTOCOL_RESET_TARGET               10
#define PROTOCOL_RESET_ADDRESS              10

// Constants
const int16_t PerchingArm::GRIPPER_OPEN_THRESHOLD  = 100;
const int16_t PerchingArm::GRIPPER_CLOSE_THRESHOLD = 0;
const int16_t PerchingArm::DEPLOY_TILT             = -90;
const int16_t PerchingArm::STOW_TILT               = 90;
const int16_t PerchingArm::STOW_PAN                = 0;
const int16_t PerchingArm::DEPLOY_PAN              = 0;
const int16_t PerchingArm::MIN_PAN                 = -90;
const int16_t PerchingArm::MAX_PAN                 = 90;
const int16_t PerchingArm::MIN_TILT                = -120;
const int16_t PerchingArm::MAX_TILT                = 0;
const float PerchingArm::MOTOR_POSITION_SCALE      = 0.088f;
const float PerchingArm::MOTOR_VELOCITY_SCALE      = 0.229f;
const float PerchingArm::MOTOR_LOAD_SCALE          = 2.690f;
const float PerchingArm::GRIPPER_POSITION_SCALE    = 0.039215686f;
const float PerchingArm::GRIPPER_LOAD_SCALE        = 3.3f / (1024.0f * 0.525f);
const float PerchingArm::CURRENT_5V_SCALE          = 3.3f / (1024.0f * 100.0f * 0.0100f);
const float PerchingArm::CURRENT_11V_SCALE         = 3.3f / (1024.0f * 100.0f * 0.0075f);
const float PerchingArm::BOARD_TEMP_SCALE          = 1.0f / 100.0f;
const float PerchingArm::LOOP_TIME_SCALE           = 1.0f;

// By default the arm is uninitialized
PerchingArm::PerchingArm() :
  serial_(std::bind(&PerchingArm::Read, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PerchingArm::Timeout, this), 1000),
  joint_state_(JOINT_STATE_UNKNOWN),
  gripper_state_(GRIPPER_STATE_CLOSED),
  cb_event_({}),
  cb_feedback_({}),
  cb_sleep_ms_({}),
  rx_ptr_(0),
  rx_datalen_(0),
  rx_state_(PROTOCOL_WF_HEADER_1) {}

// Initialize the serial port
PerchingArmResult PerchingArm::Initialize(std::string const& port, uint32_t baud,
  PerchingArmSleepMsCallback cb_sleep_ms, PerchingArmEventCallback cb_event, PerchingArmFeedbackCallback cb_feedback) {
  // Check the serial opens and return failure if not
  if (!serial_.Open(port, baud))
    return RESULT_PORT_NOT_OPEN;
  // Save the callbacks
  if (cb_event) cb_event_ = cb_event;
  if (cb_feedback) cb_feedback_ = cb_feedback;
  if (cb_sleep_ms) cb_sleep_ms_ = cb_sleep_ms;
  // Success!
  return RESULT_SUCCESS;
}

// Deploy the gripper
PerchingArmResult PerchingArm::Deploy() {
  // We can only deploy from a stowed state with the gripper closed
  if (joint_state_ != JOINT_STATE_STOWED || gripper_state_ != GRIPPER_STATE_CLOSED)
    return RESULT_COMMAND_REJECTED;
  // Perform a float to int conversion (good to 1 degree error)
  goal_pan_ = PerchingArm::DEPLOY_PAN;
  goal_tilt_ = PerchingArm::DEPLOY_TILT;
  // Set the state to deploying
  joint_state_ = JOINT_STATE_DEPLOYING;
  // Start the deploy action
  return SendCommand(PROTOCOL_TILT_TARGET, PROTOCOL_TILT_ADDRESS_POSITION, goal_tilt_);
}

// Stow the arm
PerchingArmResult PerchingArm::Stow() {
  // We can only deploy from a stopped state with the gripper closed
  if (joint_state_ != JOINT_STATE_STOPPED || gripper_state_ != GRIPPER_STATE_CLOSED)
    return RESULT_COMMAND_REJECTED;
  // Save the initial position for progress calculation
  initial_gripper_ = feedback_.gripper.position;
  initial_pan_ = feedback_.pan.position;
  initial_tilt_ = feedback_.tilt.position;
  // Perform a float to int conversion (good to 1 degree error)
  goal_pan_ = PerchingArm::STOW_PAN;
  goal_tilt_ = PerchingArm::STOW_TILT;
  // Set the state to deploying
  joint_state_ = JOINT_STATE_STOWING_PANNING;
  // Start the move action
  return SendCommand(PROTOCOL_PAN_TARGET, PROTOCOL_PAN_ADDRESS_POSITION, goal_pan_);
}

// Move (pan/tilt) the arm
PerchingArmResult PerchingArm::Move(int16_t pan, int16_t tilt) {
  // We can only deploy from a stowed state with the gripper closed
  if (joint_state_ != JOINT_STATE_STOPPED)
    return RESULT_COMMAND_REJECTED;
  if (pan < PerchingArm::MIN_PAN || pan > PerchingArm::MAX_PAN)
    return RESULT_COMMAND_REJECTED;
  if (tilt < PerchingArm::MIN_TILT || tilt > PerchingArm::MAX_TILT)
    return RESULT_COMMAND_REJECTED;
  // Save the initial position for progress calculation
  initial_pan_ = feedback_.pan.position;
  initial_tilt_ = feedback_.tilt.position;
  // Perform a float to int conversion (good to 1 degree error)
  goal_pan_ = pan;
  goal_tilt_ = tilt;
  // Set the state
  joint_state_ = JOINT_STATE_MOVING_PANNING;
  // Send the new command
  return SendCommand(PROTOCOL_PAN_TARGET, PROTOCOL_PAN_ADDRESS_POSITION, goal_pan_);
}

// Pan the arm
PerchingArmResult PerchingArm::Pan(int16_t pan) {
  // We can only deploy from a stowed state with the gripper closed
  if (joint_state_ != JOINT_STATE_STOPPED)
    return RESULT_COMMAND_REJECTED;
  if (pan < PerchingArm::MIN_PAN || pan > PerchingArm::MAX_PAN)
    return RESULT_COMMAND_REJECTED;
  // Save the initial pan position for progress calculation
  initial_pan_ = feedback_.pan.position;
  initial_tilt_ = feedback_.tilt.position;
  // Perform a float to int conversion (good to 1 degree error)
  goal_pan_ = pan;
  goal_tilt_ = initial_tilt_;
  // Set the state
  joint_state_ = JOINT_STATE_PANNING;
  // Start the move action
  return SendCommand(PROTOCOL_PAN_TARGET, PROTOCOL_PAN_ADDRESS_POSITION, goal_pan_);
}

// Tilt the arm
PerchingArmResult PerchingArm::Tilt(int16_t tilt) {
  // We can only deploy from a stowed state with the gripper closed
  if (joint_state_ != JOINT_STATE_STOPPED)
    return RESULT_COMMAND_REJECTED;
  if (tilt < PerchingArm::MIN_TILT || tilt > PerchingArm::MAX_TILT)
    return RESULT_COMMAND_REJECTED;
  // Save the initial tilt position for progress calculation
  initial_pan_ = feedback_.pan.position;
  initial_tilt_ = feedback_.tilt.position;
  // Perform a float to int conversion (good to 1 degree error)
  goal_pan_ = initial_pan_;
  goal_tilt_ = tilt;
  // Set the state
  joint_state_ = JOINT_STATE_TILTING;
  // Start the move action
  return SendCommand(PROTOCOL_TILT_TARGET, PROTOCOL_TILT_ADDRESS_POSITION, goal_tilt_);
}

// Open the gripper
PerchingArmResult PerchingArm::Open() {
  // We can only open the gripper if the arm is stopped and the gripper is closed
  if (joint_state_ != JOINT_STATE_STOPPED || gripper_state_ != GRIPPER_STATE_CLOSED)
    return RESULT_COMMAND_REJECTED;
  // Gripper position
  initial_gripper_ = feedback_.gripper.position;
  // Set the state
  gripper_state_ = GRIPPER_STATE_OPENING;
  // Start the move action
  return SendCommand(PROTOCOL_GRIPPER_TARGET, PROTOCOL_GRIPPER_ADDRESS_OPEN, 0);
}

// Close the gripper
PerchingArmResult PerchingArm::Close() {
  // We can only open the gripper if the arm is stopped and the gripper is open
  if (joint_state_ != JOINT_STATE_STOPPED || gripper_state_ != GRIPPER_STATE_OPEN)
    return RESULT_COMMAND_REJECTED;
  // Gripper position
  initial_gripper_ = feedback_.gripper.position;
  // Set the state
  gripper_state_ = GRIPPER_STATE_CLOSING;
  // Start the move action
  return SendCommand(PROTOCOL_GRIPPER_TARGET, PROTOCOL_GRIPPER_ADDRESS_CLOSE, 0);
}

// Calibrate the gripper
PerchingArmResult PerchingArm::Calibrate() {
  // We can only open the gripper if the arm is stopped
  if (joint_state_ != JOINT_STATE_STOPPED)
    return RESULT_COMMAND_REJECTED;
  // Gripper position
  initial_gripper_ = feedback_.gripper.position;
  // Set the state
  gripper_state_ = GRIPPER_STATE_CALIBRATING;
  // Start the move action
  return SendCommand(PROTOCOL_GRIPPER_TARGET, PROTOCOL_GRIPPER_ADDRESS_CALIBRATE, 0);
}

// Stop the arm
PerchingArmResult PerchingArm::Stop() {
  // Set the state
  joint_state_ = JOINT_STATE_STOPPED;
  // Set the velocity to zero on both degrees of freedom
  PerchingArmResult result;
  result = SendCommand(PROTOCOL_PAN_TARGET, PROTOCOL_PAN_ADDRESS_VELOCITY, 0);
  if (result != RESULT_SUCCESS)
    return result;
  return SendCommand(PROTOCOL_TILT_TARGET, PROTOCOL_TILT_ADDRESS_VELOCITY, 0);
}

// Reset the software
PerchingArmResult PerchingArm::SoftReset() {
  return SendCommand(PROTOCOL_RESET_TARGET, PROTOCOL_RESET_ADDRESS, 0);
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
PerchingArmResult PerchingArm::SendCommand(uint8_t target, int16_t address, int16_t data) {
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

// Print current joint and gripper states
void PerchingArm::PrintStates() {
  std::cout << "System state" << std::endl;
  std::cout << "- joint: ";
  switch (joint_state_) {
    case JOINT_STATE_UNKNOWN:          std::cout << "UNKNOWN";          break;
    case JOINT_STATE_STOPPED:          std::cout << "STOPPED";          break;
    case JOINT_STATE_DEPLOYING:        std::cout << "DEPLOYING";        break;
    case JOINT_STATE_MOVING_PANNING:   std::cout << "MOVING_PANNING";   break;
    case JOINT_STATE_MOVING_TILTING:   std::cout << "MOVING_TILTING";   break;
    case JOINT_STATE_PANNING:          std::cout << "PANNING";          break;
    case JOINT_STATE_TILTING:          std::cout << "TILTING";          break;
    case JOINT_STATE_STOWING_PANNING:  std::cout << "STOWING_PANNING";  break;
    case JOINT_STATE_STOWING_CLOSING:  std::cout << "STOWING_CLOSING";  break;
    case JOINT_STATE_STOWING_TILTING:  std::cout << "STOWING_TILTING";  break;
    case JOINT_STATE_STOWED:           std::cout << "STOWED";           break;
  }
  std::cout << std::endl;
  std::cout << "- gripper: ";
  switch (gripper_state_) {
    case GRIPPER_STATE_UNKNOWN:        std::cout << "UNKNOWN";          break;
    case GRIPPER_STATE_CLOSED:         std::cout << "CLOSED";           break;
    case GRIPPER_STATE_OPENING:        std::cout << "OPENING";          break;
    case GRIPPER_STATE_OPEN:           std::cout << "OPEN";             break;
    case GRIPPER_STATE_CLOSING:        std::cout << "CLOSING";          break;
    case GRIPPER_STATE_CALIBRATING:    std::cout << "CALIBRATING";      break;
    case GRIPPER_STATE_UNCALIBRATED:   std::cout << "UNCALIBRATED";     break;
  }
  std::cout << std::endl;
}

// Process a packet
void PerchingArm::Process(const uint8_t* buf, size_t len) {
  // Prevent a buffer overrun
  if (len < 32) return;
  feedback_.tilt.load         = (static_cast<int16_t>(buf[ 1]) << 8)      // FEEDBACK[0]
                              |  static_cast<int16_t>(buf[ 0]);
  feedback_.tilt.velocity     = (static_cast<int32_t>(buf[ 5]) << 24)     // FEEDBACK[1]
                              | (static_cast<int32_t>(buf[ 4]) << 16)
                              | (static_cast<int32_t>(buf[ 3]) << 8)
                              |  static_cast<int32_t>(buf[ 2]);
  feedback_.tilt.position     = (static_cast<int32_t>(buf[ 9]) << 24)     // FEEDBACK[2]
                              | (static_cast<int32_t>(buf[ 8]) << 16)
                              | (static_cast<int32_t>(buf[ 7]) << 8)
                              |  static_cast<int32_t>(buf[ 6]);
  feedback_.pan.load          = (static_cast<int16_t>(buf[11]) << 8)      // FEEDBACK[3]
                              |  static_cast<int16_t>(buf[10]);
  feedback_.pan.velocity      = (static_cast<int32_t>(buf[15]) << 24)     // FEEDBACK[4]
                              | (static_cast<int32_t>(buf[14]) << 16)
                              | (static_cast<int32_t>(buf[13]) << 8)
                              |  static_cast<int32_t>(buf[12]);
  feedback_.pan.position      = (static_cast<int32_t>(buf[19]) << 24)     // FEEDBACK[5]
                              | (static_cast<int32_t>(buf[18]) << 16)
                              | (static_cast<int32_t>(buf[17]) << 8)
                              |  static_cast<int32_t>(buf[16]);
  feedback_.gripper.position  = (static_cast<int16_t>(buf[21]) << 8)      // FEEDBACK[6]
                              |  static_cast<int16_t>(buf[20]);
  feedback_.gripper.load      = (static_cast<int16_t>(buf[23]) << 8)      // FEEDBACK[7]
                              |  static_cast<int16_t>(buf[22]);
  feedback_.current_11v       = (static_cast<int16_t>(buf[25]) << 8)      // FEEDBACK[8]
                              |  static_cast<int16_t>(buf[24]);
  feedback_.current_5v        = (static_cast<int16_t>(buf[27]) << 8)      // FEEDBACK[9]
                              |  static_cast<int16_t>(buf[26]);
  feedback_.loop_time         = (static_cast<int16_t>(buf[29]) << 8)      // FEEDBACK[10]
                              |  static_cast<int16_t>(buf[28]);
  feedback_.board_temp        = (static_cast<int16_t>(buf[31]) << 8)      // FEEDBACK[11]
                              |  static_cast<int16_t>(buf[30]);
  // Sign adjustments (these seem a little hacky to me, In Won)
  feedback_.pan.position -= 2048;
  feedback_.tilt.position -= 2048;
  // By default we have no event and 100% progress
  PerchingArmEvent event = EVENT_NONE;
  // State machine for arm actions
  switch (joint_state_) {
  // Initialization only: guess the state based on the arm position
  case JOINT_STATE_UNKNOWN:
    if (JointPositionSatisfied(PerchingArm::STOW_TILT, feedback_.tilt.position))
      joint_state_ = JOINT_STATE_STOWED;
    else
      joint_state_ = JOINT_STATE_STOPPED;
    break;
  // Awaiting command
  case JOINT_STATE_STOPPED:
    break;
  // Deploying arm
  case JOINT_STATE_DEPLOYING:
    if (JointPositionSatisfied(goal_tilt_, feedback_.tilt.position)) {
      joint_state_ = JOINT_STATE_STOPPED;
      event = EVENT_DEPLOY_COMPLETE;
    }
    break;
  // Moving arm: move phase
  case JOINT_STATE_MOVING_PANNING:
    if (JointPositionSatisfied(goal_pan_, feedback_.pan.position)) {
      SendCommand(PROTOCOL_TILT_TARGET, PROTOCOL_TILT_ADDRESS_POSITION, goal_tilt_);
      joint_state_ = JOINT_STATE_MOVING_TILTING;
    }
    break;
  // Moving arm: tilt phase
  case JOINT_STATE_MOVING_TILTING:
    if (JointPositionSatisfied(goal_tilt_, feedback_.tilt.position)) {
      joint_state_ = JOINT_STATE_STOPPED;
      event = EVENT_MOVE_COMPLETE;
    }
    break;
  // Panning arm
  case JOINT_STATE_PANNING:
    if (JointPositionSatisfied(goal_pan_, feedback_.pan.position)) {
      joint_state_ = JOINT_STATE_STOPPED;
      event = EVENT_PAN_COMPLETE;
    }
    break;
  // Tilting arm
  case JOINT_STATE_TILTING:
    if (JointPositionSatisfied(goal_tilt_, feedback_.tilt.position)) {
      joint_state_ = JOINT_STATE_STOPPED;
      event = EVENT_TILT_COMPLETE;
    }
    break;
  // Stowing: pan phase
  case JOINT_STATE_STOWING_PANNING:
    if (JointPositionSatisfied(goal_pan_, feedback_.pan.position)) {
      if (gripper_state_ == GRIPPER_STATE_OPEN) {
        SendCommand(PROTOCOL_GRIPPER_TARGET, PROTOCOL_GRIPPER_ADDRESS_CLOSE, 0);
        joint_state_ = JOINT_STATE_STOWING_CLOSING;
        gripper_state_ = GRIPPER_STATE_CLOSING;
      } else {
        SendCommand(PROTOCOL_TILT_TARGET, PROTOCOL_TILT_ADDRESS_POSITION, goal_tilt_);
        joint_state_ = JOINT_STATE_STOWING_TILTING;
      }
    }
    break;
  // Stowing: We are closing the gripper before stowing
  case JOINT_STATE_STOWING_CLOSING:
    if (feedback_.gripper.position == PerchingArm::GRIPPER_CLOSE_THRESHOLD) {
      SendCommand(PROTOCOL_TILT_TARGET, PROTOCOL_TILT_ADDRESS_POSITION, goal_tilt_);
      joint_state_ = JOINT_STATE_STOWING_TILTING;
      gripper_state_ = GRIPPER_STATE_CLOSED;
    }
    break;
  // Stowing: We are doing the final tilt to stow the arm
  case JOINT_STATE_STOWING_TILTING:
    if (JointPositionSatisfied(goal_tilt_, feedback_.tilt.position)) {
      joint_state_ = JOINT_STATE_STOWED;
      event = EVENT_STOW_COMPLETE;
    }
    break;
  // We don't need to do anything in this state
  case JOINT_STATE_STOWED:
    break;
  }
  // State machine for gripper actions
  switch (gripper_state_) {
  // Initialization only: guess the state based on the gripper position
  case GRIPPER_STATE_UNKNOWN:
    if (GripperPositionSatisfied(PerchingArm::GRIPPER_CLOSE_THRESHOLD, feedback_.gripper.position))
      gripper_state_ = GRIPPER_STATE_CLOSED;
    else
      gripper_state_ = GRIPPER_STATE_OPEN;
    break;
  // Close the gripper
  case GRIPPER_STATE_CLOSED:
  case GRIPPER_STATE_OPEN:
  case GRIPPER_STATE_UNCALIBRATED:
    break;
  // Gripper opening
  case GRIPPER_STATE_OPENING:
    if (GripperPositionSatisfied(PerchingArm::GRIPPER_OPEN_THRESHOLD, feedback_.gripper.position)) {
      gripper_state_ = GRIPPER_STATE_OPEN;
      event = EVENT_OPEN_COMPLETE;
    }
    break;
  // Gripper closing
  case GRIPPER_STATE_CLOSING:
    if (GripperPositionSatisfied(PerchingArm::GRIPPER_CLOSE_THRESHOLD, feedback_.gripper.position)) {
      gripper_state_ = GRIPPER_STATE_CLOSED;
      event = EVENT_CLOSE_COMPLETE;
    }
    break;
  // Gripper calibrating
  case GRIPPER_STATE_CALIBRATING:
    if (GripperPositionSatisfied(PerchingArm::GRIPPER_OPEN_THRESHOLD, feedback_.gripper.position)) {
      gripper_state_ = GRIPPER_STATE_CLOSED;
      event = EVENT_CALIBRATE_COMPLETE;
    }
    break;
  }
  // If needed, send out an event
  if (cb_event_) {
    // By default, progress is 100%
    float progress = 100.0f;
    // Progress calculation
    switch (joint_state_) {
    // Moving, panning, tilting
    case JOINT_STATE_MOVING_PANNING:
    case JOINT_STATE_MOVING_TILTING:
    case JOINT_STATE_PANNING:
    case JOINT_STATE_TILTING:
      progress = static_cast<float>(feedback_.pan.position - initial_pan_ + feedback_.tilt.position - initial_tilt_)
               / static_cast<float>(goal_pan_ - initial_pan_ + goal_tilt_ - initial_tilt_ )
               * 100.0f;
      break;
    // Deploying
    case JOINT_STATE_DEPLOYING:
      progress = static_cast<float>(feedback_.tilt.position - STOW_TILT)
               / static_cast<float>(DEPLOY_TILT - STOW_TILT)
               * 100.0f;
      break;
    // Stowing
    case JOINT_STATE_STOWING_PANNING:
    case JOINT_STATE_STOWING_CLOSING:
    case JOINT_STATE_STOWING_TILTING:
      progress = static_cast<float>(feedback_.pan.position - initial_pan_
                                  + feedback_.tilt.position - initial_tilt_
                                  + feedback_.gripper.position - initial_gripper_)
               / static_cast<float>(STOW_PAN - initial_pan_
                                  + STOW_TILT - initial_tilt_
                                  + GRIPPER_CLOSE_THRESHOLD - initial_gripper_)
               * 100.0f;
      break;
    // No valid calculation for progress
    case JOINT_STATE_UNKNOWN:
    case JOINT_STATE_STOPPED:
    case JOINT_STATE_STOWED:
    default:
      break;
    }
    // State machine for gripper actions
    switch (gripper_state_) {
    // Gripper opening
    case GRIPPER_STATE_OPENING:
      progress = static_cast<float>(feedback_.gripper.position - initial_gripper_)
               / static_cast<float>(GRIPPER_OPEN_THRESHOLD - initial_gripper_)
               * 100.0f;
      break;
    // Gripper closing or calibrating
    case GRIPPER_STATE_CALIBRATING:
    case GRIPPER_STATE_CLOSING:
      progress = static_cast<float>(feedback_.gripper.position - initial_gripper_)
               / static_cast<float>(GRIPPER_CLOSE_THRESHOLD - initial_gripper_)
               * 100.0f;
      break;
    //  No valid calculation for progress
    case GRIPPER_STATE_UNKNOWN:
    case GRIPPER_STATE_CLOSED:
    case GRIPPER_STATE_OPEN:
    case GRIPPER_STATE_UNCALIBRATED:
      break;
    }
    // Send the event to the callee
    cb_event_(event, progress);
  }
  // Send comprehensive feedback
  if (cb_feedback_)
    cb_feedback_(joint_state_, gripper_state_, feedback_);
}

// Callback with serial data
void PerchingArm::Read(const uint8_t* buf, size_t len) {
  for (size_t i = 0; i < len; i++)
    FiniteStateMachine(buf[i]);
}

// Asynchronous callback with serial data
void PerchingArm::Timeout(void) {
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

// Tick to position conversion with a tolerance
bool PerchingArm::JointPositionSatisfied(int16_t goal, int16_t ticks, float tol) {
  float goal_f = static_cast<float>(goal);
  float curr_f = MOTOR_POSITION_SCALE * static_cast<float>(ticks);
  // std::cout << fabs(goal_f - curr_f) << " : "  << goal_f << ", " << curr_f <<  std::endl;
  return (fabs(goal_f - curr_f) < tol);
}

// Tick to position conversion with a tolerance
bool PerchingArm::GripperPositionSatisfied(int16_t goal, int16_t ticks, float tol) {
  float goal_f = static_cast<float>(goal);
  float curr_f = GRIPPER_POSITION_SCALE * static_cast<float>(ticks);
  // std::cout << fabs(goal_f - curr_f) << " : "  << goal_f << ", " << curr_f <<  std::endl;
  return (fabs(goal_f - curr_f) < tol);
}

}  // namespace perching_arm
