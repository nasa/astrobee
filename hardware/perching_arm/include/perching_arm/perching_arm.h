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

#ifndef PERCHING_ARM_PERCHING_ARM_H_
#define PERCHING_ARM_PERCHING_ARM_H_

#include <serial/serial.h>

#include <functional>
#include <string>

#define MAX_PACKET_SIZE 64

/**
 * \ingroup hw
 */
namespace perching_arm {

enum PerchingArmJointState {
  JOINT_STATE_UNKNOWN,          // Unknown
  JOINT_STATE_STOPPED,          // Awaiting command
  JOINT_STATE_DEPLOYING,        // Deploying in progress
  JOINT_STATE_MOVING_PANNING,   // Move: panning              [substate]
  JOINT_STATE_MOVING_TILTING,   // Move: tilting              [substate]
  JOINT_STATE_PANNING,          // Panning                    [substate]
  JOINT_STATE_TILTING,          // Tilting                    [substate]
  JOINT_STATE_STOWING_PANNING,  // Stowing: panning to 0      [substate]
  JOINT_STATE_STOWING_CLOSING,  // Stowing: gripper closing   [substate]
  JOINT_STATE_STOWING_TILTING,  // Stowing: tilting in place  [substate]
  JOINT_STATE_STOWED            // Stowed
};

enum PerchingArmGripperState {
  GRIPPER_STATE_UNKNOWN,        // Unknown
  GRIPPER_STATE_CLOSED,         // Gripper closed
  GRIPPER_STATE_OPENING,        // Gripper opening
  GRIPPER_STATE_OPEN,           // Gripper open
  GRIPPER_STATE_CLOSING,        // Gripper closing
  GRIPPER_STATE_CALIBRATING,    // Gripper calibrating
  GRIPPER_STATE_UNCALIBRATED    // Gripper uncalibrated
};

enum PerchingArmResult {
  RESULT_SUCCESS,               // Everything happened well
  RESULT_PORT_NOT_OPEN,         // Serial port could not be opened
  RESULT_COMMAND_REJECTED,      // Invalid command
  RESULT_INVALID_COMMAND,       // Invalid command
  RESULT_PORT_WRITE_FAILURE,    // Port not writeable
  RESULT_PORT_INIT_FAILURE,     // Port cannot be initialized
  RESULT_FIRMWARE_ERROR,        // Firmware error
  RESULT_RESPONSE_TIMEOUT       // Response timeout
};

enum PerchingArmEvent {
  EVENT_NONE,                   // Progress update
  EVENT_PROGRESS,               // Progress update
  EVENT_BACK_DRIVE,             // Back drive detected
  EVENT_STOW_COMPLETE,          // Arm is now stowed
  EVENT_MOVE_COMPLETE,          // Arm has finished moving
  EVENT_PAN_COMPLETE,           // Arm has finished panning
  EVENT_TILT_COMPLETE,          // Arm has finished tilting
  EVENT_DEPLOY_COMPLETE,        // Arm is now deployed
  EVENT_OPEN_COMPLETE,          // Gripper is now open
  EVENT_CLOSE_COMPLETE,         // Gripper is now closed
  EVENT_CALIBRATE_COMPLETE,     // Gripper is now calibrated
  EVENT_ERROR                   // Error encountered
};

struct PerchingArmMotor {
  int16_t load;                   // S: 2.690 [mA]
  int32_t velocity;               // S: 0.229 [RPM]
  int32_t position;               // S: 0.088 [deg]
};

struct PerchingArmGripper {
  int16_t load;                   // S: 3.3 / (1024 * 0.525)          [A]
  int16_t position;               // CONFIRM
};

struct PerchingArmFeedback {
  PerchingArmMotor tilt;
  PerchingArmMotor pan;
  PerchingArmGripper gripper;
  int16_t current_11v;            // S: 3.3 / (1024 * 100 * 0.0075)   [A]
  int16_t current_5v;             // S: 3.3 / (1024 * 100 * 0.0100)   [A]
  int16_t board_temp;             // S: 1 / 100
  int16_t loop_time;              // CONFIGM
};

// Convenience declarations
typedef std::function<void(uint32_t)> PerchingArmSleepMsCallback;
typedef std::function<void(PerchingArmEvent, float)> PerchingArmEventCallback;
typedef std::function<void(PerchingArmJointState joint_state, PerchingArmGripperState gripper_state,
  PerchingArmFeedback const&)> PerchingArmFeedbackCallback;

class PerchingArm {
 public:
  // Static members
  static const int16_t GRIPPER_OPEN_THRESHOLD;
  static const int16_t GRIPPER_CLOSE_THRESHOLD;
  static const int16_t STOW_PAN;
  static const int16_t STOW_TILT;
  static const int16_t DEPLOY_PAN;
  static const int16_t DEPLOY_TILT;
  static const int16_t MIN_PAN;
  static const int16_t MAX_PAN;
  static const int16_t MIN_TILT;
  static const int16_t MAX_TILT;
  static const float MOTOR_POSITION_SCALE;
  static const float MOTOR_VELOCITY_SCALE;
  static const float MOTOR_LOAD_SCALE;
  static const float GRIPPER_POSITION_SCALE;
  static const float GRIPPER_LOAD_SCALE;
  static const float CURRENT_5V_SCALE;
  static const float CURRENT_11V_SCALE;
  static const float BOARD_TEMP_SCALE;
  static const float LOOP_TIME_SCALE;

  // Constructor
  PerchingArm();

  // Initialize the serial port
  PerchingArmResult Initialize(std::string const& port, uint32_t baud, PerchingArmSleepMsCallback cb_sleep_ms,
    PerchingArmEventCallback cb_event, PerchingArmFeedbackCallback cb_feedback);

  // Asynchronous actions

  // Open the gripper
  PerchingArmResult Open();

  // Close the gripper
  PerchingArmResult Close();

  // Close the gripper
  PerchingArmResult Calibrate();

  // Stow the arm
  PerchingArmResult Stow();

  // Stow the arm
  PerchingArmResult Deploy();

  // Pan/tilt the arm and return whether the command was accepted
  PerchingArmResult Move(int16_t pan, int16_t tilt);

  // Pan the arm and return whether the command was accepted
  PerchingArmResult Pan(int16_t pan);

  // Tilt the arm and return whether the command was accepted
  PerchingArmResult Tilt(int16_t tilt);

  // Instantaneous actions

  // Stop the current action
  PerchingArmResult Stop();

  // Reset the software
  PerchingArmResult SoftReset();

  // Reset the hardware
  PerchingArmResult HardReset();

  // Configure the arm
  PerchingArmResult SendCommand(uint8_t target, int16_t address, int16_t value);

  // Print current joint and gripper states
  void PrintStates();

 protected:
  enum Protocol {
    PROTOCOL_WF_HEADER_1,   // 0xFF
    PROTOCOL_WF_HEADER_2,   // OxFF
    PROTOCOL_WF_LENGTH,     // N
    PROTOCOL_WF_DATA,       // DATA
    PROTOCOL_WF_CHECKSUM    // CRC
  };

  // Asynchronous callback with serial data
  void Read(const uint8_t *buffer, size_t len);

  // Asynchronous callback with serial data
  void Timeout(void);

  // Process a valid packet
  void Process(const uint8_t* buf, size_t len);

  // Process a packet
  void FiniteStateMachine(uint8_t const& c);

  // Implementation-specific checksumming
  uint8_t Checksum(const uint8_t* buf, size_t len);

  // Tick to position conversion with a tolerance
  bool JointPositionSatisfied(int16_t goal, int16_t ticks, float tol = 1.0);

  // Tick to position conversion with a tolerance
  bool GripperPositionSatisfied(int16_t goal, int16_t ticks, float tol = 1.0);

 private:
  serial::Serial serial_;                     // Serial port
  PerchingArmJointState joint_state_;         // Joint state
  PerchingArmGripperState gripper_state_;     // Gripper state
  PerchingArmEventCallback cb_event_;         // Event callback
  PerchingArmFeedbackCallback cb_feedback_;   // Feedback callback
  PerchingArmSleepMsCallback cb_sleep_ms_;    // Sleep callback
  PerchingArmFeedback feedback_;              // Feedback data structure
  uint8_t rx_buf_[MAX_PACKET_SIZE];           // Buffer
  uint8_t rx_ptr_;                            // Buffer pointer
  uint8_t rx_datalen_;                        // Data length
  Protocol rx_state_;                         // Receive state
  int16_t goal_pan_;                          // Pan goal
  int16_t goal_tilt_;                         // Tilt goal
  int16_t initial_pan_;                       // Pan initial value
  int16_t initial_tilt_;                      // Tilt initial value
  int16_t initial_gripper_;                   // Gripper initial value
};

}  // namespace perching_arm

#endif  // PERCHING_ARM_PERCHING_ARM_H_
