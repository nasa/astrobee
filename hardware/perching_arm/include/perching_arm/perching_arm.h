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

// Result of calling a perching arm action
enum PerchingArmResult {
  RESULT_SUCCESS,             // Everything happened well
  RESULT_PORT_NOT_OPEN,       // Serial port could not be opened
  RESULT_COMMAND_REJECTED,    // Invalid command
  RESULT_INVALID_COMMAND,     // Invalid command
  RESULT_PORT_WRITE_FAILURE,  // Port not writeable
  RESULT_PORT_INIT_FAILURE,   // Port cannot be initialized
  RESULT_FIRMWARE_ERROR,      // Firmware error
  RESULT_RESPONSE_TIMEOUT,    // Response timeout
  RESULT_OUT_OF_BOUNDS        // Cannot reach the specified angle
};

// Raw data struct, which contains more info than required
struct PerchingArmRaw {
  // Individual joint state
  struct PerchingArmRawJoint {
    int16_t load;      // S: 2.690 [mA]
    int32_t velocity;  // S: 0.229 [RPM]
    int32_t position;  // S: 0.088 [deg]
  };
  // Gripper state
  struct PerchingArmRawGripper {
    int16_t load;      // S: 3.3 / (1024 * 0.525)          [A]
    int16_t position;  // CONFIRM
    int16_t maximum;   // Maximum size
  };
  PerchingArmRawJoint prox;
  PerchingArmRawJoint dist;
  PerchingArmRawGripper grip;
  int16_t current_11v;  // S: 3.3 / (1024 * 100 * 0.0075)   [A]
  int16_t current_5v;   // S: 3.3 / (1024 * 100 * 0.0100)   [A]
  int16_t board_temp;   // S: 1 / 100
  int16_t loop_time;    // CONFIGM
};

// Convenience declarations
typedef std::function<void(uint32_t)> PerchingArmSleepMsCallback;
typedef std::function<void(PerchingArmRaw const&)> PerchingArmRawDataCallback;

class PerchingArm {
 public:
  // Hard-coded limits for the various axes
  static constexpr int16_t PROX_POS_MIN = -120;
  static constexpr int16_t PROX_POS_MAX = 90;
  static constexpr int16_t PROX_VEL_MIN = 0;
  static constexpr int16_t PROX_VEL_MAX = 20;
  static constexpr int16_t DIST_POS_MIN = -90;
  static constexpr int16_t DIST_POS_MAX = 90;
  static constexpr int16_t DIST_VEL_MIN = 0;
  static constexpr int16_t DIST_VEL_MAX = 20;
  static constexpr int16_t GRIP_POS_MIN = -100;
  static constexpr int16_t GRIP_POS_MAX = 0;
  static constexpr int16_t GRIP_CALIBRATE = -100;

  // Limits for the virtual gripper joints (in degrees)
  // Min = closed, MAX = open, L = left, R = right, D = distal, P = proximal
  static constexpr int16_t GRIP_L_P_MIN = 20;
  static constexpr int16_t GRIP_L_P_MAX = 40;
  static constexpr int16_t GRIP_L_D_MIN = -70;
  static constexpr int16_t GRIP_L_D_MAX = -40;
  static constexpr int16_t GRIP_R_P_MIN = -20;
  static constexpr int16_t GRIP_R_P_MAX = -40;
  static constexpr int16_t GRIP_R_D_MIN = 70;
  static constexpr int16_t GRIP_R_D_MAX = 40;

  // Useful constants for deploying, stowing, panning, tiltint, etc
  static constexpr int16_t STOW_PROX = 90;
  static constexpr int16_t STOW_DIST = 0;
  static constexpr int16_t DEPLOY_PROX = -90;
  static constexpr int16_t DEPLOY_DIST = 0;
  static constexpr int16_t TILT_MIN = -120;
  static constexpr int16_t TILT_MAX = 0;
  static constexpr int16_t PAN_MIN = -90;
  static constexpr int16_t PAN_MAX = 90;

  // Scale factors used to convert raw bytes to firmware units (deg, rpm, perc)
  static constexpr double K_POSITION_DEG = 0.088;
  static constexpr double K_VELOCITY_RPM = 0.229;
  static constexpr double K_LOAD_GRIPPER_MA = 3.3 / (1024.0 * 0.525);
  static constexpr double K_LOAD_JOINT_MA = 2.690;
  static constexpr double K_CURRENT_5V = 3.3 / (1024.0);
  static constexpr double K_CURRENT_11V = 3.3 / (1024.0 * 0.75);
  static constexpr double K_TEMPERATURE_DEG = 1.0 / 100.0;
  static constexpr double K_LOOP_TIME_MS = 1.0;
  static constexpr double K_MOTOR_VOLTAGE = 11.0;

  // Constructor
  PerchingArm();

  // Connect to the arm
  PerchingArmResult Connect(std::string const& port, uint32_t baud,
                            PerchingArmSleepMsCallback cb_sleep_ms,
                            PerchingArmRawDataCallback cb_raw_data);

  // Disconnect from the arm
  void Disconnect();

  // Print a human-readable string from a result, and return success
  bool ResultToString(PerchingArmResult result, std::string& msg);

  // Calibrate the gripper (it will end up closed after completion)
  PerchingArmResult CalibrateGripper();

  // Set the gripper position (in percentage from 0 to 100)
  PerchingArmResult SetGripperPosition(int16_t perc);

  // Enable or disabled the proximal motor
  PerchingArmResult SetProximalEnabled(bool enabled);

  // Set the velocity of the proximal joint in rads / sec
  PerchingArmResult SetProximalVelocity(int16_t rpm);

  // Set the position of the proximal joint in rads
  PerchingArmResult SetProximalPosition(int16_t degrees);

  // Enable or disabled the distal motor
  PerchingArmResult SetDistalEnabled(bool enabled);

  // Set the velocity of the distal joint in rads / sec
  PerchingArmResult SetDistalVelocity(int16_t rpm);

  // Set the position of the distal joint in rads
  PerchingArmResult SetDistalPosition(int16_t degrees);

  // Open the gripper
  PerchingArmResult OpenGripper();

  // Close the gripper
  PerchingArmResult CloseGripper();

  // Reset the software
  PerchingArmResult SoftReset();

  // Reset the hardware
  PerchingArmResult HardReset();

  // Configure the arm
  PerchingArmResult SendCommand(uint8_t target, int16_t address, int16_t value);

 protected:
  // Protocol header constants
  static constexpr uint8_t PROTOCOL_HEADER_1 = 0xFF;
  static constexpr uint8_t PROTOCOL_HEADER_2 = 0xFF;

  // Protocol target constants
  static constexpr int16_t TARGET_PROXIMAL = 0;
  static constexpr int16_t TARGET_DISTAL = 1;
  static constexpr int16_t TARGET_POWER = 2;
  static constexpr int16_t TARGET_TORQUE = 3;
  static constexpr int16_t TARGET_GRIPPER = 4;
  static constexpr int16_t TARGET_RESET = 10;

  // Protocol address constants
  static constexpr int16_t ADDRESS_POSITION = 1;
  static constexpr int16_t ADDRESS_VELOCITY = 2;
  static constexpr int16_t ADDRESS_ENABLE = 64;
  static constexpr int16_t ADDRESS_RESET = 10;
  static constexpr int16_t ADDRESS_TORQUE_LIMIT = 34;
  static constexpr int16_t ADDRESS_GRIPPER_CALIBRATE = 51;
  static constexpr int16_t ADDRESS_GRIPPER_OPEN = 52;
  static constexpr int16_t ADDRESS_GRIPPER_CLOSE = 53;
  static constexpr int16_t ADDRESS_GRIPPER_SET = 54;
  static constexpr int16_t ADDRESS_GRIPPER_RESET = 55;

  // Protocol value constants
  static constexpr int16_t VALUE_POWER_DISABLE = 0;
  static constexpr int16_t VALUE_POWER_ENABLE = 1;

  // Finite states of protocol
  enum Protocol {
    PROTOCOL_WF_HEADER_1,  // See constant above
    PROTOCOL_WF_HEADER_2,  // See constant above
    PROTOCOL_WF_LENGTH,    // N
    PROTOCOL_WF_DATA,      // DATA
    PROTOCOL_WF_CHECKSUM   // CRC
  };

  // Asynchronous callback with serial data
  void Read(const uint8_t* buffer, size_t len);

  // Asynchronous callback with serial data
  void Timeout(void);

  // Asynchronous callback with serial data
  void Shutdown(void);

  // Process a valid packet
  void Process(const uint8_t* buf, size_t len);

  // Process a packet
  void FiniteStateMachine(uint8_t const& c);

  // Implementation-specific checksumming
  uint8_t Checksum(const uint8_t* buf, size_t len);

 private:
  serial::Serial serial_;                   // Serial port
  PerchingArmRawDataCallback cb_raw_data_;  // Feedback callback
  PerchingArmSleepMsCallback cb_sleep_ms_;  // Sleep callback
  PerchingArmRaw raw_;                      // Feedback data structure
  Protocol rx_state_;                       // Receive state
  uint8_t rx_buf_[MAX_PACKET_SIZE];         // Buffer
  uint8_t rx_ptr_;                          // Buffer pointer
  uint8_t rx_datalen_;                      // Data length
  bool telemetry_rx_;                       // Mark as received telemetry
};

}  // namespace perching_arm

#endif  // PERCHING_ARM_PERCHING_ARM_H_
