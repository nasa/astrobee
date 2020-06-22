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

#include <ff_serial/serial.h>

#include <iostream>
#include <functional>
#include <string>

#define MAX_PACKET_SIZE 64

#define TELEMETRY_PACKET_LEADIN_SIZE sizeof(host_arm_telemetry_leadin_t)

#define HOST_ARM_CMD_LEADIN_1 0xFF
#define HOST_ARM_CMD_LEADIN_2 0xFE

#define HOST_ARM_TELEMETRY_SW_VERSION_LEN \
  20  // bytes required to encode a 40char git hash
#define HOST_ARM_TELEMETRY_MAX_LOG_MSG_LEN 128  // field MAY be shorter

/**
 * \ingroup hw
 */
namespace perching_arm {

// CRC-8-CCITT: polynomal 0x07 (x^8 + x^2 + x + 1)
static const uint8_t host_arm_comm_crc8_table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3};

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
    int16_t load;      // unit: 0.1 mA
    int32_t velocity;  // unit: 0.1 rpm
    int32_t position;  // unit: 0.1 degree
  };
  // Gripper state
  struct PerchingArmRawGripper {
    int16_t load;      // unit: 6.138mA
    int16_t position;  // unit: ticks (open = 0, closed = -8000)
    int16_t maximum;   // unit: fixed at -8000
  };
  PerchingArmRawJoint prox;
  PerchingArmRawJoint dist;
  PerchingArmRawGripper grip;
  int16_t current_11v;  // S: 3.3 / (1024 * 100 * 0.0075)   [A]
  int16_t current_5v;   // S: 3.3 / (1024 * 100 * 0.0100)   [A]
  int16_t board_temp;   // unit: 0.1 Celsius
  int16_t loop_time;    // ???
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
  PerchingArmResult SetProximalEnabled();
  PerchingArmResult SetProximalDisabled();

  // Set the velocity of the proximal joint in rads / sec
  PerchingArmResult SetProximalVelocity(int16_t rpm);

  // Set the position of the proximal joint in rads
  PerchingArmResult SetProximalPosition(int16_t degrees);

  // Enable or disabled the distal motor
  PerchingArmResult SetDistalEnabled();
  PerchingArmResult SetDistalDisabled();

  // Set the velocity of the distal joint in rads / sec
  PerchingArmResult SetDistalVelocity(int16_t rpm);

  // Set the position of the distal joint in rads
  PerchingArmResult SetDistalPosition(int16_t degrees);

  // Enable the gripper
  PerchingArmResult EnableGripper();
  PerchingArmResult DisableGripper();

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

  void ProcessPacket(const uint8_t* buffer, size_t length, double curr_time);

 protected:
  // Protocol header constants
  static constexpr uint8_t PROTOCOL_HEADER_1 = 0xFF;
  static constexpr uint8_t PROTOCOL_HEADER_2 = 0xFE;

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
  static constexpr int16_t ADDRESS_ENABLE = 3;
  static constexpr int16_t ADDRESS_DISABLE = 4;
  static constexpr int16_t ADDRESS_RESET = 10;
  static constexpr int16_t ADDRESS_TORQUE_LIMIT = 34;
  static constexpr int16_t ADDRESS_GRIPPER_CALIBRATE = 51;
  static constexpr int16_t ADDRESS_GRIPPER_OPEN = 52;
  static constexpr int16_t ADDRESS_GRIPPER_CLOSE = 53;
  static constexpr int16_t ADDRESS_GRIPPER_SET = 54;
  static constexpr int16_t ADDRESS_GRIPPER_RESET = 55;
  static constexpr int16_t ADDRESS_GRIPPER_ENABLE = 101;
  static constexpr int16_t ADDRESS_GRIPPER_DISABLE = 100;

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
  void Process(const uint8_t* buf);

  // Process a packet
  void FiniteStateMachine(uint8_t const& c);

  // Implementation-specific checksumming
  uint8_t Checksum(const uint8_t* buf, size_t len);

  typedef struct {
    uint8_t leadin1;
    uint8_t leadin2;
  } __attribute__((packed)) host_arm_cmd_leadin_t;

  typedef struct {
    host_arm_cmd_leadin_t leadin;
    uint8_t length;  // number of bytes FOLLOWING the header including checksum
  } __attribute__((packed)) host_arm_cmd_header_t;

  typedef struct {
    host_arm_cmd_header_t header;
    uint8_t type;    // e.g. HOST_ARM_BASIC_CMD_...
    uint16_t index;  // optional (rolling) unique identifier for this packet
  } __attribute__((packed)) host_arm_cmd_prefix_t;

  typedef struct {
    host_arm_cmd_prefix_t prefix;
    uint16_t address;
    int16_t data;
  } __attribute__((packed)) host_arm_basic_cmd_t;

  typedef struct {
    uint8_t leadin1;
    uint8_t leadin2;
  } __attribute__((packed)) host_arm_telemetry_leadin_t;

  typedef struct {
    host_arm_telemetry_leadin_t leadin;
    uint8_t
        length;  // number of bytes in packet FOLLOWING this field incl checksum
  } __attribute__((packed)) host_arm_telemetry_header_t;

  // must be even number of bytes for alignment!
  typedef struct {
    host_arm_telemetry_header_t header;
    uint8_t type;  // e.g. HOST_ARM_TELEMETRY_...
    uint16_t time;
  } __attribute__((packed)) host_arm_telemetry_prefix_t;

  enum {
    HOST_ARM_TELEMETRY_NULL = 0,

    HOST_ARM_TELEMETRY_DC_GRIPPER_STATE = 1,
    HOST_ARM_TELEMETRY_SERVO_JOINT_STATE = 2,
    HOST_ARM_TELEMETRY_SERVO_JOINT_STATUS = 3,

    HOST_ARM_TELEMETRY_ACK = 62,
    HOST_ARM_TELEMETRY_PONG = 63,

    HOST_ARM_TELEMETRY_BOARD_STATE = 64,
    HOST_ARM_TELEMETRY_BOARD_INFO = 65,

    HOST_ARM_TELEMETRY_LOG_MSG = 126,
    HOST_ARM_TELEMETRY_ERROR_MSG = 127,

    HOST_ARM_TELEMETRY_HOST_STATS = 140,
    HOST_ARM_TELEMETRY_JOINT_STATS = 141,
  };

  enum {
    HOST_ARM_BASIC_CMD_NULL = 0,

    HOST_ARM_BASIC_CMD_TILT_JOINT = 3,
    HOST_ARM_BASIC_CMD_PAN_JOINT = 4,
    HOST_ARM_BASIC_CMD_ALL_JOINTS = 6,

    HOST_ARM_BASIC_CMD_RAW_MOTOR = 7,
    HOST_ARM_BASIC_CMD_DC_GRIPPER = 8,

    HOST_ARM_BASIC_CMD_POWER = 10,

    HOST_ARM_BASIC_CMD_PING = 63,

    HOST_ARM_BASIC_CMD_DUMP_HOST_STATS = 90,
    HOST_ARM_BASIC_CMD_DUMP_JOINT_STATS = 91,

    HOST_ARM_BASIC_CMD_RESET = 100,

    HOST_ARM_BASIC_CMD_DUMP_LOG = 126,

    HOST_ARM_BASIC_CMD_MAX = 127,
  };

  enum {
    HOST_ARM_DC_GRIPPER_FLAG_MOTOR_ENABLED = 0b00000001,    // aka DCM_EN
    HOST_ARM_DC_GRIPPER_FLAG_MOTOR_HW_STATUS = 0b00000010,  // aka DCM_EN_STATUS
    HOST_ARM_DC_GRIPPER_FLAG_CALIBRATING = 0b00000100,
    HOST_ARM_DC_GRIPPER_FLAG_CALIBRATED = 0b00001000,
    HOST_ARM_DC_GRIPPER_FLAG_OVERLOAD = 0b00010000,
  };

  typedef struct {
    host_arm_telemetry_prefix_t prefix;

    // encoder ticks
    int32_t gripper_tick;
    int32_t gripper_tick_goal;

    // in units of 6.138mA
    uint16_t avg_current;   // average over interval since last report
    uint16_t max_current;   // largest in interval since last report
    uint32_t ewma_current;  // long term average, scaled by 10000x

    // commanded duty cycle, -100 .. 100
    int16_t avg_pwm;  // average over interval since last report
    int16_t max_pwm;  // largest in interval since last report

    uint8_t motor_flags;  // bitfield of OR'd HOST_ARM_DC_GRIPPER_FLAG_*
  } __attribute__((packed)) host_arm_dc_gripper_state_telemetry_t;

  enum {
    // joint operating normally
    HOST_ARM_JOINT_BACKDRIVE_STATUS_NONE = 0,
    // joint is being externally backdriven
    HOST_ARM_JOINT_BACKDRIVE_STATUS_DETECTED = 1,
    // external disturbance ended, waiting to resume operation
    HOST_ARM_JOINT_BACKDRIVE_STATUS_ENDING = 2,
  };

  typedef struct {
    int16_t pwm;               // units of 0.1%
    int16_t current;           // units of 0.1mA
    int16_t velocity;          // units of 0.1rpm
    int16_t position;          // units of 0.1deg
    int16_t position_goal;     // units of 0.1deg
    uint8_t backdrive_status;  // HOST_ARM_JOINT_BACKDRIVE_STATUS_...
  } __attribute__((packed)) host_arm_servo_joint_state_telemetry_element_t;

  typedef struct {
    host_arm_telemetry_prefix_t prefix;
    host_arm_servo_joint_state_telemetry_element_t tilt_state;
    host_arm_servo_joint_state_telemetry_element_t pan_state;
  } __attribute__((packed)) host_arm_servo_joint_state_telemetry_t;

  enum {
    HOST_ARM_SERVO_ERROR_FLAG_OVERLOAD = 0b00100000,
    HOST_ARM_SERVO_ERROR_FLAG_ELEC_SHOCK = 0b00010000,
    HOST_ARM_SERVO_ERROR_FLAG_ENCODER_ERR = 0b00001000,
    HOST_ARM_SERVO_ERROR_FLAG_OVERTEMP = 0b00000100,
    HOST_ARM_SERVO_ERROR_FLAG_VOLTAGE_ERR = 0b00000001,
  };

  typedef struct {
    uint16_t error_flags;   // ORing of HOST_ARM_SERVO_ERROR_FLAG_...
    int16_t temperature;    // units of 0.1degC
    uint8_t input_voltage;  // units of 0.1V
  } __attribute__((packed)) host_arm_servo_joint_status_telemetry_element_t;

  typedef struct {
    host_arm_telemetry_prefix_t prefix;
    host_arm_servo_joint_status_telemetry_element_t tilt_status;
    host_arm_servo_joint_status_telemetry_element_t pan_status;
  } __attribute__((packed)) host_arm_servo_joint_status_telemetry_t;

  typedef struct {
    host_arm_telemetry_prefix_t prefix;
    uint16_t address;
    int16_t data;
  } __attribute__((packed)) host_arm_pong_telemetry_t;

  enum {
    HOST_ARM_ACK_STATUS_NULL = -128,
    HOST_ARM_ACK_STATUS_ERROR = -127,
    HOST_ARM_ACK_STATUS_BUSY = -3,
    HOST_ARM_ACK_STATUS_INVALID_ARG = -2,
    HOST_ARM_ACK_STATUS_UNIMPLEMENTED = -1,
    HOST_ARM_ACK_STATUS_PENDING = 0,  // command received, operation started
    HOST_ARM_ACK_STATUS_CONFIRMED =
        1,                         // operations without completion detection
    HOST_ARM_ACK_STATUS_DONE = 2,  // operations with completion detection
  };

  typedef struct {
    host_arm_telemetry_prefix_t prefix;
    uint16_t index;
    int8_t status;  // e.g. HOST_ARM_ACK_STATUS_...
  } __attribute__((packed)) host_arm_ack_telemetry_t;

  enum {
    //  trap conflict reset
    HOST_ARM_RESET_REASON_TRAP = 0b1000000000000000,  // dsPIC33EP TRAPR
    //  illegal opcode
    HOST_ARM_RESET_REASON_IOP = 0b0100000000000000,  // dsPIC33EP IOPUWR
    //  configuration mismatch
    HOST_ARM_RESET_REASON_CM = 0b0000001000000000,  // dsPIC33EP CM
    //  external reset
    HOST_ARM_RESET_REASON_EXT = 0b0000000010000000,  // dsPIC33EP EXTR (MCLR)
    //  software reset
    HOST_ARM_RESET_REASON_SW = 0b0000000001000000,  // dsPIC33EP SWR
    //  watchdog reset
    HOST_ARM_RESET_REASON_WD = 0b0000000000010000,  // dsPIC33EP WDTO
    //  brownout reset
    HOST_ARM_RESET_REASON_BRNOUT = 0b0000000000000010,  // dsPIC33EP BOR
    //  power-on reset
    HOST_ARM_RESET_REASON_PWRON = 0b0000000000000001,  // dsPIC33EP POR
  };

  enum {
    HOST_ARM_BOARD_FLAG_HAS_LOGMSG = 0b00000001,
  };

  typedef struct {
    host_arm_telemetry_prefix_t prefix;

    // units of 4.297mA
    uint16_t avg_v11_current;  // average over interval since last report
    uint16_t max_v11_current;  // largest in interval since last report

    // units of 3.223mA
    uint16_t avg_v06_current;  // average over interval since last report
    uint16_t max_v06_current;  // largest in interval since last report

    uint16_t temperature;  // units of 0.1degC

    // in units of 1/273437.5s (70MHz/256)
    uint16_t avg_tick_time;
    uint16_t max_tick_time;

    uint16_t reset_reason;  // ORing of HOST_ARM_RESET_REASON_...

    uint8_t board_flags;  // bitfield of OR'd HOST_ARM_BOARD_FLAG_...
  } __attribute__((packed)) host_arm_board_state_telemetry_t;

  typedef struct {
    host_arm_telemetry_prefix_t prefix;

    uint32_t protocol_version;  // local value of HOST_ARM_COMM_PROTOCOL_VERSION
    uint32_t build_time;        // unix time, in seconds
    uint8_t sw_version[HOST_ARM_TELEMETRY_SW_VERSION_LEN];
  } __attribute__((packed)) host_arm_board_info_telemetry_t;

  typedef struct {
    host_arm_telemetry_prefix_t prefix;

    // this field may be UP TO this size, but can be smaller
    char msg[HOST_ARM_TELEMETRY_MAX_LOG_MSG_LEN];  // including terminating '\0'
  } __attribute__((packed)) host_arm_log_msg_telemetry_t;

  typedef struct {
    host_arm_telemetry_prefix_t prefix;

    // this field may be UP TO this size, but can be smaller
    char msg[HOST_ARM_TELEMETRY_MAX_LOG_MSG_LEN];  // including terminating '\0'
  } __attribute__((packed)) host_arm_error_msg_telemetry_t;

  void ProcessGripperStateTelemetry(
      const host_arm_dc_gripper_state_telemetry_t* packet);

  void ProcessServoJointStateTelemetry(
      const host_arm_servo_joint_state_telemetry_t* packet);

  void ProcessServoJointStatusTelemetry(
      const host_arm_servo_joint_status_telemetry_t* packet);

  void ProcessBoardStateTelemetry(
      const host_arm_board_state_telemetry_t* packet);

  void ProcessBoardInfoTelemetry(const host_arm_board_info_telemetry_t* packet);

  void ProcessErrorMsgTelemetry(const host_arm_error_msg_telemetry_t* packet);

 private:
  ff_serial::Serial serial_;                   // Serial port
  PerchingArmRawDataCallback cb_raw_data_;  // Feedback callback
  PerchingArmSleepMsCallback cb_sleep_ms_;  // Sleep callback
  PerchingArmRaw raw_;                      // Feedback data structure
  Protocol rx_state_;                       // Receive state
  uint8_t rx_buf_[MAX_PACKET_SIZE];         // Buffer
  uint8_t rx_ptr_;                          // Buffer pointer
  uint8_t rx_datalen_;                      // Data length
  bool telemetry_rx_;                       // Mark as received telemetry
  uint16_t command_index_;                  // Rolling command index
  bool uncalibrated_ = true;
};

}  // namespace perching_arm

#endif  // PERCHING_ARM_PERCHING_ARM_H_
