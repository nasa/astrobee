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
 *
 * Based off libsurvive: https://github.com/cnlohr/libsurvive
 */

#ifndef VIVE_VIVE_H_
#define VIVE_VIVE_H_

#include <libusb-1.0/libusb.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define MAX_PACKET_LEN          64
#define PREAMBLE_LENGTH         17

#define MAX_NUM_LIGHTHOUSES     2
#define MAX_NUM_TRACKERS        128
#define MAX_NUM_SENSORS         32
#define MAX_SERIAL_LENGTH       32
#define USB_INT_BUFF_LENGTH     64

#define USB_VEND_HTC            0x28de
#define USB_PROD_TRACKER_V1     0x2022
#define USB_PROD_TRACKER_V2     0x2300
#define USB_PROD_CONTROLLER_V1  0x2012
#define USB_PROD_WATCHMAN       0x2101

#define USB_ENDPOINT_WATCHMAN   0x81
#define USB_ENDPOINT_V1_GENERAL 0x81
#define USB_ENDPOINT_V1_LIGHT   0x82
#define USB_ENDPOINT_V1_BUTTONS 0x83
#define USB_ENDPOINT_V2_GENERAL 0x81
#define USB_ENDPOINT_V2_LIGHT   0x83
#define USB_ENDPOINT_V2_BUTTONS 0x84
#define MAX_ENDPOINTS           3

#define DEFAULT_ACC_SCALE       (float)(9.80665/4096.0)
#define DEFAULT_GYR_SCALE       (float)((1./32.768)*(3.14159/180.));

// Richer debug info
// #define DEBUG_PRINTF(f_, ...) printf((f_), ##__VA_ARGS__)
#define DEBUG_PRINTF(f_, ...)

// Forward declaration of driver context
typedef struct Driver driver_t;
typedef struct Tracker tracker_t;
typedef struct Lighthouse lighthouse_t;

// Extrinsics axes
typedef enum Data {
  DATA_IMU, DATA_LIGHT_7, DATA_LIGHT_9, DATA_BUTTONS, DATA_WATCHMAN
} data_t;

// Button types
typedef enum Event {
  EVENT_TRIGGER    = (1<<8),
  EVENT_GRIP       = (1<<10),
  EVENT_MENU       = (1<<20),
  EVENT_PAD_CLICK  = (1<<26),
  EVENT_PAD_TOUCH  = (1<<28)
} event_t;

// Interrupt buffer for an endpoint
typedef struct Endpoint {
  tracker_t *tracker;
  data_t type;
  struct libusb_transfer *tx;
  uint8_t buffer[USB_INT_BUFF_LENGTH];
} endpoint_t;

// Calibration for a given tracker
typedef struct Calibration {
  uint32_t timestamp;                       // Time of last update
  uint8_t num_channels;                     // Number of photodiodes (PDs)
  uint8_t channels[MAX_NUM_SENSORS];        // Channel assignment for PDs
  float positions[MAX_NUM_SENSORS][3];      // PD positions
  float normals[MAX_NUM_SENSORS][3];        // PD normals
  float acc_bias[3];                        // Acceleromater bias
  float acc_scale[3];                       // Accelerometer scale
  float gyr_bias[3];                        // Gyro bias
  float gyr_scale[3];                       // Gyro scale
  // Transforms below are (wx, qy, qz, qw, X, Y, Z)
  float imu_transform[7];                   // Tracker -> IMU trasform
  float head_transform[7];                  // Tracker -> Head transform
} calibration_t;

// OOTX state machine for each lighthouse
typedef enum {PREAMBLE, LENGTH, PAYLOAD, CHECKSUM} state_t;
typedef struct OOTX {
  state_t state;                    // Current RX state
  uint8_t data[MAX_PACKET_LEN];   // Data buffer
  uint16_t length;                // Length in bytes
  uint8_t pad;                    // Padding length in bytes : 0 or 1
  uint16_t pos;                   // Bit position
  uint16_t syn;                   // Sync bit counter
  uint32_t crc;                   // CRC32 Checksum
  uint8_t preamble;               // Preamble
  uint32_t lasttime;              // Last sync time
  lighthouse_t *lighthouse;       // Lighthouse reference...
} ootx_t;

typedef struct LightcapSweepData {
  uint32_t sweep_time[MAX_NUM_SENSORS];
  uint16_t sweep_len[MAX_NUM_SENSORS];
} lightcap_sweep_data_t;

typedef struct PerSweepData {
  int recent_sync_time;
  int activeSweepStartTime;
  int activeLighthouse;
  int activeAcode;
  int lh_start_time[MAX_NUM_LIGHTHOUSES];
  int lh_max_pulse_length[MAX_NUM_LIGHTHOUSES];
  int8_t lh_acode[MAX_NUM_LIGHTHOUSES];
  int current_lh;
} per_sweep_data_t;

typedef struct GlobalData {
  double acode_offset;
} global_data_t;

typedef struct LightcapData {
  lightcap_sweep_data_t sweep;
  per_sweep_data_t per_sweep;
  global_data_t global;
} lightcap_data_t;

// Information about a tracked device
typedef struct Tracker {
  uint16_t type;                            // Tracker type
  driver_t * driver;                        // Parent driver
  struct libusb_device_handle * udev;       // Udev handle
  char serial[MAX_SERIAL_LENGTH];           // Serial number
  endpoint_t endpoints[MAX_ENDPOINTS];      // USB endpoints
  calibration_t cal;                        // Calibration data
  lightcap_data_t lcd;                      // Lightcap data
  ootx_t ootx[MAX_NUM_LIGHTHOUSES];         // OOTX data
  uint8_t charge;                           // Current charge
  uint8_t ischarging:1;                     // Charging?
  uint8_t ison:1;                           // Turned on?
  uint8_t axis[3];                          // Gravitational axis
  uint8_t buttonmask;                       // Buttom mask
  uint32_t timecode;                        // Timecode of last update
} tracker_t;

// Motor information
typedef enum Axis { MOTOR_AXIS0, MOTOR_AXIS1, MAX_NUM_MOTORS } axis_t;

// Motor data structure
typedef struct Motor {
  float phase;
  float tilt;
  float gibphase;
  float gibmag;
  float curve;
} motor_t;

// Lighthouse information
typedef struct Lighthouse {
  uint32_t timestamp;               // Time of last update (0 = invalud)
  uint8_t id;                       // ID of this lighthouse
  uint16_t fw_version;              // Firmware version
  char serial[MAX_SERIAL_LENGTH];   // Unique serial number
  motor_t motors[MAX_NUM_MOTORS];   // Motor calibration data
  float accel[3];                   // acceleration vector
  uint8_t sys_unlock_count;         // Lowest 8 bits of desynchronization
  uint8_t hw_version;               // Hardware version
  uint8_t mode_current;             // Current mode (default: 0=A, 1=B, 2=C)
  uint8_t sys_faults;               // "fault detect flags" (should be 0)
} lighthouse_t;

// General configuration
typedef struct General {
  int32_t timebase_hz;              // 48,000,000 (checked)
  int32_t timecenter_ticks;         // 200,000    (checked)  (2x = sweep length)
  int32_t pulsedist_max_ticks;      // 500,000    (guessed)
  int32_t pulselength_min_sync;     // 2,200      (guessed)
  int32_t pulse_in_clear_time;      // 35,000     (guessed)
  int32_t pulse_max_for_sweep;      // 1,800      (guessed)
  int32_t pulse_synctime_offset;    // 20,000     (guessed)
  int32_t pulse_synctime_slack;     // 5,000      (guessed)
} general_t;

typedef struct Light {
  uint32_t timestamp;
  uint8_t sensor_id;
  uint16_t length;
} light_t;

typedef struct IMU {
  uint32_t timestamp;
  int16_t acc[3];
  int16_t gyr[3];
} imu_t;

typedef struct Button {
  uint32_t mask;
  int16_t trigger;
  int16_t horizontal;
  int16_t vertical;
} button_t;

// Callbacks
typedef void (*light_cb_t)(tracker_t const * tracker, uint8_t lh, uint8_t axis,
  uint32_t synctime, uint16_t num_sensors, light_t const * measurement);
typedef void (*imu_cb_t)(tracker_t const * tracker, imu_t const * measurement);
typedef void (*button_cb_t)(tracker_t const * tracker, button_t const * measurement);
typedef void (*tracker_cb_t)(tracker_t const * tracker);
typedef void (*lighthouse_cb_t)(lighthouse_t const * lighthouse);

// Driver context
typedef struct Driver {
  struct libusb_context* usb;
  uint16_t num_trackers;
  tracker_t *trackers[MAX_NUM_TRACKERS];
  light_cb_t light_cb;            // Called when new light data arrives
  imu_cb_t imu_cb;                // Called when new IMU data arrives
  button_cb_t button_cb;          // Called when new button data arrives
  tracker_cb_t tracker_cb;        // Called when tracker cal info is ready
  lighthouse_cb_t lighthouse_cb;  // Called when lighthouse cal info is ready
  lighthouse_t lighthouses[MAX_NUM_LIGHTHOUSES];
  general_t general;              // General configuration
  uint8_t pushed;                 // Have we pushed thie tracker/general config
} driver_t;

// Initialize the driver
driver_t * vive_init();

// Register a light callback function
void vive_install_light_fn(driver_t * drv, light_cb_t fbp);

// Register an IMU callback function
void vive_install_imu_fn(driver_t * drv, imu_cb_t fbp);

// Register a button callback function
void vive_install_button_fn(driver_t *  drv, button_cb_t fbp);

// Register a tracker callback function
void vive_install_tracker_fn(driver_t * drv, tracker_cb_t fbp);

// Register a lighthouse callback function
void vive_install_lighthouse_fn(driver_t * drv, lighthouse_cb_t fbp);

// Get the general configuration data
general_t * vive_general(driver_t * drv);

// Get the calibration data for the lighthouse with the given serial number
lighthouse_t  * vive_lighthouse(driver_t * drv, const char* id);

// Get the calibration data for a tracker with the given serial number
tracker_t * vive_tracker(driver_t * tracker, const char* id);

// Poll the driver for events
uint8_t vive_poll(driver_t * drv);

// Close the driver and clean up memory
void vive_kill(driver_t * drv);

#endif  // VIVE_VIVE_H_
