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

#include "vive_dev_tracker.h"
#include "vive_data_imu.h"
#include "vive_data_light.h"
#include "vive_data_button.h"

// Process tracker packets of size 9 (TRACKER V2)
void vive_dev_tracker_light_9(tracker_t * tracker,
  const uint8_t const * buf, int32_t len) {
  static uint16_t sensor;
  static uint16_t length;
  static uint32_t timestamp;
  for (size_t i = 0; i < 9; i++ ) {
    sensor = *((uint8_t*)(&(buf[i*7+1])));
    length = *((uint16_t*)(&(buf[i*7+2])));
    timestamp = *((uint32_t*)(&(buf[i*7+4])));
    if (sensor > 0xfd)
      continue;
    vive_data_light(tracker, timestamp, sensor, length);
  }
}

// Process tracker packets of size 7 (WATCHMAN & TRACKER V1)
void vive_dev_tracker_light_7(tracker_t * tracker,
  const uint8_t const * buf, int32_t len) {
  static uint16_t sensor;
  static uint16_t length;
  static uint32_t timestamp;
  for (size_t i = 0; i < 7; i++ ) {
    sensor = *((uint16_t*)(&(buf[i*8+1])));      // 2 bytes
    length = *((uint16_t*)(&(buf[i*8+3])));      // 2 bytes
    timestamp = *((uint32_t*)(&(buf[i*8+5])));    // 4 bytes
    if (sensor > 0xfd)
      continue;
    vive_data_light(tracker, timestamp, sensor, length);
  }
}

// Process IMU data
void vive_dev_tracker_imu(tracker_t * tracker,
  const uint8_t const * buf, int32_t len) {
  // Accelerometer (moved from imu-frame to tracker-frame)
  int16_t acc[3], gyr[3];
  acc[0] = *((int16_t*)(buf+1));
  acc[1] = *((int16_t*)(buf+3));
  acc[2] = *((int16_t*)(buf+5));
  gyr[0] = *((int16_t*)(buf+7));
  gyr[1] = *((int16_t*)(buf+9));
  gyr[2] = *((int16_t*)(buf+11));
  // Timecode is wrapping milliseconds
  uint32_t timecode = *((uint32_t*)(buf+13));
  // Process the data
  vive_data_imu(tracker, timecode, acc, gyr);
}

// Process button data
void vive_dev_tracker_button(tracker_t * tracker,
  const uint8_t const * buf, int32_t len) {
    // Extract the button data
  uint32_t mask = *((uint32_t*)(buf+7));
  int16_t horizontal = *((int16_t*)(buf+20));
  int16_t vertical = *((int16_t*)(buf+22));
  uint16_t trigger = *((uint16_t*)(buf+26));
  // Process the data
  vive_data_button(tracker, mask, trigger, horizontal, vertical);
}
