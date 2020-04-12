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

#include "vive_data_light.h"
#include "vive_data_ootx.h"

// LIGHTCAP

// Get the acode from the
static int handle_acode(lightcap_data_t *lcd, int length) {
  double old_offset = lcd->global.acode_offset;
  double new_offset = (((length) + 250) % 500) - 250;
  lcd->global.acode_offset = old_offset * 0.9 + new_offset * 0.1;
  return (uint8_t)((length - 2750) / 500);
}

// Handle measuements
static void handle_measurements(tracker_t *tracker) {
  // Get the tracker-specific lightcap data
  lightcap_data_t *lcd = &tracker->lcd;
  unsigned int longest_pulse = 0;
  // unsigned int timestamp_of_longest_pulse = 0;
  for (int i = 0; i < MAX_NUM_SENSORS; i++) {
    if (lcd->sweep.sweep_len[i] > longest_pulse) {
      longest_pulse = lcd->sweep.sweep_len[i];
      // timestamp_of_longest_pulse = lcd->sweep.sweep_time[i];
    }
  }

  // Temporary data structures to hold results
  light_t event[MAX_NUM_SENSORS];

  // Get the sync pulse rising edge time, lighthouse and axis
  uint32_t st = lcd->per_sweep.activeSweepStartTime;
  uint8_t lh = lcd->per_sweep.activeLighthouse;
  uint8_t ax = lcd->per_sweep.activeAcode & 1;

  // Copy over the final data
  uint8_t num_sensors = 0;
  for (uint8_t i = 0; i < MAX_NUM_SENSORS; i++) {
    if (lcd->sweep.sweep_len[i] != 0) {
      event[num_sensors].sensor_id = i;
      event[num_sensors].timestamp = lcd->sweep.sweep_time[i] -
                                     lcd->per_sweep.activeSweepStartTime +
                                     lcd->sweep.sweep_len[i] / 2;
      event[num_sensors].length = lcd->sweep.sweep_len[i];
      num_sensors++;
    }
  }

  // Push off the measurement bundle ONLY when we have received
  // an OOTX from the current lighthouse and if we have data
  if (num_sensors > 0 && tracker->driver->light_cb)
    tracker->driver->light_cb(tracker, lh, ax, st, num_sensors, event);

  // Clear memory
  memset(&lcd->sweep, 0, sizeof(lightcap_sweep_data_t));
}

// Handle sync
static void handle_sync(tracker_t *tracker, uint32_t timecode, uint16_t sensor, uint16_t length) {
  // Get the tracker-specific lightcap data
  lightcap_data_t *lcd = &tracker->lcd;
  // Get the acode from the sendor treading
  int acode = handle_acode(lcd, length);
  // Process any cached measurements
  handle_measurements(tracker);
  // Calculate the time since last sweet
  int time_since_last_sync = (timecode - lcd->per_sweep.recent_sync_time);
  // Store up sync pulses so we can take the earliest starting time
  if (time_since_last_sync < 2400) {
    lcd->per_sweep.recent_sync_time = timecode;
    if (length >
        lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh]) {
      lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = length;
      lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = timecode;
      lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;
    }
  } else if (time_since_last_sync < 24000) {
    lcd->per_sweep.activeLighthouse = -1;
    lcd->per_sweep.recent_sync_time = timecode;
    lcd->per_sweep.current_lh = 1;
    lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = timecode;
    lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = length;
    lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;
  } else if (time_since_last_sync > 370000) {
    // Initialize here
    memset(&lcd->per_sweep, 0, sizeof(lcd->per_sweep));
    lcd->per_sweep.activeLighthouse = -1;
    for (uint8_t i = 0; i < MAX_NUM_LIGHTHOUSES; ++i)
      lcd->per_sweep.lh_acode[i] = -1;
    lcd->per_sweep.recent_sync_time = timecode;
    lcd->per_sweep.current_lh = 0;
    lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = timecode;
    lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = length;
    lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;
  }
  // Feed the lighthouse OOTX decoder with the data bit
  if (lcd->per_sweep.current_lh < MAX_NUM_LIGHTHOUSES)
    ootx_feed(tracker, lcd->per_sweep.current_lh, ((acode & 0x2) ? 1 : 0),
              timecode);
}

// Called to process sweep events
static void handle_sweep(tracker_t *tracker, uint32_t timecode, uint16_t sensor, uint16_t length) {
  // Get the tracker-specific lightcap data
  lightcap_data_t *lcd = &tracker->lcd;
  // Reset the active lighthouse, start time and acode
  lcd->per_sweep.activeLighthouse = -1;
  lcd->per_sweep.activeSweepStartTime = 0;
  lcd->per_sweep.activeAcode = 0;
  for (uint8_t i = 0; i < MAX_NUM_LIGHTHOUSES; ++i) {
    int acode = lcd->per_sweep.lh_acode[i];
    if ((acode >= 0) && !(acode >> 2 & 1)) {
      lcd->per_sweep.activeLighthouse = i;
      lcd->per_sweep.activeSweepStartTime = lcd->per_sweep.lh_start_time[i];
      lcd->per_sweep.activeAcode = acode;
    }
  }
  // Check that we have an active lighthouse
  if (lcd->per_sweep.activeLighthouse < 0)
    return;
  // Store the data
  if (lcd->sweep.sweep_len[sensor] < length) {
    lcd->sweep.sweep_len[sensor] = length;
    lcd->sweep.sweep_time[sensor] = timecode;
  }
}

void vive_data_light(tracker_t *tracker, uint32_t timecode, uint16_t sensor, uint16_t length) {
  if (sensor > MAX_NUM_SENSORS)
    return;
  if (length > 6750)
    return;
  if (length > 2750)
    handle_sync(tracker, timecode, sensor, length);
  else
    handle_sweep(tracker, timecode, sensor, length);
}
