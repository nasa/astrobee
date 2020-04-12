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

#include <vive/vive.h>

// Interface implementations
#include "vive_usb.h"

// Initialize the driver
driver_t * vive_init() {
  // Create a new driver context
  driver_t * drv = malloc(sizeof(driver_t));
  if (drv == NULL)
    return NULL;
  // Make sure we are zeroed by default
  memset(drv, 0, sizeof(driver_t));
  // General constants
  drv->general.timebase_hz            = 48000000UL; // Ticks per second
  drv->general.timecenter_ticks       = 200000UL;   // Midpoint of sweep
  drv->general.pulsedist_max_ticks    = 500000UL;
  drv->general.pulselength_min_sync   = 2200UL;
  drv->general.pulse_in_clear_time    = 35000UL;
  drv->general.pulse_max_for_sweep    = 1800UL;
  drv->general.pulse_synctime_offset  = 20000UL;
  drv->general.pulse_synctime_slack   = 5000UL;
  // Initialize tracker
  if (vive_usb_init(drv) == 0) {
    printf("No devices found\n");
    free(drv);
    return NULL;
  }
  return drv;
}

// CALLBACKS

// Register a light callback function
void vive_install_light_fn(driver_t * drv,  light_cb_t fbp) {
  if (drv == NULL) return;
  if (fbp) drv->light_cb = fbp;
}

// Register an IMU callback function
void vive_install_imu_fn(driver_t * drv,  imu_cb_t fbp) {
  if (drv == NULL) return;
  if (fbp) drv->imu_cb = fbp;
}

// Register an button callback function
void vive_install_button_fn(driver_t * drv, button_cb_t fbp) {
  if (drv == NULL) return;
  if (fbp) drv->button_cb = fbp;
}

// Register an button callback function
void vive_install_tracker_fn(driver_t * drv, tracker_cb_t fbp) {
  if (drv == NULL) return;
  if (fbp) drv->tracker_cb = fbp;
}

// Register an button callback function
void vive_install_lighthouse_fn(driver_t * drv, lighthouse_cb_t fbp) {
  if (drv == NULL) return;
  if (fbp) drv->lighthouse_cb = fbp;
}

// GETTERS

// Get the general configuration data
general_t * vive_general(driver_t * drv) {
  if (!drv) return NULL;
  return &drv->general;
}

// Get the calibration data for the lighthouse with the given serial number
lighthouse_t * vive_lighthouse(driver_t * drv, const char* id) {
  if (!drv) return NULL;
  for (size_t i = 0; i < MAX_NUM_LIGHTHOUSES; i++) {
    if (strcmp(drv->lighthouses[i].serial, id) == 0)
      return &drv->lighthouses[i];
  }
  return NULL;
}

// Get the calibration data for a tracker with the given serial number
tracker_t * vive_tracker(driver_t * drv, const char* id) {
  if (!drv) return NULL;
  for (size_t i = 0; i < drv->num_trackers; i++) {
    if (strcmp(drv->trackers[i]->serial, id) == 0)
      return drv->trackers[i];
  }
  return NULL;
}

// Poll the driver for events
uint8_t vive_poll(driver_t * drv) {
  if (drv == NULL) return -1;
  // Push general and tracker config
  if (drv->pushed == 0) {
    for (size_t i = 0; i < drv->num_trackers; i++)
      if (drv->tracker_cb)
        drv->tracker_cb(drv->trackers[i]);
    drv->pushed = 1;
  }
  // Handle any USB events
  return libusb_handle_events(drv->usb);
}

// Close the driver and clean up memory
void vive_kill(driver_t * drv) {
  if (drv == NULL) return;
  for (size_t i = 0; i < drv->num_trackers; i++) {
    libusb_close(drv->trackers[i]->udev);
    free(drv->trackers[i]);
  }
  libusb_exit(drv->usb);
  free(drv);
}
