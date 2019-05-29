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

#ifndef HARDWARE_VIVE_SRC_VIVE_DRIVER_VIVE_DATA_LIGHT_H_
#define HARDWARE_VIVE_SRC_VIVE_DRIVER_VIVE_DATA_LIGHT_H_

#include <vive/vive.h>

// Process light data
void vive_data_light(tracker_t * tracker,
  uint32_t timecode, uint16_t sensor, uint16_t length);

#endif  // HARDWARE_VIVE_SRC_VIVE_DRIVER_VIVE_DATA_LIGHT_H_
