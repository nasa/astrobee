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

#include "vive_data_button.h"

// Called when a new button event occurs
void vive_data_button(tracker_t const * tracker,
  uint32_t mask, uint16_t trigger, int16_t horizontal, int16_t vertical) {
  button_t event;
  event.mask = mask;
  event.trigger = trigger;
  event.horizontal = horizontal;
  event.vertical = vertical;
  if (tracker->driver->button_cb && (event.mask || event.trigger))
    tracker->driver->button_cb(tracker, &event);
}
