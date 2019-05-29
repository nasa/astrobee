# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

string tracker                      # Serial of tracker that produced the event
uint32 mask                         # Button type
uint32 MASK_TRIGGER_TOUCH    = 1    # Primary trigger touched
uint32 MASK_TRIGGER_CLICK    = 2    # Primary trigger clicked
uint32 MASK_GRIP_CLICK       = 4    # Side grip click
uint32 MASK_MENU_CLICK       = 8    # Menu button click
uint32 MASK_PAD_TOUCH        = 32   # Pad touched
uint32 MASK_PAD_CLICK        = 16   # Pad click
uint16 trigger_val                  # Current trigger value
int16 pad_x                         # Pad X value (horizontal)
int16 pad_y                         # Pad Y value (vertical)
