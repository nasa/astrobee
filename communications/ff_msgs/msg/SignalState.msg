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
#
# Signal state which is based on what the Astrobee is doing. Should be used to
# figure out what should be displayed on the signal lights and touch screen.

# Header with timestamp
std_msgs/Header header

uint8 VIDEO_ON              = 0
uint8 VIDEO_OFF             = 1
uint8 SUCCESS               = 3
uint8 ENTER_HATCHWAY        = 4
uint8 UNDOCK                = 5
uint8 UNPERCH               = 6
uint8 MOTION_IMPAIRED       = 7
uint8 THRUST_FORWARD        = 8
uint8 THRUST_AFT            = 9
uint8 TURN_RIGHT            = 10
uint8 TURN_LEFT             = 11
uint8 TURN_UP               = 12
uint8 TURN_DOWN             = 13
uint8 CLEAR                 = 14
uint8 SLEEP                 = 15
uint8 WAKE                  = 16
uint8 STOP_ALL_LIGHTS       = 17
uint8 CHARGING              = 18

# Signal state
uint8 state
