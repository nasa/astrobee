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
# The state of the perching system

# Header with timestamp
std_msgs/Header header

# Feedback
int8 state
int8 INITIALIZING                       = 0
int8 UNKNOWN                            = 1
int8 TESTING_SWITCHING_TO_HR_LOC        = 2
int8 TESTING_SWITCHING_TO_PL_LOC        = 3
int8 UNPERCHED                          = 4
int8 PERCHING_SWITCHING_TO_HR_LOC       = 5
int8 PERCHING_MOVING_TO_APPROACH_POSE   = 6
int8 PERCHING_DEPLOYING_ARM             = 7
int8 PERCHING_OPENING_GRIPPER           = 8
int8 PERCHING_CALIBRATING_GRIPPER       = 9
int8 PERCHING_INGRESSING                = 10
int8 PERCHING_CLOSING_GRIPPER           = 11
int8 PERCHING_CHECKING_ATTACHED         = 12
int8 PERCHING_SWITCHING_TO_PL_LOC       = 13
int8 PERCHED                            = 14
int8 UNPERCHING_RESETTING_PAN_TILT      = 15
int8 UNPERCHING_SWITCHING_TO_HR_LOC     = 16
int8 UNPERCHING_OPENING_GRIPPER         = 17
int8 UNPERCHING_CALIBRATING_GRIPPER     = 18
int8 UNPERCHING_EGRESSING               = 19
int8 UNPERCHING_CLOSING_GRIPPER         = 20
int8 UNPERCHING_STOWING_ARM             = 21
int8 UNPERCHING_SWITCHING_TO_ML_LOC     = 22