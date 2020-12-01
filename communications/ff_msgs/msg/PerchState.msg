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

int8 RECOVERY_MOVING_TO_RECOVERY_POSE   = 18
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 17
int8 RECOVERY_STOWING_ARM               = 16
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 15
int8 RECOVERY_OPENING_GRIPPER           = 14
int8 INITIALIZING                       = 13
int8 UNKNOWN                            = 12
# Used to check the perching/unperching ranges
int8 PERCHING_MAX_STATE                 = 11
int8 PERCHING_SWITCHING_TO_HR_LOC       = 11
int8 PERCHING_MOVING_TO_APPROACH_POSE   = 10
int8 PERCHING_ENSURING_APPROACH_POSE    = 9
int8 PERCHING_DEPLOYING_ARM             = 8
int8 PERCHING_OPENING_GRIPPER           = 7
int8 PERCHING_MOVING_TO_COMPLETE_POSE   = 6
int8 PERCHING_CLOSING_GRIPPER           = 5
int8 PERCHING_CHECKING_ATTACHED         = 4
int8 PERCHING_WAITING_FOR_SPIN_DOWN     = 3
int8 PERCHING_SWITCHING_TO_PL_LOC       = 2
int8 PERCHING_STOPPING                  = 1
int8 PERCHED                            = 0
int8 UNPERCHED                          = -1
int8 UNPERCHING_SWITCHING_TO_HR_LOC     = -2
int8 UNPERCHING_WAITING_FOR_SPIN_UP     = -3
int8 UNPERCHING_OPENING_GRIPPER         = -4
int8 UNPERCHING_MOVING_TO_APPROACH_POSE = -5
int8 UNPERCHING_STOWING_ARM             = -6
int8 UNPERCHING_SWITCHING_TO_ML_LOC     = -7
# Used to check the perching/unperching ranges
int8 UNPERCHING_MAX_STATE               = -7

# A human readable version of the (event) -> [state] transition
string fsm_event
string fsm_state
