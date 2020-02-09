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
int8 PERCHING_UPDATING_HR_POSE          = 24
int8 PERCHING_ENSURING_APPROACH_POSE    = 23
int8 PERCHING_CALIBRATING_GRIPPER       = 22
int8 UNPERCHING_ENABLING_SERVOS         = 21
int8 PERCHING_ENABLING_SERVOS           = 20
int8 PERCHING_STOPPING                  = 19
int8 UNPERCHING_SWITCHING_TO_ML_LOC     = 18
int8 UNPERCHING_MOVING_TO_PREP_POSE     = 17
int8 RECOVERY_MOVING_TO_RECOVERY_POSE   = 16
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15
int8 RECOVERY_STOWING_ARM               = 14
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 13
int8 RECOVERY_OPENING_GRIPPER           = 12
int8 INITIALIZING                       = 11
int8 UNKNOWN                            = 10
int8 PERCHING_MAX_STATE                 = 9
int8 PERCHING_SWITCHING_TO_HR_LOC       = 9
int8 PERCHING_MOVING_TO_APPROACH_POSE   = 8
int8 PERCHING_DEPLOYING_ARM             = 7
int8 PERCHING_OPENING_GRIPPER           = 6
int8 PERCHING_MOVING_TO_COMPLETE_POSE   = 5
int8 PERCHING_CLOSING_GRIPPER           = 4
int8 PERCHING_CHECKING_ATTACHED         = 3
int8 PERCHING_WAITING_FOR_SPIN_DOWN     = 2
int8 PERCHING_SWITCHING_TO_PL_LOC       = 1
int8 PERCHED                            = 0
int8 UNPERCHING_SWITCHING_TO_HR_LOC     = -1
int8 UNPERCHING_WAITING_FOR_SPIN_UP     = -2
int8 UNPERCHING_OPENING_GRIPPER         = -3
int8 UNPERCHING_MOVING_TO_APPROACH_POSE = -4
int8 UNPERCHING_STOWING_ARM             = -5
int8 UNPERCHED                          = -6
int8 UNPERCHING_MAX_STATE               = -6

# A human readable version of the (event) -> [state] transition
string fsm_event
string fsm_state
