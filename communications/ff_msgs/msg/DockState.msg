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
# Response for Dock/Undock goals

# Header with timestamp
std_msgs/Header header

# Feedback
int8 state
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14
int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13
int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12
int8 INITIALIZING                       = 11
int8 UNKNOWN                            = 10
int8 DOCKING_MAX_STATE                  = 7
int8 DOCKING_SWITCHING_TO_ML_LOC        = 7
int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6
int8 DOCKING_SWITCHING_TO_AR_LOC        = 5
int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4
int8 DOCKING_CHECKING_ATTACHED          = 3
int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2
int8 DOCKING_SWITCHING_TO_NO_LOC        = 1
int8 DOCKED                             = 0
int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1
int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2
int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3
int8 UNDOCKED                           = -4
int8 UNDOCKING_MAX_STATE                = -4
