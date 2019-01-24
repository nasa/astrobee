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
# The state of the localization system

# Header with timestamp
std_msgs/Header header

# Tee current state
int32 state                                 # Current state
int32 INITIALIZING                    = 0   # Waiting on dependencies
int32 DISABLED                        = 1   # Localization disabled
int32 LOCALIZING                      = 2   # Localization enabled
int32 SWITCH_WAITING_FOR_PIPELINE     = 3   # Waiting for pipeline to stabilize
int32 SWITCH_WAITING_FOR_FILTER       = 4   # Waiting for filter to stabilize
int32 BIAS_WAITING_FOR_FILTER         = 5   # Waiting for bias estimation
int32 RESET_WAITING_FOR_FILTER        = 6   # Waiting for EKF stability
int32 UNSTABLE                        = 7   # Fallback pipeline unstable

# A human readable version of the (event) -> [state] transition
string fsm_event
string fsm_state

# The current localization pipeline being used
ff_msgs/LocalizationPipeline pipeline
