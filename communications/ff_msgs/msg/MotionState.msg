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
# Locked topic that registers updates to the internal dock state

# Header with timestamp
std_msgs/Header header

# The state of the mobility subsystem
int32 state
int32 INITIALIZING        = 0
int32 WAITING_FOR_STOP    = 1
int32 WAITING             = 2
int32 IDLING              = 3
int32 STOPPING            = 4
int32 PREPPING            = 5
int32 BOOTSTRAPPING       = 6
int32 PLANNING            = 7
int32 VALIDATING          = 8
int32 PREPARING           = 9
int32 CONTROLLING         = 10
int32 REPLANNING          = 11
int32 REVALIDATING        = 12