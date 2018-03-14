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
int8 state
int8 INITIALIZING        = 0
int8 IDLE                = 1
int8 STOPPED             = 2
int8 IDLING              = 3
int8 STOPPING            = 4
int8 PREPPING            = 5
int8 BOOTSTRAPPING       = 6
int8 PLANNING            = 7
int8 VALIDATING          = 8
int8 PREPARING           = 9
int8 CONTROLLING         = 10
int8 REPLANNING          = 11
int8 REVALIDATING        = 12