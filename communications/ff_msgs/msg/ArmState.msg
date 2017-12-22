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
# The state of the arm system

int32 state                         # Current state
int32 INITIALIZING        = 0       # Waiting on child services, actions, etc.
int32 UNKNOWN             = 1       # Waiting on feedback from driver
int32 STOWED              = 2       # The arm is stowed
int32 DEPLOYED            = 3       # The arm is deployed
int32 SETTING             = 4       # The gripper is being set to a value
int32 CALIBRATING         = 5       # The gripper is calibrating
int32 PANNING             = 6       # We are panning as part of a move
int32 TILTING             = 7       # We are tilting as part of a move
int32 STOWING_SETTING     = 8       # We are closing the gripper for stowing
int32 STOWING_PANNING     = 9       # We are panning to zero for stowing
int32 STOWING_TILTING     = 10      # We are tilting to zero for stowing
int32 DEPLOYING_PANNING   = 11      # We are panning to zero for stowing
int32 DEPLOYING_TILTING   = 12      # We are tilting to zero for stowing
