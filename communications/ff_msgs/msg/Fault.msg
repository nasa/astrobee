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
# Fault message is used to provide all the information about an occurring fault

uint8 NO_HB_GNC_WRAPPER                               = 14
uint8 NO_HB_LOCALIZATION_MANAGER                      = 21
uint8 NO_HB_CHOREOGRAPHER                             = 26
uint8 NO_HB_EXECUTIVE                                 = 27
uint8 NO_HB_ACCESS_CONTROL                            = 29
uint8 NO_HB_DDS_ROS_BRIDGE                            = 30
uint8 NO_HB_LLP_CPU_MONITOR                           = 31
uint8 NO_HB_MLP_CPU_MONITOR                           = 32
uint8 NO_HB_HLP_CPU_MONITOR                           = 33
uint8 NO_HB_LLP_DISK_MONITOR                          = 34
uint8 NO_HB_MLP_DISK_MONITOR                          = 35
uint8 NO_HB_HLP_DISK_MONITOR                          = 36
uint8 NO_HB_PLANNER                                   = 37
uint8 NO_HB_PLANNER_QP                                = 38
uint8 NO_HB_PLANNER_TRAPEZOIDAL                       = 39
uint8 NO_HB_MAPPER                                    = 40
uint8 NO_HB_SENTINEL                                  = 41
uint8 NO_HB_NAV_CAM_IMG_SAMPLER                       = 42
uint8 NO_HB_DOCK_CAM_IMG_SAMPLER                      = 43
uint8 NO_HB_FRAME_STORE                               = 44
uint8 LLP_CPU_LOAD_TOO_HIGH                           = 46
uint8 MLP_CPU_LOAD_TOO_HIGH                           = 47
uint8 HLP_CPU_LOAD_TOO_HIGH                           = 48
uint8 LLP_TEMP_TOO_HIGH                               = 52
uint8 MLP_TEMP_TOO_HIGH                               = 53
uint8 HLP_TEMP_TOO_HIGH                               = 54
uint8 OFF_TRAJECTORY                                  = 59

time time_of_fault        # Time when fault occurred

uint32 id                 # id specifying fault

string msg                # string specifying why the fault occurred

ff_msgs/FaultData[] data  # Data used for fault analysis
