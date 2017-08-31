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
# This message describes the MOBILITY RESULT message

int32 response                                        #
int32 ALREADY_THERE                           =   4   # 
int32 CALCULATED                              =   3   # 
int32 SUCCESS                                 =   2   # 
int32 CANCELLED                               =   1   # 
int32 PREEMPTED                               =   0   # 
int32 COULD_NOT_FIND_CONTROL_ENABLE_SERVICE   =  -1   # 
int32 COULD_NOT_CALL_CONTROL_ENABLE_SERVICE   =  -2   # 
int32 ACTION_TIMEOUT_WAITING_FOR_ACTIVE       =  -3   # 
int32 ACTION_TIMEOUT_WAITING_FOR_RESPONSE     =  -4   # 
int32 ACTION_TIMEOUT_WAITING_FOR_DEADLINE     =  -5   # 
int32 BAD_STATE_TRANSITION                    =  -6   # 
int32 OBSTACLE_AND_REPLANNING_DISABLED        =  -7   # 
int32 OBSTACLE_AND_INSUFFICIENT_REPLAN_TIME   =  -8   # 
int32 UNAVOIDABLE_COLLISION_DETECTED          =  -9   # 
int32 NOT_YET_INITIALIZED                     = -10   # 
int32 WATCH_ACTION_DIED_UNEXPECTEDLY          = -11   #
int32 STOP_CALLED_WITHOUT_CONTROL_ENABLED     = -12   #
int32 IDLE_CALLED_WITHOUT_CONTROL_ENABLED     = -13   #
int32 CONTROL_ENABLE_SERVICE_LOST             = -14   #
int32 NO_FLIGHT_MODE_SPECIFIED                = -15   #
int32 UNEXPECTED_EMPTY_SEGMENT                = -16   #
int32 CONTROL_FAILED                          = -17   #
int32 SELECTED_PLANNER_DOES_NOT_EXIST         = -18   #
int32 PLANNER_FAILED                          = -19   #
int32 VALIDATE_FAILED                         = -20   #
int32 CANNOT_QUERY_ROBOT_POSE                 = -21   #
int32 NOT_ON_FIRST_POSE                       = -22   #
int32 COULD_NOT_RESAMPLE                      = -23   #

uint8 state                                           # State the system was in

string flight_mode                                    # The flight mode
ff_msgs/ControlState[] segment                        # Generated segment

ff_msgs/ControlProgress progress                      # Progress up until stop
