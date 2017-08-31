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
# The result of a call to a choreographer action

int32 value

# Expected return values (not errors)
int32 OK                                =  3    # Validation / planning passed
int32 APPLIED                           =  2    # Success, and control applied
int32 CALCULATED                        =  1    # Success, and no control applied
int32 PREEMPTED                         =  0    # Preempted

# Planner problems
int32 NO_TRAJECTORY_FOUND               = -101  # Planner could not find trajectory
int32 STALE_PLAN                        = -102  # Callback without state

# Choreographer problems
int32 INSUFFICIENT_SETPOINTS            = -201  # Insufficient number of setpoints
int32 TRANSFORMS_UNDEFINED              = -202  # Transformations undefined
int32 VELOCITIES_UNDEFINED              = -203  # Velocities undefined
int32 ACCLERATIONS_UNDEFINED            = -204  # Accelerations undefined
int32 TIMEOUT_BEFORE_STATION            = -205  # Timeout before station was reached
int32 BAD_INITIAL_STATE                 = -206  # Bad initial state (not equal to current state)
int32 LAST_SETPOINT_NOT_STATIC          = -207  # The last setpoint is not static
int32 TIME_RUNS_BACKWARDS               = -208  # Time runs backwards in the plan
int32 GRANITE_Z                         = -209  # Granite mode and Z varies
int32 GRANITE_ROLL                      = -210  # Granite mode and roll varies
int32 GRANITE_PITCH                     = -211  # Granite mode and pitch varies
int32 HAZARD_DETECTED                   = -212  # Hazard detected within stop horizon
int32 PROBLEM_REPLANNING                = -213  # Could not replan around a hazard

# Validation errors
int32 INVALID_KINEMATICS_VEL_LIN_X      = -301
int32 INVALID_KINEMATICS_POS_LIN_X      = -302
int32 INVALID_KINEMATICS_VEL_LIN_Y      = -303
int32 INVALID_KINEMATICS_POS_LIN_Y      = -304
int32 INVALID_KINEMATICS_VEL_LIN_Z      = -305
int32 INVALID_KINEMATICS_POS_LIN_Z      = -306
int32 INVALID_KINEMATICS_VEL_ROT_X      = -307
int32 INVALID_KINEMATICS_POS_ROT_X      = -308
int32 INVALID_KINEMATICS_VEL_ROT_Y      = -309
int32 INVALID_KINEMATICS_POS_ROT_Y      = -310
int32 INVALID_KINEMATICS_VEL_ROT_Z      = -311
int32 INVALID_KINEMATICS_POS_ROT_Z      = -312
int32 INVALID_LIMIT_VEL_LIN_X           = -313
int32 INVALID_LIMIT_ACC_LIN_X           = -314
int32 INVALID_LIMIT_VEL_LIN_Y           = -315
int32 INVALID_LIMIT_ACC_LIN_Y           = -316
int32 INVALID_LIMIT_VEL_LIN_Z           = -317
int32 INVALID_LIMIT_ACC_LIN_Z           = -318
int32 INVALID_LIMIT_VEL_ROT_X           = -319
int32 INVALID_LIMIT_ACC_ROT_X           = -320
int32 INVALID_LIMIT_VEL_ROT_Y           = -321
int32 INVALID_LIMIT_ACC_ROT_Y           = -322
int32 INVALID_LIMIT_VEL_ROT_Z           = -323
int32 INVALID_LIMIT_ACC_ROT_Z           = -324

# Tolerance problems
int32 TOLERANCE_FAILURE_POS_LIN_X       = -401
int32 TOLERANCE_FAILURE_POS_LIN_Y       = -402
int32 TOLERANCE_FAILURE_POS_LIN_Z       = -403
int32 TOLERANCE_FAILURE_POS_ROT_X       = -404
int32 TOLERANCE_FAILURE_POS_ROT_Y       = -405
int32 TOLERANCE_FAILURE_POS_ROT_Z       = -406
int32 TOLERANCE_FAILURE_VEL_LIN_X       = -407
int32 TOLERANCE_FAILURE_VEL_LIN_Y       = -408
int32 TOLERANCE_FAILURE_VEL_LIN_Z       = -409
int32 TOLERANCE_FAILURE_VEL_ROT_X       = -410
int32 TOLERANCE_FAILURE_VEL_ROT_Y       = -411
int32 TOLERANCE_FAILURE_VEL_ROT_Z       = -412
