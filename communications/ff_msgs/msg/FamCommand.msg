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
# Command sent from control to the FAM.

std_msgs/Header header # header with time stamp

# force and torque
geometry_msgs/Wrench wrench
# linear acceleration (wrench w/out estimated mass)
geometry_msgs/Vector3 accel
# angular accceleration (wrench w/out estimated mass)
geometry_msgs/Vector3 alpha

# status byte from GNC ICD
uint8 status

# position error
geometry_msgs/Vector3 position_error
# integrated position error
geometry_msgs/Vector3 position_error_integrated

# attitude error
geometry_msgs/Vector3 attitude_error
# integrated attitude error
geometry_msgs/Vector3 attitude_error_integrated
# magnitude of attitude error
float32 attitude_error_mag

# control mode from GNC ICD
uint8 control_mode

