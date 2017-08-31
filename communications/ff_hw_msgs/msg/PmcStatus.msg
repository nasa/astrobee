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
# Status of an individual PMC

uint8 motor_speed		# Motor speed. Unitless. 0-255
uint8 motor_current		# Current consumption. Unitless. 0-255
uint8 v6_current		# TODO Needs info from Prop
uint16 pressure			# Pressure. Unitless? 0-65535
uint8[] temperatures		# temperatures
uint8 status_1			# Status 1 (read ICD for codes)
uint8 status_2			# Status 2 (read ICD for codes)
uint8 command_id		# Command ID that is currently executed
