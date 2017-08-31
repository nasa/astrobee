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
# Execution States, based off of the enumeration constants in
# rapid::ext::astrobee::AgentState
#
# *MUST* be kept in sync with the DDS IDL file in astrobee_common

uint8 IDLE      = 0   # Process is idle
uint8 EXECUTING = 1   # Process is executing
uint8 PAUSED    = 2   # Process is paused
uint8 ERROR     = 3   # Process encountered an error

# Execution state
uint8 state
