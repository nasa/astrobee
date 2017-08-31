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
# Fault info message is used in the fault config message to contain all the 
# information GDS needs to know about a fault

uint16 subsystem    # index into subsystem names array found in fault config msg

uint16 node         # index into node names array found in fault config msg

uint32 id           # id corresponding to the fault

bool warning        # whether the fault is a warning or not

string description  # A short description of why the fault occurred
