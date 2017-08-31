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
# Command Message, loosely based off of the RAPID Command.idl

# Header with timestamp
std_msgs/Header header

# Command name
string cmd_name

# Unique identifier for command = unique counter + participant + timestamp
string cmd_id

# Source of the command, either operators, the system monitor or guest science
string cmd_src

# Origin of the command, ground for operators, astrobee for another astrobee,
# sys_monitor for fault responses, and guest_science for guest science
# commands
string cmd_origin

# Name of subsystem the command is going to (not used but kept to be consistant
# with the command idl)
string subsys_name

# Arguments for the command 
ff_msgs/CommandArg[] args
