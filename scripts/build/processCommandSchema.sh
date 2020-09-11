#!/bin/bash
#
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

SCHEMA=../../astrobee/commands/freeFlyerPlanSchema.json
CONFIG_DIR=../../astrobee/config
MSG_DIR=../../communications/ff_msgs/msg
IDL_DIR=../../communications/dds_msgs/idl
DOC_DIR=../../doc

cd "$( dirname "${BASH_SOURCE[0]}" )"

./genCommandConfigLua.py $SCHEMA $CONFIG_DIR/commands.config
./genRosMsgCommandConstants.py $SCHEMA $MSG_DIR/CommandConstants.msg
./genCommandConstantsIdl.py $SCHEMA $IDL_DIR/AstrobeeCommandConstants.idl
./genCommandDictionary.py $SCHEMA $DOC_DIR/AstrobeeCommandDictionary.html
