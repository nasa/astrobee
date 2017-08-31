# Undock manually astrobee
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

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo $DIR

TOOLS_DIR=${DIR}
PLAN_DIR=${DIR}/../plans

rosservice call /pmc_actuator/enable true

rosservice call /ekf/set_input 0
sleep 5

${TOOLS_DIR}/run_plan.sh ${PLAN_DIR}/undockp4.fplan

rosservice call /dock/control '{ bay: 2 }'

