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

battery_loc=$1
present=$2

if [ "$battery_loc" = tl ]
then
  if [ "$present" = true ]
  then
    rosservice call /add_remove_battery '{voltage: 11.4, current: 0.1, charge: 2.8, capacity: 3.0, design_capacity: 3.4, power_supply_status: 2, power_supply_health: 1, power_supply_technology: 2, present: true, location: "TOP_LEFT", serial_number: "1423850-348645"}' 0.05 28.0 0.2
  else
    rosservice call /add_remove_battery '{voltage: 11.4, current: 0.1, charge: 2.8, capacity: 3.0, design_capacity: 3.4, power_supply_status: 2, power_supply_health: 1, power_supply_technology: 2, present: false, location: "TOP_LEFT", serial_number: "1423850-348645"}' 0.05 28.0 0.2
  fi
elif [ "$battery_loc" = tr ]
then
  if [ "$present" = true ]
  then
    rosservice call /add_remove_battery '{voltage: 11.6, current: 0.2, charge: 3.0, capacity: 3.2, design_capacity: 3.4, power_supply_status: 2, power_supply_health: 1, power_supply_technology: 2, present: true, location: "TOP_RIGHT", serial_number: "1423850-348646"}' 0.02 25.0 0.1
  else
    rosservice call /add_remove_battery '{voltage: 11.6, current: 0.2, charge: 3.0, capacity: 3.2, design_capacity: 3.4, power_supply_status: 2, power_supply_health: 1, power_supply_technology: 2, present: false, location: "TOP_RIGHT", serial_number: "1423850-348646"}' 0.02 25.0 0.1
  fi
elif [ "$battery_loc" = bl ]
then
  if [ "$present" = true ]
  then
    rosservice call /add_remove_battery '{voltage: 11.0, current: 0.4, charge: 3.1, capacity: 3.3, design_capacity: 3.4, power_supply_status: 2, power_supply_health: 1, power_supply_technology: 2, present: true, location: "BOTTOM_LEFT", serial_number: "1423850-348647"}' 0.01 32.0 0.5
  else
    rosservice call /add_remove_battery '{voltage: 11.0, current: 0.4, charge: 3.1, capacity: 3.3, design_capacity: 3.4, power_supply_status: 2, power_supply_health: 1, power_supply_technology: 2, present: false, location: "BOTTOM_LEFT", serial_number: "1423850-348647"}' 0.01 32.0 0.5
  fi
elif [ "$battery_loc" = br ]
then
  if [ "$present" = true ]
  then  
    rosservice call /add_remove_battery '{voltage: 12.0, current: 0.5, charge: 3.4, capacity: 3.4, design_capacity: 3.4, power_supply_status: 2, power_supply_health: 1, power_supply_technology: 2, present: true, location: "BOTTOM_RIGHT", serial_number: "1423850-348648"}' 0.1 26.0 0.1
  else
    rosservice call /add_remove_battery '{voltage: 12.0, current: 0.5, charge: 3.4, capacity: 3.4, design_capacity: 3.4, power_supply_status: 2, power_supply_health: 1, power_supply_technology: 2, present: false, location: "BOTTOM_RIGHT", serial_number: "1423850-348648"}' 0.1 26.0 0.1
  fi
fi
