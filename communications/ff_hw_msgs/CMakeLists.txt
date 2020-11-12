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

cmake_minimum_required(VERSION 2.8.3)
project(ff_hw_msgs)

find_package(catkin2 REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArmCommand.msg
  ArmState.msg
  ConfigureLED.msg
  ConfigureLEDGroup.msg
  EpsBatteryLocation.msg
  EpsDockStateStamped.msg
  EpsHousekeeping.msg
  EpsHousekeepingValue.msg
  EpsPowerState.msg
  EpsPowerStateValue.msg
  OpticalFlow.msg
  PmcCommand.msg
  PmcGoal.msg
  PmcState.msg
  PmcStatus.msg
  PmcTelemetry.msg
  ViveButton.msg
  ViveLight.msg
  ViveLighthouse.msg
  ViveLighthouses.msg
  ViveMotor.msg
  VivePulse.msg
  ViveSensor.msg
  ViveTracker.msg
  ViveTrackers.msg
)

# Generate services in the 'srv' folder
add_service_files(
 FILES
  CalibrateGripper.srv
  ClearTerminate.srv
  ConfigureAdvancedPower.srv
  ConfigurePayloadPower.srv
  ConfigureSystemLeds.srv
  GetBatteryStatus.srv
  GetBoardInfo.srv
  Reset.srv
  RingBuzzer.srv
  SetEnabled.srv
  SetFlashlight.srv
  SetJointMaxVelocity.srv
  SetLedMatrix.srv
  Undock.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)