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

cmake_minimum_required(VERSION 3.0)
project(ff_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  trajectory_msgs
  sensor_msgs
)


# Generate messages in the 'msg' folder
add_message_files(
  FILES
  AccessControlStateStamped.msg
  AckCompletedStatus.msg
  AckStamped.msg
  AckStatus.msg
  AgentStateStamped.msg
  ArmGripperState.msg
  ArmJointState.msg
  ArmState.msg
  ArmStateStamped.msg
  AvailableRobots.msg
  CameraRegistration.msg
  CameraState.msg
  CameraStatesStamped.msg
  CommandArg.msg
  CommandConstants.msg
  CommandStamped.msg
  CommStateStamped.msg
  CompressedFile.msg
  CompressedFileAck.msg
  ControlCommand.msg
  ControlState.msg
  CpuState.msg
  CpuStateStamped.msg
  DataToDiskState.msg
  DataTopicsList.msg
  DepthLandmark.msg
  DepthLandmarks.msg
  DiskState.msg
  DiskStateStamped.msg
  DockState.msg
  EkfState.msg
  ExecState.msg
  FamCommand.msg
  Fault.msg
  FaultConfig.msg
  FaultData.msg
  FaultInfo.msg
  FaultState.msg
  Feature2d.msg
  Feature2dArray.msg
  FlightMode.msg
  GraphState.msg
  GuestScienceApk.msg
  GuestScienceCommand.msg
  GuestScienceConfig.msg
  GuestScienceData.msg
  GuestScienceState.msg
  Hazard.msg
  Heartbeat.msg
  JointSample.msg
  JointSampleStamped.msg
  LocalizationGraph.msg
  LocalizationPipeline.msg
  LocalizationState.msg
  MemState.msg
  MemStateStamped.msg
  MobilityState.msg
  MotionState.msg
  OpState.msg
  PerchState.msg
  Performance.msg
  PicoflexxIntermediateData.msg
  PlanStatusStamped.msg
  SaveSettings.msg
  Segment.msg
  SignalState.msg
  Status.msg
  TimeSyncStamped.msg
  VisualeyezCalibration.msg
  VisualeyezData.msg
  VisualeyezDataArray.msg
  VisualeyezFeedback.msg
  VisualeyezFeedbackArray.msg
  VisualLandmark.msg
  VisualLandmarks.msg
  Zone.msg
)

# Generate services in the 'srv' folder
add_service_files(
 FILES
  ConfigureCamera.srv
  EnableCamera.srv
  EnableRecording.srv
  GetFloat.srv
  GetMap.srv
  GetOccupancyMap.srv
  GetPipelines.srv
  GetZones.srv
  RegisterPlanner.srv
  SetBool.srv
  SetDataToDisk.srv
  SetEkfInput.srv
  SetFloat.srv
  SetInertia.srv
  SetRate.srv
  SetState.srv
  SetStreamingLights.srv
  SetZones.srv
  Trigger.srv
  UnloadLoadNodelet.srv
  VisualeyezConfig.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  Arm.action
  Control.action
  Dock.action
  Localization.action
  Motion.action
  Perch.action
  Plan.action
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs trajectory_msgs sensor_msgs std_msgs
)

catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs actionlib_msgs trajectory_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
