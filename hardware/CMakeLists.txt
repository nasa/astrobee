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

add_subdirectory(i2c)
add_subdirectory(serial)
add_subdirectory(flashlight)
add_subdirectory(laser)
add_subdirectory(gpio)
if (USE_ROS)
  add_subdirectory(epson_imu)
  #add_subdirectory(flycapture)
  add_subdirectory(is_camera)
  add_subdirectory(eps_driver)
  add_subdirectory(pmc_actuator)
  add_subdirectory(perching_arm)
  add_subdirectory(speed_cam)
  add_subdirectory(smart_dock)
  add_subdirectory(fam_cmd_i2c)
  add_subdirectory(temp_monitor)
  if(ENABLE_PICOFLEXX)
    add_subdirectory(pico_driver)
  endif()
endif (USE_ROS)
