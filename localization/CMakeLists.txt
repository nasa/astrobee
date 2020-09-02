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

add_subdirectory(camera)
add_subdirectory(imu_integration)
add_subdirectory(interest_point)
add_subdirectory(localization_common)
add_subdirectory(localization_measurements)
add_subdirectory(sparse_mapping)

if (USE_ROS)
  add_subdirectory(fake_localizer)
  add_subdirectory(graph_localizer)
  add_subdirectory(imu_augmentor)
  add_subdirectory(marker_tracking)
  add_subdirectory(lk_optical_flow)
  add_subdirectory(handrail_detect)
  add_subdirectory(localization_manager)
  add_subdirectory(localization_node)
  if (ENABLE_VIVE_SOLVER)
    add_subdirectory(vive_localization)
  endif()
endif (USE_ROS)

set(SPARSE_MAPPING_LIBRARIES ${SPARSE_MAPPING_LIBRARIES} PARENT_SCOPE)
set(SPARSE_MAPPING_INCLUDE_DIRS ${SPARSE_MAPPING_INCLUDE_DIRS} PARENT_SCOPE)

