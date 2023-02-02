#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 3.5)
project(localization_common)

## Compile as C++14, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )


# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_common REQUIRED)
find_package(camera REQUIRED)
find_package(config_reader REQUIRED)
find_package(msg_conversions REQUIRED)
find_package(ff_msgs REQUIRED)

# Find GTSAM
find_package(GTSAM REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# Find OpenCV
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV331 REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME} SHARED
  src/averager.cc
  src/combined_nav_state.cc
  src/combined_nav_state_covariances.cc
  src/pose_interpolater.cc
  # src/rate_timer.cc
  # src/ros_timer.cc
  src/time.cc
  src/timer.cc
  src/test_utilities.cc
  src/utilities.cc
)
target_link_libraries(${PROJECT_NAME} gtsam ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp ff_msgs config_reader camera msg_conversions)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pose_interpolater
    test/test_pose_interpolater.cc
  )
  target_link_libraries(test_pose_interpolater
    ${PROJECT_NAME}  
  )

  ament_add_gtest(test_timestamped_set
    test/test_timestamped_set.cc
  )
  target_link_libraries(test_timestamped_set
    ${PROJECT_NAME}  
  )
  ament_add_gtest(test_utilities_localization_common
    test/test_utilities.cc
  )
  target_link_libraries(test_utilities_localization_common
    ${PROJECT_NAME}  
  )
endif()

#############
## Install ##
#############

ament_export_include_directories(include)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()
