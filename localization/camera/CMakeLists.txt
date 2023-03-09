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

cmake_minimum_required(VERSION 3.5)
project(camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

# System dependencies are found with CMake's conventions
# find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)

# Find OpenCV
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV331 REQUIRED)


# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_common REQUIRED)
find_package(config_reader REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(camera
  src/camera_model.cc
  src/camera_params.cc
)
ament_target_dependencies(camera rclcpp ff_common config_reader)
ament_export_targets(camera HAS_LIBRARY_TARGET)
target_link_libraries(camera ${EIGEN_LIBRARIES} ${OpenCV_LIBRARIES})

# Declare a C++ executable: undistort_image
add_executable(undistort_image tools/undistort_image.cc)
target_link_libraries(undistort_image
  camera gflags glog ${Boost_LIBRARIES})
ament_target_dependencies(undistort_image rclcpp ff_common)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # define TEST_DIR so that tests can find their test data
  get_filename_component(ABSOLUTE_TEST_DIR test ABSOLUTE)
  add_definitions(-DTEST_DIR=\"${ABSOLUTE_TEST_DIR}\")

  ament_add_gtest(test_camera_model
    test/test_camera_model.cc
  )
  target_link_libraries(test_camera_model
    camera
  )

  ament_add_gtest(test_camera_params
    test/test_camera_params.cc
  )
  target_link_libraries(test_camera_params
    camera
  )

endif()

#############
## Install ##
#############

target_include_directories(camera
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT camera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install C++ executables
install(TARGETS undistort_image DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/undistort_image share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()
