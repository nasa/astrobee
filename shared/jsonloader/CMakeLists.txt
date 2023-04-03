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
project(jsonloader)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(JsonCpp REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(jsonloader SHARED
  src/command.cc
  src/command_repo.cc
  src/keepout.cc
  src/keepoutio.cc
  src/plan.cc
  src/planio.cc
  src/validation.cc
)
target_link_libraries(jsonloader gflags glog ${JSONCPP_LIBRARIES} )
ament_export_targets(jsonloader HAS_LIBRARY_TARGET)

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#
#  add_rostest_gtest(test_command
#    test/test_command.test
#    test/test_command.cxx
#  )
#  target_link_libraries(test_command
#    jsonloader
#  )
#
#  add_rostest_gtest(test_keepout
#    test/test_keepout.test
#    test/test_keepout.cxx
#  )
#  target_link_libraries(test_keepout
#    jsonloader
#  )
#
#  add_rostest_gtest(test_keepoutio
#    test/test_keepoutio.test
#    test/test_keepoutio.cxx
#  )
#  target_link_libraries(test_keepoutio
#    jsonloader
#  )
#
#  add_rostest_gtest(test_plan
#    test/test_plan.test
#    test/test_plan.cxx
#  )
#  target_link_libraries(test_plan
#    jsonloader
#  )
#
#  add_rostest_gtest(test_validation
#    test/test_validation.test
#    test/test_validation.cxx
#  )
#  target_link_libraries(test_validation
#    jsonloader
#  )
#
#endif()

#############
## Install ##
#############

ament_export_include_directories(include)

target_include_directories(jsonloader
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT jsonloader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()
