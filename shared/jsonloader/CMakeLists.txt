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

find_package(Boost REQUIRED COMPONENTS filesystem)
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
target_link_libraries(jsonloader
  PUBLIC gflags glog ${JSONCPP_LIBRARIES}
  PRIVATE Boost::filesystem
)
ament_export_targets(jsonloader HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  foreach(test_name test_command test_keepout test_keepoutio test_plan test_validation)
    ament_add_gtest(${test_name} test/${test_name}.cxx
      SKIP_LINKING_MAIN_LIBRARIES
      ENV "DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/data/"
    )
    if(TARGET ${test_name})
      target_link_libraries(${test_name} jsonloader)
    endif()
  endforeach()

  if(TARGET test_plan)
    target_link_libraries(test_plan Boost::filesystem)
  endif()
endif()

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
