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
project(jsonloader)


## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  std_msgs
  nodelet
)

find_package(Eigen3 REQUIRED)
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(JsonCpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jsonloader
  CATKIN_DEPENDS roscpp std_msgs nodelet
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(jsonloader
  src/command.cc
  src/command_repo.cc
  src/keepout.cc
  src/keepoutio.cc
  src/plan.cc
  src/planio.cc
  src/validation.cc
)
add_dependencies(jsonloader ${catkin_EXPORTED_TARGETS})
target_link_libraries(jsonloader gflags glog ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES} )


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_command
    test/test_command.test
    test/test_command.cxx
  )
  target_link_libraries(test_command
    jsonloader
  )

  add_rostest_gtest(test_keepout
    test/test_keepout.test
    test/test_keepout.cxx
  )
  target_link_libraries(test_keepout
    jsonloader
  )

  add_rostest_gtest(test_keepoutio
    test/test_keepoutio.test
    test/test_keepoutio.cxx
  )
  target_link_libraries(test_keepoutio
    jsonloader
  )

  add_rostest_gtest(test_plan
    test/test_plan.test
    test/test_plan.cxx
  )
  target_link_libraries(test_plan
    jsonloader
  )

  add_rostest_gtest(test_validation
    test/test_validation.test
    test/test_validation.cxx
  )
  target_link_libraries(test_validation
    jsonloader
  )

endif()

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
