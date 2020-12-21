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
if (ENABLE_PICOFLEXX)

cmake_minimum_required(VERSION 2.8.3)
project(pico_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_msgs
)

# System dependencies are found with CMake's conventions
find_package(royale REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet ff_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Declare C++ libraries
add_library(pico_driver
  src/pico_driver.cc
)
add_dependencies(pico_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(pico_driver ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(pico_proxy tools/pico_proxy.cc)
add_dependencies(pico_proxy ${catkin_EXPORTED_TARGETS})
target_link_libraries(pico_proxy
  pico_driver gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(pico_tool tools/pico_tool.cc)
add_dependencies(pico_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(pico_tool
  pico_driver gflags glog ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # Pico flexx initialization fault tester
  add_rostest_gtest(test_init_pico_flexx
    test/test_init_pico_flexx.test
    test/test_init_pico_flexx.cc
  )

  target_link_libraries(test_init_pico_flexx
    ${catkin_LIBRARIES} config_reader ff_nodelet
  )
endif()

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

endif (ENABLE_PICOFLEXX)