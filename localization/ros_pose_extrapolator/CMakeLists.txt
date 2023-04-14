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

cmake_minimum_required(VERSION 3.0)
project(ros_pose_extrapolator)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Find GTSAM
find_package(GTSAM REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  ff_util
  imu_integration
  localization_common
  localization_measurements
  nodelet
  parameter_reader
)

catkin_package(
  INCLUDE_DIRS include ${GTSAM_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS} 
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} gtsam 
  CATKIN_DEPENDS
    ff_util
    imu_integration
    localization_common
    localization_measurements
    nodelet
    parameter_reader
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/parameter_reader.cc
  src/ros_pose_extrapolator_nodelet.cc
  src/ros_pose_extrapolator_wrapper.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	#  add_rostest_gtest(test_ros_pose_extrapolator_wrapper
	#    test/test_ros_pose_extrapolator_wrapper.test
	#    test/test_ros_pose_extrapolator_wrapper.cc
	#    test/test_utilities.cc
	#  )
	#  target_link_libraries(test_ros_pose_extrapolator_wrapper
	#    ros_pose_extrapolator ${catkin_LIBRARIES} 
	#  )
  add_rostest_gtest(test_ros_pose_extrapolator_parameter_reader
    test/test_ros_pose_extrapolator_parameter_reader.test
    test/test_ros_pose_extrapolator_parameter_reader.cc
  )
  target_link_libraries(test_ros_pose_extrapolator_parameter_reader
    ${PROJECT_NAME} ${catkin_LIBRARIES}
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

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

