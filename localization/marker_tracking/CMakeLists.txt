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
project(marker_tracking)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  ff_util
  message_runtime
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  camera msg_conversions
)

# System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Alvar REQUIRED)
find_package(Alvar REQUIRED)


# Find OpenCV
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV331 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp ff_util message_runtime std_msgs geometry_msgs cv_bridge image_transport camera msg_conversions
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ALVAR_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(marker_tracking
  src/marker_tracking/arconfigio.cc
  src/marker_tracking/arxmlio.cc
  src/marker_tracking/labelling_method.cc
  src/marker_tracking/marker_detector.cc
)
add_dependencies(marker_tracking ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_tracking ${catkin_LIBRARIES} ${ALVAR_LIBRARIES})

## Declare a C++ executable: generate_ar_target
add_executable(generate_ar_target tools/marker_tracking/generate_ar_target.cc)
add_dependencies(generate_ar_target ${catkin_EXPORTED_TARGETS})
target_link_libraries(generate_ar_target
  marker_tracking gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: generate_multiscale_target
add_executable(generate_multiscale_target tools/marker_tracking/generate_multiscale_target.cc)
add_dependencies(generate_multiscale_target ${catkin_EXPORTED_TARGETS})
target_link_libraries(generate_multiscale_target
  marker_tracking gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: generate_svg_markers
add_executable(generate_svg_markers tools/marker_tracking/generate_svg_markers.cc)
add_dependencies(generate_svg_markers ${catkin_EXPORTED_TARGETS})
target_link_libraries(generate_svg_markers
  marker_tracking gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: identify_marker_in_image
add_executable(identify_marker_in_image tools/marker_tracking/identify_marker_in_image.cc)
add_dependencies(identify_marker_in_image ${catkin_EXPORTED_TARGETS})
target_link_libraries(identify_marker_in_image
  marker_tracking gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: marker_config2xml
add_executable(marker_config2xml tools/marker_tracking/marker_config2xml.cc)
add_dependencies(marker_config2xml ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_config2xml
  marker_tracking gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: markers_to_Kalibr
add_executable(markers_to_Kalibr tools/marker_tracking/markers_to_Kalibr.cc)
add_dependencies(markers_to_Kalibr ${catkin_EXPORTED_TARGETS})
target_link_libraries(markers_to_Kalibr
  marker_tracking gflags glog yaml-cpp ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(marker_tracking_node
  src/marker_tracking_node/marker_tracker.cc
  src/marker_tracking_node/marker_tracking_nodelet.cc
)
add_dependencies(marker_tracking_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_tracking_node marker_tracking ${catkin_LIBRARIES})

## Declare a C++ executable: overhead_tracking_node
add_executable(overhead_tracking_node tools/marker_tracking_node/overhead_tracking_node.cc)
add_dependencies(overhead_tracking_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(overhead_tracking_node
  marker_tracking_node gflags glog ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_arxmlio test/test_arxmlio.test
    test/test_arxmlio.cc
  )
  target_link_libraries(test_arxmlio
    marker_tracking
  )

  add_rostest_gtest(test_marker_config_loader test/test_marker_config_loader.test
    test/test_marker_config_loader.cc
  )
  target_link_libraries(test_marker_config_loader
    marker_tracking
  )

  add_rostest_gtest(test_marker_detector test/test_marker_detector.test
    test/test_marker_detector.cc
  )
  target_link_libraries(test_marker_detector
    marker_tracking
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
install(TARGETS marker_tracking_node
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

# Install C++ executables
install(TARGETS overhead_tracking_node DESTINATION bin)
install(TARGETS overhead_tracking_node DESTINATION share/${PROJECT_NAME})

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(FILES
  tools/marker_tracking_node/features_counter.py
  DESTINATION share/${PROJECT_NAME})

