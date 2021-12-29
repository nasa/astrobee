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
project(point_cloud_common)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  ff_util
  localization_common 
  msg_conversions 
  optimization_common
)

find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

find_package(PCL REQUIRED COMPONENTS common features filters octree search)

find_package(pcl_conversions REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${CERES_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS} 
  LIBRARIES ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARIES} 
  CATKIN_DEPENDS roscpp ff_msgs ff_util localization_common msg_conversions optimization_common 
  DEPENDS gtsam pcl 
)

###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/parameter_reader.cc
  src/point_cloud_with_known_correspondences_aligner.cc
  src/test_utilities.cc
  src/utilities.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

if(CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)
  add_rostest_gtest(test_point_cloud_common_utilities
    test/test_point_cloud_common_utilities.test
    test/test_point_cloud_common_utilities.cc
  )
  target_link_libraries(test_point_cloud_common_utilities
    ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam 
  )
  add_rostest_gtest(test_point_to_plane_icp
    test/test_point_to_plane_icp.test
    test/test_point_to_plane_icp.cc
  )
  target_link_libraries(test_point_to_plane_icp
    ${PROJECT_NAME} ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_point_cloud_with_known_correspondences_aligner
    test/test_point_cloud_with_known_correspondences_aligner.test
    test/test_point_cloud_with_known_correspondences_aligner.cc
  )
  target_link_libraries(test_point_cloud_with_known_correspondences_aligner
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

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
