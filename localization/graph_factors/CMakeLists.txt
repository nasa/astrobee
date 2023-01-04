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
project(graph_factors)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
	localization_common
  vision_common
)

# Find GTSAM
find_package(GTSAM REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES}
  INCLUDE_DIRS include ${GLOG_INCLUDE_DIRS}
  CATKIN_DEPENDS localization_common vision_common 
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

file(GLOB cc_files
     "src/*.cc"
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  ${cc_files}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)
 add_rostest_gtest(test_inverse_depth_projection_factor
    test/test_inverse_depth_projection_factor.test
    test/test_inverse_depth_projection_factor.cc
  )
  target_link_libraries(test_inverse_depth_projection_factor
    graph_factors ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_point_to_line_factor
    test/test_point_to_line_factor.test
    test/test_point_to_line_factor.cc
  )
  target_link_libraries(test_point_to_line_factor
    graph_factors ${catkin_LIBRARIES}
  )
  add_rostest_gtest(test_point_to_line_segment_factor
    test/test_point_to_line_segment_factor.test
    test/test_point_to_line_segment_factor.cc
  )
  target_link_libraries(test_point_to_line_segment_factor
    graph_factors ${catkin_LIBRARIES}
  )
  add_rostest_gtest(test_point_to_plane_factor
    test/test_point_to_plane_factor.test
    test/test_point_to_plane_factor.cc
  )
  target_link_libraries(test_point_to_plane_factor
    graph_factors ${catkin_LIBRARIES}
  )
  add_rostest_gtest(test_point_to_point_between_factor
    test/test_point_to_point_between_factor.test
    test/test_point_to_point_between_factor.cc
  )
  target_link_libraries(test_point_to_point_between_factor
    graph_factors ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_point_to_handrail_endpoint_factor
    test/test_point_to_handrail_endpoint_factor.test
    test/test_point_to_handrail_endpoint_factor.cc
  )
  target_link_libraries(test_point_to_handrail_endpoint_factor
    graph_factors ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_silu
    test/test_silu.test
    test/test_silu.cc
  )
  target_link_libraries(test_silu
    graph_factors ${catkin_LIBRARIES}
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
