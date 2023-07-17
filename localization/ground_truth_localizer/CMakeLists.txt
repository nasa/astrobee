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
project(ground_truth_localizer)

# if (USE_ROS)
 
## Compile as C++14, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )


# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)

find_package(std_srvs REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(ff_util REQUIRED)
find_package(localization_common REQUIRED)
find_package(ff_common REQUIRED)
find_package(camera REQUIRED)
find_package(GTSAM REQUIRED)



###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME} SHARED
  src/ground_truth_localizer_component.cc
  src/utilities.cc
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components ff_msgs ff_util localization_common std_srvs)
rclcpp_components_register_nodes(${PROJECT_NAME} "ground_truth_localizer::GroundTruthLocalizerComponent")


#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mark launch files for installation
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)

ament_package()

# else (USE_ROS)
#   find_package(catkin REQUIRED COMPONENTS)
#   catkin_package()
# endif (USE_ROS)
