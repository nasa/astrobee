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

cmake_minimum_required(VERSION 2.8.3)
project(graph_bag)

if (USE_ROS)

  ## Compile as C++14, supported in ROS Kinetic and newer
  add_compile_options(-std=c++14)

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    ff_util
    ff_msgs
    ff_common
    graph_localizer
    camera
    lk_optical_flow
    imu_bias_tester
    localization_node
    imu_augmentor
  )

  # Find GTSAM
  find_package(GTSAM REQUIRED)

  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rosbag ff_util ff_msgs ff_common graph_localizer camera lk_optical_flow imu_bias_tester localization_node imu_augmentor
  )
   
  ###########
  ## Build ##
  ###########
  # Specify additional locations of header files
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

  # Declare C++ libraries
  add_library(${PROJECT_NAME}
    src/bag_imu_filterer.cc
    src/graph_bag.cc
    src/graph_localizer_simulator.cc
    src/imu_bias_tester_adder.cc
    src/live_measurement_simulator.cc
    src/parameter_reader.cc
    src/sparse_mapping_pose_adder.cc
    src/utilities.cc
  )
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_bag_imu_filterer
  add_executable(run_bag_imu_filterer tools/run_bag_imu_filterer.cc)
  add_dependencies(run_bag_imu_filterer ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_bag_imu_filterer
    ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_graph_bag
  add_executable(run_graph_bag tools/run_graph_bag.cc)
  add_dependencies(run_graph_bag ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_graph_bag
    ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_imu_bias_tester_adder
  add_executable(run_imu_bias_tester_adder tools/run_imu_bias_tester_adder.cc)
  add_dependencies(run_imu_bias_tester_adder ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_imu_bias_tester_adder
    ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_sparse_mapping_pose_adder
  add_executable(run_sparse_mapping_pose_adder tools/run_sparse_mapping_pose_adder.cc)
  add_dependencies(run_sparse_mapping_pose_adder ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_sparse_mapping_pose_adder
    ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

endif (USE_ROS)
