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
project(localization_analysis)

if (USE_ROS)

  ## Compile as C++14, supported in ROS Kinetic and newer
  add_compile_options(-std=c++14)

  ## Find catkin macros and libraries
  find_package(catkin2 REQUIRED COMPONENTS
    roscpp
    rosbag
    camera
    depth_odometry
    ff_util
    ff_msgs
    ff_common
    graph_localizer
    imu_augmentor
    imu_bias_tester
    lk_optical_flow
    localization_node
  )

  # Find GTSAM
  find_package(GTSAM REQUIRED)

  catkin_package(
    LIBRARIES ${PROJECT_NAME} ${OpenCV_LIBRARIES} 
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rosbag camera depth_odometry ff_util ff_msgs ff_common graph_localizer imu_augmentor imu_bias_tester lk_optical_flow localization_node 
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
    src/depth_odometry_adder.cc
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

  ## Declare a C++ executable: run_bag_imu_filterer
  add_executable(run_depth_odometry_adder tools/run_depth_odometry_adder.cc)
  add_dependencies(run_depth_odometry_adder ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_depth_odometry_adder
    ${PROJECT_NAME} ${catkin_LIBRARIES})

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

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install C++ executables
install(TARGETS run_bag_imu_filterer DESTINATION bin)
install(TARGETS run_depth_odometry_adder DESTINATION bin)
install(TARGETS run_graph_bag DESTINATION bin)
install(TARGETS run_imu_bias_tester_adder DESTINATION bin)
install(TARGETS run_sparse_mapping_pose_adder DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/run_bag_imu_filterer share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_depth_odometry_adder share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_graph_bag share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_imu_bias_tester_adder share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_sparse_mapping_pose_adder share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

endif (USE_ROS)
