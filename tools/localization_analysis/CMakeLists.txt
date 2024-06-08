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
    graph_vio
    lk_optical_flow
    localization_node
    msg_conversions 
    parameter_reader
    ros_graph_localizer
    ros_graph_vio
    ros_pose_extrapolator
  )

  # Find GTSAM
  find_package(GTSAM REQUIRED)

  catkin_package(
    LIBRARIES ${PROJECT_NAME} ${OpenCV_LIBRARIES} 
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rosbag camera depth_odometry ff_util ff_msgs ff_common graph_localizer
    msg_conversions lk_optical_flow localization_node parameter_reader ros_graph_localizer ros_graph_vio ros_pose_extrapolator
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
    #src/bag_imu_filterer.cc
    src/depth_odometry_adder.cc
    src/feature_track_image_adder.cc
    src/graph_localizer_simulator.cc
    src/graph_vio_simulator.cc
    src/imu_bias_extrapolator.cc
    src/live_measurement_simulator.cc
    src/offline_replay.cc
    src/map_matcher.cc
    src/parameter_reader.cc
    src/sparse_mapping_pose_adder.cc
    src/utilities.cc
  )
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  ## Declare a C++ executable: convert_depth_msg 
  add_executable(convert_depth_msg tools/convert_depth_msg.cc)
  add_dependencies(convert_depth_msg ${catkin_EXPORTED_TARGETS})
  target_link_libraries(convert_depth_msg
    ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_bag_imu_filterer
  #add_executable(run_bag_imu_filterer tools/run_bag_imu_filterer.cc)
  #add_dependencies(run_bag_imu_filterer ${catkin_EXPORTED_TARGETS})
  #target_link_libraries(run_bag_imu_filterer
  #  ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_depth_odometry_adder
  add_executable(run_depth_odometry_adder tools/run_depth_odometry_adder.cc)
  add_dependencies(run_depth_odometry_adder ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_depth_odometry_adder
    ${PROJECT_NAME} ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_offline_replay
  add_executable(run_offline_replay tools/run_offline_replay.cc)
  add_dependencies(run_offline_replay ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_offline_replay
    ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_imu_bias_extrapolator
  add_executable(run_imu_bias_extrapolator tools/run_imu_bias_extrapolator.cc)
  add_dependencies(run_imu_bias_extrapolator ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_imu_bias_extrapolator
    ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_map_matcher
  add_executable(run_map_matcher tools/run_map_matcher.cc)
  add_dependencies(run_map_matcher ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_map_matcher
    ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

  ## Declare a C++ executable: run_sparse_mapping_pose_adder
  add_executable(run_sparse_mapping_pose_adder tools/run_sparse_mapping_pose_adder.cc)
  add_dependencies(run_sparse_mapping_pose_adder ${catkin_EXPORTED_TARGETS})
  target_link_libraries(run_sparse_mapping_pose_adder
  ${PROJECT_NAME} gflags gtsam ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_offline_replay_parameter_reader
    test/test_offline_replay_parameter_reader.test
    test/test_offline_replay_parameter_reader.cc
  )
  target_link_libraries(test_offline_replay_parameter_reader
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

# Install C++ executables
#install(TARGETS run_bag_imu_filterer DESTINATION bin)
install(TARGETS run_depth_odometry_adder DESTINATION bin)
install(TARGETS run_offline_replay DESTINATION bin)
install(TARGETS run_imu_bias_extrapolator DESTINATION bin)
install(TARGETS run_map_matcher DESTINATION bin)
install(TARGETS run_sparse_mapping_pose_adder DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/convert_depth_msg share/${PROJECT_NAME}
  #COMMAND ln -s ../../bin/run_bag_imu_filterer share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_depth_odometry_adder share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_offline_replay share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_imu_bias_extrapolator share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_map_matcher share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_sparse_mapping_pose_adder share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

  # Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

endif (USE_ROS)
