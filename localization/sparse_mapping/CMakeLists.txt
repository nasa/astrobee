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
project(sparse_mapping)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  ff_msgs
  ff_common
  camera
  interest_point
  config_reader
)

# System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMVG QUIET REQUIRED)
find_package(Protobuf REQUIRED)
find_package(dbow2 REQUIRED)


# Find OpenCV
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV331 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sparse_mapping
  CATKIN_DEPENDS ff_msgs ff_common camera interest_point config_reader
)

###########
## Build ##
###########

# Build the protobuf message
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS protobuf/sparse_map.proto)
get_filename_component(PROTO_HDR_DIR ${PROTO_HDRS} DIRECTORY)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENMVG_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${PROTO_HDR_DIR}
  ${DBOW2_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(sparse_mapping
  src/ransac.cc
  src/reprojection.cc
  src/sparse_map.cc
  src/sparse_mapping.cc
  src/tensor.cc
  src/vocab_tree.cc
  ${PROTO_SRCS}
)
add_dependencies(sparse_mapping ${catkin_EXPORTED_TARGETS})
target_link_libraries(sparse_mapping
  ${catkin_LIBRARIES}
  ${DBOW2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENMVG_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)


if (NOT USE_CTC)
## Declare a C++ executable: build_map
add_executable(build_map tools/build_map.cc)
add_dependencies(build_map ${catkin_EXPORTED_TARGETS})
target_link_libraries(build_map
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: evaluate_camera
add_executable(evaluate_camera tools/evaluate_camera.cc)
add_dependencies(evaluate_camera ${catkin_EXPORTED_TARGETS})
target_link_libraries(evaluate_camera
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: evaluate_localization
add_executable(evaluate_localization tools/evaluate_localization.cc)
add_dependencies(evaluate_localization ${catkin_EXPORTED_TARGETS})
target_link_libraries(evaluate_localization
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: extract_camera_info
add_executable(extract_camera_info tools/extract_camera_info.cc)
add_dependencies(extract_camera_info ${catkin_EXPORTED_TARGETS})
target_link_libraries(extract_camera_info
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: extract_submap
add_executable(extract_submap tools/extract_submap.cc)
add_dependencies(extract_submap ${catkin_EXPORTED_TARGETS})
target_link_libraries(extract_submap
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: generate_localization_test
add_executable(generate_localization_test tools/generate_localization_test.cc)
add_dependencies(generate_localization_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(generate_localization_test
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: gen_trajectory
add_executable(gen_trajectory tools/gen_trajectory.cc)
add_dependencies(gen_trajectory ${catkin_EXPORTED_TARGETS})
target_link_libraries(gen_trajectory
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: import_map
add_executable(import_map tools/import_map.cc)
add_dependencies(import_map ${catkin_EXPORTED_TARGETS})
target_link_libraries(import_map
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: localize_cams
add_executable(localize_cams tools/localize_cams.cc)
add_dependencies(localize_cams ${catkin_EXPORTED_TARGETS})
target_link_libraries(localize_cams
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: localize
add_executable(localize tools/localize.cc)
add_dependencies(localize ${catkin_EXPORTED_TARGETS})
target_link_libraries(localize
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: merge_maps 
add_executable(merge_maps tools/merge_maps.cc)
add_dependencies(merge_maps ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_maps
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: nvm_visualize
add_executable(nvm_visualize tools/nvm_visualize.cc)
add_dependencies(nvm_visualize ${catkin_EXPORTED_TARGETS})
target_link_libraries(nvm_visualize
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: parse_cam
add_executable(parse_cam tools/parse_cam.cc)
add_dependencies(parse_cam ${catkin_EXPORTED_TARGETS})
target_link_libraries(parse_cam
  sparse_mapping gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: select_images
add_executable(select_images tools/select_images.cc)
add_dependencies(select_images ${catkin_EXPORTED_TARGETS})
target_link_libraries(select_images
  sparse_mapping gflags glog ${catkin_LIBRARIES})
endif (NOT USE_CTC)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_build_db
    test/test_build_db.test
    test/test_build_db.cc
  )
  target_link_libraries(test_build_db
    sparse_mapping glog
  )

  add_rostest_gtest(test_nvm_fileio
    test/test_nvm_fileio.test
    test/test_nvm_fileio.cc
  )
  target_link_libraries(test_nvm_fileio
    sparse_mapping
  )

  add_rostest_gtest(test_registration
    test/test_registration.test
    test/test_registration.cc
  )
  target_link_libraries(test_registration
    sparse_mapping
  )

  add_rostest_gtest(test_reprojection
    test/test_reprojection.test
    test/test_reprojection.cc
  )
  target_link_libraries(test_reprojection
    sparse_mapping
  )

  add_rostest_gtest(test_sparse_map
    test/test_sparse_map.test
    test/test_sparse_map.cc
  )
  target_link_libraries(test_sparse_map
    sparse_mapping glog
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
