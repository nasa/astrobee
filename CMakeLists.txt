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
project(Astrobee)

set(ASTROBEE_VERSION 0.14.0)

# Define our options
option(USE_CCACHE
  "Use ccache to speed up compiling, at the cost of optimization"
  ON)
option(USE_CTC
  "Use cross compile toolchain for making ARM binaries"
  OFF)
option(USE_ROS
  "Build the ROS-dependent functionality."
  ON)
option(USE_DDS
  "Build the DDS-dependent functionality."
  ON)
option(USE_STATIC_LIBS
  "Build using static libraries. Will use lots of drive space."
  OFF)
option(TEST_COVERAGE
  "Build the code with code coverage options. Not compatible with USE_CTC."
  OFF)
option(USE_DRIVERS
  "Build the tools in the drivers directory."
  ON)
option(IS_BAMBOO_BUILD
  "The code is being built under the bamboo CI system"
  OFF)
option(ENABLE_GPROF
  "Enable compling with support for profiling wih gprof (the GNU Profiler)."
  OFF)
option(ENABLE_GOOGLE_PROF
  "Enable support for profiling wih pprof (the Google Profiler)."
  OFF)
option(ENABLE_QP
  "Enable support for the QP planner."
  ON)
option(ENABLE_PICOFLEXX
  "Enable support for building the PicoFlexx driver"
  ON)
option(ENABLE_GAZEBO
  "Enable support for building the Gazebo simulator"
  ON)
option(ENABLE_VIVE
  "Enable support for building the Vive drivers"
  ON)
option(ENABLE_VIVE_SOLVER
  "Enable support for building the Vive offline solver"
  ON)
option(ENABLE_INTEGRATION_TESTING
  "Build the integration tests if tests are active."
  ON)
option(BUILD_LOC_RVIZ_PLUGINS
  "Build the localization rviz plugins."
  ON)


if ( "${CMAKE_VERSION}" VERSION_GREATER 3.0.0 )
  cmake_policy(SET CMP0045 OLD)
  cmake_policy(SET CMP0046 OLD)
endif()

set(ROS_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
  set(ROS_FOUND TRUE)
else()
  message("ROS distro variable not set. Trying to figure it out...")
  set(AVAILABLE_ROS_VERSIONS "melodic;lunar;kinetic;jade;indigo")
  set(ROS_FOUND FALSE)
  foreach(version ${AVAILABLE_ROS_VERSIONS})
    if(NOT ROS_FOUND)
      find_path(ROS_H ros.h PATHS /opt/ros/${version}/include/ros)
      if(ROS_H)
        set(ROS_DISTRO ${version})
        message("Found ros version ${ROS_DISTRO}")
        set(ROS_FOUND TRUE)
      endif()
    endif()
  endforeach()
endif()

# Verify the user has the pre-commit hook
execute_process(
  COMMAND cp scripts/git/pre-commit .git/hooks
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )

# Let cmake know of our additional scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake")

if (USE_CTC)
  if("${ROS_DISTRO}" STREQUAL "kinetic")
    list(APPEND CMAKE_PREFIX_PATH "${ARM_CHROOT_DIR}/opt/ros/kinetic")
  elseif("${ROS_DISTRO}" STREQUAL "melodic")
    list(APPEND CMAKE_PREFIX_PATH "${ARM_CHROOT_DIR}/opt/ros/melodic")
  endif()
else (USE_CTC)
  if("${ROS_DISTRO}" STREQUAL "kinetic")
    list(APPEND CMAKE_PREFIX_PATH "/opt/ros/kinetic")
  elseif("${ROS_DISTRO}" STREQUAL "melodic")
    list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic")
  endif()
endif (USE_CTC)

# Enable RPATHs
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# The following works for then the binaries are not installed, but are
# ran from the build directory.
list(APPEND CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
if("${ROS_DISTRO}" STREQUAL "kinetic")
  list(APPEND CMAKE_INSTALL_RPATH "/opt/ros/kinetic/lib")
elseif("${ROS_DISTRO}" STREQUAL "melodic")
  list(APPEND CMAKE_INSTALL_RPATH "/opt/ros/melodic/lib")
endif()

# The following RPATHs work for when the binaries are installed
if (USE_CTC)
  list(APPEND CMAKE_INSTALL_RPATH "/opt/rti/ndds/lib/$ENV{NDDSARCH}")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
else (USE_CTC)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif (USE_CTC)

# If the user selected static libs .. set BUILD_SHARED_LIBS accordingly
if(USE_STATIC_LIBS)
  set(BUILD_SHARED_LIBS OFF)
else(USE_STATIC_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif(USE_STATIC_LIBS)

# Check that the compiler supports C++11
include(CheckCXXCompilerFlag)
include(CheckCXXSourceCompiles)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  SET(LAMBDA_SRC "int main(int, char*[]) {int r = [] (int x) {return x;}(0); return r;}")
  CHECK_CXX_SOURCE_COMPILES("${LAMBDA_SRC}" COMPILER_SUPPORTS_LAMBDAS)
  if (NOT COMPILER_SUPPORTS_LAMBDAS)
    message(SEND_ERROR "C++0x compiler doesn't support lambda functions.")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  endif()
else()
  message(SEND_ERROR "Couldn't find compiler which supports C++11 or C++0X.")
endif()

if (TEST_COVERAGE)
  find_program(GCOV_PATH gcov)
  if (NOT GCOV_PATH)
    message(SEND_ERROR "Couldn't find gcov, but test coverage was opted for.")
  endif()
endif()

if (ENABLE_GPROF)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
endif()

# if we're compiling for native use ccache to speed things up
if (NOT USE_CTC)
  if (USE_CCACHE)
    find_program(CCACHE_EXECUTABLE ccache)
    mark_as_advanced(CCACHE_EXECUTABLE)
    if(CCACHE_EXECUTABLE)
      foreach(LANG C CXX)
        message(STATUS "Enabling ccache for ${LANG}")
        set(CMAKE_${LANG}_COMPILER_LAUNCHER ${CCACHE_EXECUTABLE} CACHE STRING "")
      endforeach()
    endif()
  endif (USE_CCACHE)
endif (NOT USE_CTC)

# Enable OpenMP if we have it. (Ceres will use it .. maybe more of our
# products will use it later).
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)

find_package(Glog REQUIRED)
find_package(Gflags REQUIRED)
ADD_DEFINITIONS(-DFREEFLYER_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})
enable_testing()

find_package(Protobuf REQUIRED)
if (NOT PROTOBUF_PROTOC_EXECUTABLE)
  message(FATAL_ERROR "Could not find system's protoc execuable")
endif (NOT PROTOBUF_PROTOC_EXECUTABLE)

find_package(Ceres REQUIRED)

find_package(Boost 1.54.0 QUIET REQUIRED COMPONENTS filesystem system iostreams thread program_options timer)
find_package(Eigen3 REQUIRED)
find_package(ZLIB)
find_package(Luajit20 REQUIRED)

# Find OpenCV and fix a 3.3.1 bug
find_package(OpenCV 3 REQUIRED)
if (${OpenCV_VERSION} MATCHES "3.3.1")
  foreach(__cvcomponent ${OpenCV_LIB_COMPONENTS})
    set (__original_cvcomponent ${__cvcomponent})
    if(NOT __cvcomponent MATCHES "^opencv_")
      set(__cvcomponent opencv_${__cvcomponent})
    endif()
    if (TARGET ${__cvcomponent})
      set_target_properties(${__cvcomponent} PROPERTIES
          MAP_IMPORTED_CONFIG_DEBUG ""
          MAP_IMPORTED_CONFIG_RELEASE ""
          MAP_IMPORTED_CONFIG_RELWITHDEBINFO ""
          MAP_IMPORTED_CONFIG_MINSIZEREL ""
      )
    endif()
  endforeach(__cvcomponent)
endif()
set(OpenCV_LIBRARIES ${OpenCV_LIBS})

find_package(dbow2 REQUIRED)
find_package(Alvar REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMVG QUIET REQUIRED)
find_package(JsonCpp REQUIRED)
find_package(FFMPEG QUIET)    # Optional
if (ENABLE_PICOFLEXX)
  find_package(royale REQUIRED)
endif (ENABLE_PICOFLEXX)
# Find pkg-config supported libraries
if (ENABLE_VIVE)
  find_package(USB REQUIRED)
  find_package(JSONC REQUIRED)
  find_package(ARGTABLE2 REQUIRED)
  find_package(ZLIB REQUIRED)
endif (ENABLE_VIVE)


if (NOT USE_CTC)  # pkg-config needs extra work to be used for cross compile
  pkg_check_modules(YAMLCPP yaml-cpp)
  if (YAMLCPP_FOUND)
    set (YAMLCPP_INCLUDE_DIRS ${YAMLCPP_INCLUDEDIR})
  endif (YAMLCPP_FOUND)
endif (NOT USE_CTC)

if(ENABLE_GOOGLE_PROF)
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-as-needed -lprofiler -Wl,--as-needed")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
endif()

if(USE_CTC) 
  set(BUILD_LOC_RVIZ_PLUGINS OFF)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-unused-local-typedefs -Wno-packed-bitfield-compat -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-local-typedefs -Wno-packed-bitfield-compat -std=c++11")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

# Set Architecture Specific Instructions
if (USE_CTC)
  # Cross Compiling, armv7-a == Cortex A5, A7, A8 (Beagle Bone), A9
  # (Odroid), A12, A15 (Snapdragon-ish), A17
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mtune=cortex-a9 -mfpu=neon -I${ARM_CHROOT_DIR}/usr/include/arm-linux-gnueabihf")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mtune=cortex-a9 -mfpu=neon -I${ARM_CHROOT_DIR}/usr/include/arm-linux-gnueabihf")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-unused-variable")
else (USE_CTC)
  if (NOT IS_BAMBOO_BUILD)
    if ((NOT USE_CCACHE) OR (NOT CCACHE_EXECUTABLE))
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -mtune=native")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mtune=native")
    endif ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mno-avx")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-avx")
  endif (NOT IS_BAMBOO_BUILD)
endif (USE_CTC)

# Bring in catkin
if (USE_CTC)
  # Catkin brings in GTest as a dependency. We don't want it because
  # we have our own ... on top of that it doesn't build correctly for
  # ARM. Our own unit tests however will continue to
  # work. (non-catkin)
  set(CATKIN_SKIP_TESTING ON)
endif (USE_CTC)
if (USE_ROS)
  set(ENV{ROS_LANG_DISABLE} "genlisp:gennodejs:geneus:$ENV{ROS_LANG_DISABLE}")
  find_package(catkin2 COMPONENTS roscpp message_generation std_msgs geometry_msgs sensor_msgs cv_bridge image_transport tf tf2 tf2_ros rosbag nodelet rviz)
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)

  if (IS_BAMBOO_BUILD)
    include(BambooFix)
  endif()

  find_package(PCL REQUIRED COMPONENTS common)
  find_package(Octomap MODULE REQUIRED)
  #pkg_check_modules(OROCOS_KDL REQUIRED orocos_kdl)
  if (USE_CTC)
    if("${ROS_DISTRO}" STREQUAL "kinetic")
      set(OROCOS_KDL_LIBRARIES ${ARM_CHROOT_DIR}/opt/ros/kinetic/lib/liborocos-kdl.so)
    elseif("${ROS_DISTRO}" STREQUAL "melodic")
      set(OROCOS_KDL_LIBRARIES ${ARM_CHROOT_DIR}/opt/ros/melodic/lib/liborocos-kdl.so)
    endif()
  endif (USE_CTC)
  set(catkin_FOUND ${catkin2_FOUND})
endif(USE_ROS)

if (USE_DDS)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/soracore")

  if (USE_CTC)
    set(SORACORE_ROOT_DIR ${ARM_CHROOT_DIR}/usr)
  else (USE_CTC)
    set(SORACORE_ROOT_DIR /usr)
  endif (USE_CTC)

  set(MIRO_ROOT_DIR ${SORACORE_ROOT_DIR})

  find_package(QtXml REQUIRED)
  find_package(Miro REQUIRED)
  find_package(RtiDds REQUIRED)
  find_package(Soracore REQUIRED)
endif(USE_DDS)

# Include Our CMake Functions
include(CreateLibrary)
include(CreateMsgTargets)
include(CreateRosTestTargets)
include(CreateTestTargets)
include(CreateToolTargets)
include(ExternalProjects)
include(FetchContent)
include(InstallLaunchFiles)

add_subdirectory(external)

if (TEST_COVERAGE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g --coverage")
  set(CMAKE_LIBS_COVERAGE "gcov")
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# install the version.txt file when install target is run
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/version.txt
  COMMAND ${CMAKE_COMMAND}
  -D OUTPUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/version.txt
  -D ASTROBEE_VERSION=${ASTROBEE_VERSION}
  -D GIT_MODULE_FILE="${CMAKE_SOURCE_DIR}/cmake/GetGitRevisionDescription.cmake"
  -P ${CMAKE_SOURCE_DIR}/cmake/SetVersion.cmake)
add_custom_target(version DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/version.txt)
install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_CURRENT_BINARY_DIR}\" --target version)")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/version.txt DESTINATION ${CMAKE_INSTALL_PREFIX} )

# install the env_wrapper.sh for running on astrobee
install(PROGRAMS scripts/deploy/env_wrapper.sh DESTINATION ${CMAKE_INSTALL_PREFIX} )

if (USE_ROS)
  add_subdirectory(tools)
  add_subdirectory(astrobee)
  add_subdirectory(communications)
  add_subdirectory(gnc)
  add_subdirectory(mobility)
  add_subdirectory(behaviors)
  add_subdirectory(management)
  add_subdirectory(description)
  if (NOT USE_CTC AND ENABLE_GAZEBO)
    add_subdirectory(simulation)
  endif (NOT USE_CTC AND ENABLE_GAZEBO)
endif (USE_ROS)

if (USE_DRIVERS)
  add_subdirectory(hardware)
endif (USE_DRIVERS)

add_subdirectory(shared)
add_subdirectory(localization)

if (USE_DDS)
  add_subdirectory(wdock)
endif (USE_DDS)
