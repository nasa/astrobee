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

cmake_minimum_required(VERSION 2.8.3)

project(traj_opt_pro)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations  -Werror=return-type")

############################ Optional Build Params #############################
set(BUILD_GUROBI FALSE) # Commercial backend
set(INTEL_MKL FALSE)   # Use Intel's Math Kernel Library (Not faster on an i7) 
############################ End Optional Params ###############################

# Find optional packages
if(BUILD_GUROBI)
  find_package(GUROBI REQUIRED)
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()
if(INTEL_MKL)
   add_definitions(-DEIGEN_USE_MKL_ALL)
   find_package(IntelMKL REQUIRED)
   set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed -lpthread -lm -ldl" )
   set(CMAKE_SHARED_LINKER_FLAGS  "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-as-needed -lpthread -lm -ldl" )
  include_directories(${INTEL_MKL_INCLUDES})
endif()

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  traj_opt_basic
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    traj_opt_pro
)


###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
)

# Add libraries
add_library(traj_opt_pro
  src/polynomial_basis.cpp
  src/trajectory_solver.cpp)

if(BUILD_GUROBI)
  add_library(gurobi_backend
    src/gurobi_solver.cpp
    src/gurobi_trajectory.cpp )
  target_link_libraries(gurobi_backend traj_opt_pro ${GUROBI_LIBRARIES})
endif()

add_library(fancy_custom_backend
  src/nonlinear_polynomial.cpp
  src/nonlinear_solver.cpp
  src/nonlinear_trajectory.cpp)
target_link_libraries(fancy_custom_backend traj_opt_pro ${OpenCV_LIBRARIES})
