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

project(gnc_autocode)

# find gnc directories
find_path ( GNC_ROOT_DIR ctl_controller0_ert_rtw/ctl_controller0.h
  PATHS ${CMAKE_SOURCE_DIR}/gnc/matlab/code_generation
  HINTS ${CMAKE_SOURCE_DIR}/gnc/matlab/code_generation
  NO_CMAKE_FIND_ROOT_PATH
)

if (GNC_ROOT_DIR)
  # blindly setting paths, TODO: confirm these paths exist
  set (GNC_ADDITIONAL_DIR ${GNC_ROOT_DIR}/AdditionalIncludes)
  set (GNC_CTL_DIR ${GNC_ROOT_DIR}/ctl_controller0_ert_rtw)
  set (GNC_EST_DIR ${GNC_ROOT_DIR}/est_estimator_ert_rtw)
  set (GNC_FAM_DIR ${GNC_ROOT_DIR}/fam_force_allocation_module_ert_rtw)
  set (GNC_SIM_DIR ${GNC_ROOT_DIR}/sim_model_lib0_ert_rtw)
  set (GNC_BL1_DIR ${GNC_ROOT_DIR}/bpm_blower_1_propulsion_module_ert_rtw)
  set (GNC_BL2_DIR ${GNC_ROOT_DIR}/bpm_blower_2_propulsion_module_ert_rtw)
  set (GNC_UTIL_DIR ${GNC_ROOT_DIR}/sharedutils)
  set (GNC_SPD_DIR ${GNC_ROOT_DIR}/esc_electronic_speed_controller0_ert_rtw)
  set (GNC_CXX_DIR ${GNC_ROOT_DIR}/../cxx_functions)
  set (GNC_INCLUDE_DIRS ${GNC_ADDITIONAL_DIR} ${GNC_CTL_DIR} ${GNC_EST_DIR} ${GNC_FAM_DIR})
else (GNC_ROOT_DIR)
  message(ERROR "  GNC_ROOT_DIR not found! check PATHS in setGncPaths.cmake")
endif (GNC_ROOT_DIR)

file(GLOB GNC_UTIL_SOURCES ${GNC_UTIL_DIR}/*.cpp)

set(GNC_SOURCES
  ${GNC_CTL_DIR}/ctl_tunable_funcs.cpp
  ${GNC_CTL_DIR}/ctl_controller0.cpp
  ${GNC_CTL_DIR}/ctl_controller0_data.cpp
  ${GNC_EST_DIR}/est_estimator.cpp
  ${GNC_EST_DIR}/est_estimator_data.cpp
  ${GNC_EST_DIR}/est_tunable_funcs.cpp
  ${GNC_FAM_DIR}/fam_force_allocation_module.cpp
  ${GNC_FAM_DIR}/fam_force_allocation_module_data.cpp
  ${GNC_FAM_DIR}/fam_tunable_funcs.cpp
  ${GNC_SIM_DIR}/sim_model_lib0.cpp
  ${GNC_SIM_DIR}/sim_model_lib0_data.cpp
  ${GNC_SIM_DIR}/sim_tunable_funcs.cpp
  ${GNC_SIM_DIR}/blower_aerodynamics.cpp
  ${GNC_BL1_DIR}/blower_aerodynamics.cpp
  ${GNC_BL1_DIR}/bpm_blower_1_propulsion_module.cpp
  ${GNC_BL1_DIR}/bpm_blower_1_propulsion_module_data.cpp
  ${GNC_BL2_DIR}/bpm_blower_2_propulsion_module.cpp
  ${GNC_BL2_DIR}/bpm_blower_2_propulsion_module_data.cpp
  ${GNC_CXX_DIR}/src/of_residual_and_h.cpp
  ${GNC_CXX_DIR}/src/compute_delta_state_and_cov.cpp
  ${GNC_CXX_DIR}/src/matrix_multiply.cpp
  ${GNC_CXX_DIR}/src/apply_delta_state.cpp
  ${GNC_CXX_DIR}/src/ros_log.cpp
  ${GNC_CXX_DIR}/src/fault_assert.cpp
  ${GNC_CXX_DIR}/src/fault_clear.cpp
  ${GNC_UTIL_SOURCES}
)

set(GNC_INCLUDES
  ${GNC_CTL_DIR}
  ${GNC_EST_DIR}
  ${GNC_FAM_DIR}
  ${GNC_SIM_DIR}
  ${GNC_BL1_DIR}
  ${GNC_BL2_DIR}
  ${GNC_SPD_DIR}
  ${GNC_UTIL_DIR}
  ${GNC_CXX_DIR}/include
)

STRING(REGEX REPLACE "-Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
STRING(REGEX REPLACE "-Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
STRING(REGEX REPLACE "-Werror[^ ]*" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
STRING(REGEX REPLACE "-Werror[^ ]*" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -DPORTABLE_WORDSIZES -ffast-math -funsafe-math-optimizations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -DPORTABLE_WORDSIZES -ffast-math -funsafe-math-optimizations")

create_library(TARGET gnc_autocode
  LIBS ${EIGEN_LIBRARIES} ff_nodelet config_reader msg_conversions
  INC ${GNC_INCLUDES} ${EIGEN_INCLUDE_DIRS}
  ADD_SRCS ${GNC_SOURCES}
)

