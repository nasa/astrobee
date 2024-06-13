###################################################################################################
#                                                                                                 #
# This file is part of HPIPM.                                                                     #
#                                                                                                 #
# HPIPM -- High-Performance Interior Point Method.                                                #
# Copyright (C) 2019 by Gianluca Frison.                                                          #
# Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              #
# All rights reserved.                                                                            #
#                                                                                                 #
# The 2-Clause BSD License                                                                        #
#                                                                                                 #
# Redistribution and use in source and binary forms, with or without                              #
# modification, are permitted provided that the following conditions are met:                     #
#                                                                                                 #
# 1. Redistributions of source code must retain the above copyright notice, this                  #
#    list of conditions and the following disclaimer.                                             #
# 2. Redistributions in binary form must reproduce the above copyright notice,                    #
#    this list of conditions and the following disclaimer in the documentation                    #
#    and/or other materials provided with the distribution.                                       #
#                                                                                                 #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 #
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   #
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          #
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 #
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  #
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     #
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   #
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    #
#                                                                                                 #
# Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             #
#                                                                                                 #
###################################################################################################

cmake_minimum_required(VERSION 2.8.11)

# installation directory ( has to be before `project()` )
set(CMAKE_INSTALL_PREFIX "/opt/hpipm" CACHE STRING "Installation path")

project(hpipm C)

set(CMAKE_VERBOSE_MAKEFILE ON)

# Options

# Target architecture
set(TARGET AVX CACHE STRING "Set CPU architecture target")
#set(TARGET GENERIC CACHE STRING "Set CPU architecture target")

# Populate a list of allowable targets and link it to the option
set(ALLOWED_TARGETS
		AVX
		GENERIC
		)
set_property(CACHE TARGET PROPERTY STRINGS ${ALLOWED_TARGETS})

# Use C99 extension to math library
set(USE_C99_MATH ON CACHE STRING "Use C99 extension to math library")

# BLAS and LAPACK version (for LA=BLAS in BLASFEO)
set(REF_BLAS 0 CACHE STRING "Set CPU architecture target")
# set(REF_BLAS OPENBLAS CACHE STRING "Set CPU architecture target")
# set(REF_BLAS NETLIB   CACHE STRING "Set CPU architecture target")
# set(REF_BLAS MKL      CACHE STRING "Set CPU architecture target")
# set(REF_BLAS BLIS     CACHE STRING "Set CPU architecture target")
# set(REF_BLAS ATLAS    CACHE STRING "Set CPU architecture target")

# Populate a list of allowable BLAS versions and link it to the option
set(ALLOWED_REF_BLAS
		0
		OPENBLAS
		NETLIB
		MKL
		BLIS
		ATLAS
		)
set_property(CACHE REF_BLAS PROPERTY STRINGS ${ALLOWED_REF_BLAS})

set(HPIPM_TESTING ON CACHE BOOL "Tests enabled")
# set(HPIPM_TESTING OFF CACHE BOOL "Tests disabled")

# build shared library
set(BUILD_SHARED_LIBS OFF CACHE STRING "Build shared libraries")

# Pass the allowed items up to the parent scope (if there is one)
# This is detected by comparing the project name to the one set earlier in this file
if(NOT ${CMAKE_PROJECT_NAME} STREQUAL hpipm)
	set(HPIPM_ALLOWED_TARGETS ${ALLOWED_TARGETS} PARENT_SCOPE)
	set(HPIPM_ALLOWED_REF_BLAS ${ALLOWED_REF_BLAS} PARENT_SCOPE)
endif()

# BLASFEO Option
if(NOT TARGET blasfeo)
	set(BLASFEO_PATH "/opt/blasfeo" CACHE STRING "BLASFEO installation path")
	if(BUILD_SHARED_LIBS MATCHES OFF)
		set(HPIPM_BLASFEO_LIB "Static" CACHE STRING "BLASFEO library link type")
	else()
		set(HPIPM_BLASFEO_LIB "Shared" CACHE STRING "BLASFEO library link type")
	endif()

	if(HPIPM_BLASFEO_LIB STREQUAL "Static")
		add_library(blasfeo STATIC IMPORTED)
		if(CMAKE_C_COMPILER_ID MATCHES MSVC)
			set_target_properties(blasfeo PROPERTIES IMPORTED_LOCATION ${BLASFEO_PATH}/lib/blasfeo.lib)
		else()
			set_target_properties(blasfeo PROPERTIES IMPORTED_LOCATION ${BLASFEO_PATH}/lib/libblasfeo.a)
		endif()
	elseif(HPIPM_BLASFEO_LIB STREQUAL "Shared")
		add_library(blasfeo SHARED IMPORTED)
		if(CMAKE_C_COMPILER_ID MATCHES MSVC)
			set_target_properties(blasfeo PROPERTIES IMPORTED_LOCATION ${BLASFEO_PATH}/lib/blasfeo.dll)
		else()
			set_target_properties(blasfeo PROPERTIES IMPORTED_LOCATION ${BLASFEO_PATH}/lib/libblasfeo.so)
		endif()
	endif()
endif()

# Needed to compile examples
set(BLASFEO_INCLUDE_DIR "${BLASFEO_PATH}/include" CACHE STRING "Path to BLASFEO header files.")
set(HPIPM_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

# headers installation directory
set(HPIPM_HEADERS_INSTALLATION_DIRECTORY "include" CACHE STRING "Headers local installation directory")

# enable runtine checks
set(RUNTIME_CHECKS ON)
#set(RUNTIME_CHECKS OFF)

## compiler flags
#set(CMAKE_C_FLAGS "")
#
## optimization flags
# common flags
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -fPIC")
endif()

# target-specific flags
if(${TARGET} MATCHES AVX)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTARGET_AVX")
	if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64 -mavx")
	endif()
elseif(${TARGET} MATCHES GENERIC)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTARGET_GENERIC")
endif()

#
if(${REF_BLAS} MATCHES 0)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
endif(${REF_BLAS} MATCHES 0)
if(${REF_BLAS} MATCHES OPENBLAS)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DREF_BLAS_OPENBLAS -I/opt/openblas/include")
endif(${REF_BLAS} MATCHES OPENBLAS)
if(${REF_BLAS} MATCHES BLIS)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DREF_BLAS_BLIS -std=c99")
endif(${REF_BLAS} MATCHES BLIS)
if(${REF_BLAS} MATCHES NETLIB)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DREF_BLAS_NETLIB")
endif(${REF_BLAS} MATCHES NETLIB)
if(${REF_BLAS} MATCHES MKL)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DREF_BLAS_MKL -m64 -I/opt/intel/mkl/include")
endif(${REF_BLAS} MATCHES MKL)
if(${REF_BLAS} MATCHES ATLAS)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DREF_BLAS_ATLAS")
endif(${REF_BLAS} MATCHES ATLAS)

#
if(${USE_C99_MATH} MATCHES ON)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_C99_MATH")
endif()

#
if(${RUNTIME_CHECKS} MATCHES ON)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DRUNTIME_CHECKS")
endif()

# source files
# cond
file(GLOB COND_SRC
	# double
	${PROJECT_SOURCE_DIR}/cond/d_cond.c
	${PROJECT_SOURCE_DIR}/cond/d_cond_aux.c
	${PROJECT_SOURCE_DIR}/cond/d_part_cond.c
	${PROJECT_SOURCE_DIR}/cond/d_cond_qcqp.c
	${PROJECT_SOURCE_DIR}/cond/d_cast_qcqp.c
	${PROJECT_SOURCE_DIR}/cond/d_part_cond_qcqp.c
	# single
	${PROJECT_SOURCE_DIR}/cond/s_cond.c
	${PROJECT_SOURCE_DIR}/cond/s_cond_aux.c
	${PROJECT_SOURCE_DIR}/cond/s_part_cond.c
	${PROJECT_SOURCE_DIR}/cond/s_cond_qcqp.c
	${PROJECT_SOURCE_DIR}/cond/s_cast_qcqp.c
	${PROJECT_SOURCE_DIR}/cond/s_part_cond_qcqp.c
	)
# dense qp
file(GLOB DENSE_QP_SRC
	# double
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qp.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qp_dim.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qp_sol.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qp_kkt.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qp_ipm.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qp_res.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qp_utils.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qcqp_dim.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qcqp.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qcqp_sol.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qcqp_res.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qcqp_ipm.c
	${PROJECT_SOURCE_DIR}/dense_qp/d_dense_qcqp_utils.c
	# single
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qp.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qp_dim.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qp_sol.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qp_kkt.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qp_ipm.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qp_res.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qp_utils.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qcqp_dim.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qcqp.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qcqp_sol.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qcqp_res.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qcqp_ipm.c
	${PROJECT_SOURCE_DIR}/dense_qp/s_dense_qcqp_utils.c
	)
# core ipm
if(${TARGET} MATCHES AVX)
	file(GLOB IPM_CORE_SRC
		${PROJECT_SOURCE_DIR}/ipm_core/d_core_qp_ipm_aux_avx.c
		${PROJECT_SOURCE_DIR}/ipm_core/d_core_qp_ipm.c
		${PROJECT_SOURCE_DIR}/ipm_core/s_core_qp_ipm_aux_avx.c
		${PROJECT_SOURCE_DIR}/ipm_core/s_core_qp_ipm.c
		)
else(${TARGET} MATCHES AVX)
	file(GLOB IPM_CORE_SRC
		${PROJECT_SOURCE_DIR}/ipm_core/d_core_qp_ipm_aux.c
		${PROJECT_SOURCE_DIR}/ipm_core/d_core_qp_ipm.c
		${PROJECT_SOURCE_DIR}/ipm_core/s_core_qp_ipm_aux.c
		${PROJECT_SOURCE_DIR}/ipm_core/s_core_qp_ipm.c
		)
endif(${TARGET} MATCHES AVX)
# ocp qp
file(GLOB OCP_QP_SRC
	# double
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qp.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qp_dim.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qp_sol.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qp_kkt.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qp_ipm.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qp_res.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qp_utils.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qp_red.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qcqp.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qcqp_dim.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qcqp_sol.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qcqp_ipm.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qcqp_res.c
	${PROJECT_SOURCE_DIR}/ocp_qp/d_ocp_qcqp_utils.c
	# single
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qp.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qp_dim.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qp_sol.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qp_kkt.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qp_ipm.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qp_res.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qp_utils.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qp_red.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qcqp.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qcqp_dim.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qcqp_sol.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qcqp_ipm.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qcqp_res.c
	${PROJECT_SOURCE_DIR}/ocp_qp/s_ocp_qcqp_utils.c
#	${PROJECT_SOURCE_DIR}/ocp_qp/m_ocp_qp.c
#	${PROJECT_SOURCE_DIR}/ocp_qp/m_ocp_qp_kkt.c
#	${PROJECT_SOURCE_DIR}/ocp_qp/m_ocp_qp_ipm_hard.c
	)
# tree qp
file(GLOB TREE_OCP_QP_SRC
	# common
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/scenario_tree.c
	# double
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qp_dim.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qp.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qp_sol.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qp_res.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qp_kkt.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qp_ipm.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qp_utils.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qcqp_dim.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qcqp.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qcqp_sol.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qcqp_res.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qcqp_ipm.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/d_tree_ocp_qcqp_utils.c
	# single
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qp_dim.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qp.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qp_sol.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qp_res.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qp_kkt.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qp_ipm.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qp_utils.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qcqp_dim.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qcqp.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qcqp_sol.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qcqp_res.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qcqp_ipm.c
	${PROJECT_SOURCE_DIR}/tree_ocp_qp/s_tree_ocp_qcqp_utils.c
	)
# auxiliary
file(GLOB AUXILIARY_SRC
	${PROJECT_SOURCE_DIR}/auxiliary/aux_string.c
	${PROJECT_SOURCE_DIR}/auxiliary/aux_mem.c
	${PROJECT_SOURCE_DIR}/auxiliary/timing.c
	)
# auxiliary
file(GLOB SIM_SRC
	# double
	${PROJECT_SOURCE_DIR}/sim_core/d_sim_rk.c
	${PROJECT_SOURCE_DIR}/sim_core/d_sim_erk.c
	# single
	${PROJECT_SOURCE_DIR}/sim_core/s_sim_rk.c
	${PROJECT_SOURCE_DIR}/sim_core/s_sim_erk.c
	)

set(HPIPM_SRC ${COND_SRC} ${DENSE_QP_SRC} ${IPM_CORE_SRC} ${OCP_QP_SRC} ${TREE_OCP_QP_SRC} ${AUXILIARY_SRC})

add_library(hpipm ${HPIPM_SRC})

target_include_directories(hpipm
	PUBLIC
		$<BUILD_INTERFACE:${HPIPM_INCLUDE_DIR}>
		$<BUILD_INTERFACE:${BLASFEO_INCLUDE_DIR}>
		$<INSTALL_INTERFACE:${HPIPM_HEADERS_INSTALLATION_DIRECTORY}>)

target_link_libraries(hpipm blasfeo)

install(TARGETS hpipm EXPORT hpipmConfig
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin)

install(EXPORT hpipmConfig DESTINATION cmake)

file(GLOB_RECURSE HPIPM_HEADERS "include/*.h")
install(FILES ${HPIPM_HEADERS} DESTINATION ${HPIPM_HEADERS_INSTALLATION_DIRECTORY})

# test problems
if(HPIPM_TESTING MATCHES ON)
	add_subdirectory(test_problems)
	add_subdirectory(examples/c)
endif()

message(STATUS "Using BLASFEO path: ${BLASFEO_PATH}")
message(STATUS "Installation directory: ${CMAKE_INSTALL_PREFIX}")
