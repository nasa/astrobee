###################################################################################################
#                                                                                                 #
# This file is part of BLASFEO.                                                                   #
#                                                                                                 #
# BLASFEO -- BLAS For Embedded Optimization.                                                      #
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
set(CMAKE_INSTALL_PREFIX "/opt/blasfeo" CACHE STRING "Installation path")

project(blasfeo C ASM)

set(CMAKE_VERBOSE_MAKEFILE ON)

# enable_language(C ASM)


# Target architecture
# set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture")
# set(TARGET X64_INTEL_HASWELL CACHE STRING "Target architecture")
# set(TARGET X64_INTEL_SANDY_BRIDGE CACHE STRING "Target architecture")
# set(TARGET X64_INTEL_CORE CACHE STRING "Target architecture")
# set(TARGET X64_AMD_BULLDOZER CACHE STRING "Target architecture")
# set(TARGET ARMV8A_ARM_CORTEX_A57 CACHE STRING "Target architecture")
# set(TARGET ARMV8A_ARM_CORTEX_A53 CACHE STRING "Target architecture")
# set(TARGET ARMV7A_ARM_CORTEX_A15 CACHE STRING "Target architecture")
# set(TARGET ARMV7A_ARM_CORTEX_A9 CACHE STRING "Target architecture")
# set(TARGET ARMV7A_ARM_CORTEX_A7 CACHE STRING "Target architecture")
set(TARGET GENERIC CACHE STRING "Target architecture")

# Select back-end linear lagebra version (LA) to implement BLASFEO API:
# HIGH_PERFORMANCE : target-tailored; performance-optimized for cache resident matrices; panel-major matrix format
# REFERENCE : target-unspecific lightly-optimized; small code footprint; {panel,column}-major matrix format(s)
# EXTERNAL_BLAS_WRAPPER : call to external BLAS and LAPACK libraries; column-major matrix format
set(LA HIGH_PERFORMANCE CACHE STRING "Linear algebra backend")
# set(LA REFERENCE CACHE STRING "Linear algebra backend")
# set(LA EXTERNAL_BLAS_WRAPPER CACHE STRING "Linear algebra backend")

# Export additional back-ends with different naming
set(BLASFEO_REF_API ON)
set(BLASFEO_HP_API OFF)

# Select the Matrix Format internally used in the blasfeo_{d,s}mat structure
# COLMAJ : column-major matrix format
# PANELMAJ : panel-major matrix format
# set(MF COLMAJ CACHE STRING "Matrix Format internally used in blasfeo_{d,s}mat struct")
set(MF PANELMAJ CACHE STRING "Matrix Format internally used in blasfeo_{d,s}mat struct")

# Compile the BLAS API routines provided by BLASFEO
set(BLAS_API ON CACHE BOOL "Compile BLAS API")
#set(BLAS_API OFF CACHE BOOL "Compile BLAS API")

# Export standard FORTRAN namings for BLAS API routines
# OFF : routines namings are in the form blasfeo_dgemm
# ON  : routines namings are in the form dgemm_
set(FORTRAN_BLAS_API OFF CACHE BOOL "Standard fortran BLAS API")

# BLAS and LAPACK version (for LA=EXTERNAL_BLAS_WRAPPER in BLASFEO or for BLASFEO_BENCHMARS=ON)
set(EXTERNAL_BLAS 0 CACHE STRING "Reference blas to use")
# set(EXTERNAL_BLAS OPENBLAS CACHE STRING "Reference blas to use")
# set(EXTERNAL_BLAS NETLIB CACHE STRING "Reference blas to use")
# set(EXTERNAL_BLAS MKL CACHE STRING "Reference blas to use")
# set(EXTERNAL_BLAS BLIS CACHE STRING "Reference blas to use")
# set(EXTERNAL_BLAS ATLAS CACHE STRING "Reference blas to use")
# set(EXTERNAL_BLAS ARMPL CACHE STRING "Reference blas to use")

# Stack buffer size (malloc for larger)
set(K_MAX_STACK 300 CACHE STRING "Maximum k value using stack memory")

# Macro level (code size vs performance in assembly kernels): 0 (no macro),
# 1 (all macro but gemm kernel), 2 (all macro)
set(MACRO_LEVEL 1)

# Use C99 extension to math library
set(USE_C99_MATH ON CACHE BOOL "Use C99 extension to math library")

# Compile auxiliary functions with external dependencies
# (for memory allocation and printing)
set(EXT_DEP ON CACHE BOOL "Compile external dependencies in BLASFEO")

# Options
# enable runtine checks
set(RUNTIME_CHECKS OFF)

# set(BLASFEO_TESTING ON CACHE BOOL "Tests enabled")
set(BLASFEO_TESTING OFF CACHE BOOL "Tests disabled")

# set(BLASFEO_BENCHMARKS ON CACHE BOOL "Benchmarks enabled")
set(BLASFEO_BENCHMARKS OFF CACHE BOOL "Benchmarks disabled")

set(BLASFEO_EXAMPLES ON CACHE BOOL "Examples enabled")
# set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples disabled")

# build shared library
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries")

# headers installation directory
set(BLASFEO_HEADERS_INSTALLATION_DIRECTORY "include" CACHE STRING "Headers local installation directory")



# Populate a list of allowable targets and link it to the option
set(ALLOWED_TARGETS
		X64_AUTOMATIC
		X64_INTEL_HASWELL
		X64_INTEL_SANDY_BRIDGE
		X64_INTEL_CORE
		X64_AMD_BULLDOZER
		ARMV8A_ARM_CORTEX_A57
		ARMV8A_ARM_CORTEX_A53
		ARMV7A_ARM_CORTEX_A15
		ARMV7A_ARM_CORTEX_A9
		ARMV7A_ARM_CORTEX_A7
		GENERIC
		)
set_property(CACHE TARGET PROPERTY STRINGS ${ALLOWED_TARGETS})

# This list contains the targets to try when the user sets the target to
# X64_AUTOMATIC. The order they appear in this list is the order in which
# they are tested, and the first working (both compile and run) will be
# selected for use
set(X64_AUTOMATIC_TARGETS
		X64_INTEL_HASWELL
		X64_AMD_BULLDOZER
		X64_INTEL_SANDY_BRIDGE
		X64_INTEL_CORE
		GENERIC
		)


list(FIND ALLOWED_TARGETS ${TARGET} isvalid)
if(${isvalid} EQUAL -1)
	message(FATAL_ERROR "Target ${TARGET} is not supported")
endif()


# Populate the list of allowed LA backends and link it to the option
set(ALLOWED_BACKENDS
	  HIGH_PERFORMANCE
	  REFERENCE
	  EXTERNAL_BLAS_WRAPPER
	  )
set_property(CACHE LA PROPERTY STRINGS ${ALLOWED_BACKENDS})

list(FIND ALLOWED_BACKENDS ${LA} isvalid)
if(${isvalid} EQUAL -1)
	message(FATAL_ERROR "LA backend ${LA} is not supported")
endif()


# Populate the list of allowed Matrix Formats and link it to the option
set(ALLOWED_MATRIX_FORMAT
	  COLMAJ
	  PANELMAJ
	  )
set_property(CACHE MF PROPERTY STRINGS ${ALLOWED_MATRIX_FORMAT})

list(FIND ALLOWED_MATRIX_FORMAT ${MF} isvalid)
if(${isvalid} EQUAL -1)
	message(FATAL_ERROR "Matrix Format ${MF} is not supported")
endif()

if(${LA} MATCHES HIGH_PERFORMANCE)
	if(${BLASFEO_REF_API})
		if(${MF} MATCHES COLMAJ)
			message(FATAL_ERROR "As of now, with LA=HIGH_PERFORMANCE only MF=PANELMAJ is supported for the additional BLASFEO_REF_API")
		endif()
	endif()
endif()

# Populate the list of allowed exernal BLAS tools and link it to the option
set(ALLOWED_EXTERNAL_BLAS
		0
		OPENBLAS
		NETLIB
		MKL
		BLIS
		ATLAS
		ARMPL
		)
set_property(CACHE EXTERNAL_BLAS PROPERTY STRINGS ${ALLOWED_EXTERNAL_BLAS})

list(FIND ALLOWED_EXTERNAL_BLAS ${EXTERNAL_BLAS} isvalid)
if(${isvalid} EQUAL -1)
	message(FATAL_ERROR "External BLAS ${EXTERNAL_BLAS} is not supported")
endif()

# Pass the allowed items up to the parent scope (if there is one)
# This is detected by comparing the project name to the one set earlier in this file
if(NOT CMAKE_PROJECT_NAME STREQUAL blasfeo)
	set(BLASFEO_ALLOWED_TARGETS ${ALLOWED_TARGETS} PARENT_SCOPE)
	set(BLASFEO_ALLOWED_BACKENDS ${ALLOWED_BACKENDS} PARENT_SCOPE)
	set(BLASFEO_ALLOWED_EXTERNAL_BLAS ${ALLOWED_EXTERNAL_BLAS} PARENT_SCOPE)
endif()

if(NOT DEFINED(BLASFEO_CROSSCOMPILING))
	set(BLASFEO_CROSSCOMPILING ${CMAKE_CROSSCOMPILING})
endif()

# Skip BLASFEO examples for unsupported systems and for Realease build
if(CMAKE_SYSTEM MATCHES "dSpace") # OR CMAKE_BUILD_TYPE MATCHES "Release")
	set(BLASFEO_TESTING OFF CACHE BOOL "Examples disabled" FORCE)
	set(BLASFEO_BENCHMARKS OFF CACHE BOOL "Benchmarks disabled" FORCE)
	set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples disabled" FORCE)
endif()

# C Compiler
# set(CC_COMPILER gcc CACHE STRING "compiler")
# set(CC_COMPILER clang)
# set(CC_COMPILER x86_64-w64-mingw32-gcc)
# set(CMAKE_C_COMPILER clang)
if(CMAKE_C_COMPILER_ID MATCHES MSVC)
	if(NOT ${TARGET} MATCHES GENERIC)
		message( FATAL_ERROR "MSVC compiler only supported for TARGET=GENERIC")
	endif()
endif()

# testing
#if(BLASFEO_TESTING MATCHES ON)
#	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTESTING_MODE=1")
#endif()

# search directories
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${BLASFEO_PATH}/include")

# Specify the linear algebra backend to use
message(STATUS "Using linear algebra: ${LA}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DLA_${LA}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DLA_${LA}")

if(${BLASFEO_REF_API})
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBLASFEO_REF_API")
endif()

# Specify the matrix format to use
message(STATUS "Using matrix format: ${MF}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMF_${MF}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DMF_${MF}")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DK_MAX_STACK=${K_MAX_STACK}")

#
if(${BLAS_API})
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBLAS_API")
	set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DBLAS_API")
	if(${FORTRAN_BLAS_API})
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DFORTRAN_BLAS_API")
	endif()
endif()

#
if(${RUNTIME_CHECKS})
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DDIM_CHECK")
endif()

#
if(${USE_C99_MATH})
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_C99_MATH")
endif()

#
if(${EXT_DEP})
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DEXT_DEP")
endif()

#
if(${MACRO_LEVEL} MATCHES 1)
	set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DMACRO_LEVEL=1")
endif()
if(${MACRO_LEVEL} MATCHES 2)
	set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DMACRO_LEVEL=2")
endif()

# Detect the system
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOS_LINUX")
	set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DOS_LINUX")
endif()
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOS_MAC")
	set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DOS_MAC")
endif()
if(CMAKE_SYSTEM_NAME MATCHES "Windows" OR CMAKE_SYSTEM_NAME MATCHES "CYGWIN")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOS_WINDOWS")
	set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DOS_WINDOWS")
	if(${BUILD_SHARED_LIBS} MATCHES ON)
		set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
	endif()
endif()

# External BLAS flags
set(C_FLAGS_BLAS_0        "-DEXTERNAL_BLAS_NONE")
set(C_FLAGS_BLAS_OPENBLAS "-DEXTERNAL_BLAS_OPENBLAS -I/opt/openblas/include")
set(C_FLAGS_BLAS_BLIS     "-DEXTERNAL_BLAS_BLIS -std=gnu99 -I/opt/blis/include/blis")
set(C_FLAGS_BLAS_NETLIB   "-DEXTERNAL_BLAS_NETLIB -I/opt/netlib/include")
set(C_FLAGS_BLAS_MKL      "-DEXTERNAL_BLAS_MKL -std=c99 -m64 -DMKL_DIRECT_CALL_SEQ -I/opt/intel/mkl/include")
set(C_FLAGS_BLAS_ATLAS    "-DEXTERNAL_BLAS_ATLAS")
set(C_FLAGS_BLAS_ARMPL    "-DEXTERNAL_BLAS_ARMPL -I/opt/arm/armpl_20.3_gcc-7.1/include")

message(STATUS "Using external BLAS: ${EXTERNAL_BLAS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_BLAS_${EXTERNAL_BLAS}}")


# common C flags
if(CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
	if(NOT CMAKE_C_SIMULATE_ID MATCHES "MSVC") # See https://stackoverflow.com/questions/49480535
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC") # Windows is always position independent code
	endif()
endif()


# architecture-specific C flags
set(C_FLAGS_TARGET_X64_INTEL_HASWELL      "-m64 -mavx -mavx2 -mfma")
set(C_FLAGS_TARGET_X64_INTEL_SANDY_BRIDGE "-m64 -mavx")
set(C_FLAGS_TARGET_X64_INTEL_CORE         "-m64 -msse3")
set(C_FLAGS_TARGET_X64_AMD_BULLDOZER      "-m64 -mavx -mfma")
set(C_FLAGS_TARGET_ARMV8A_ARM_CORTEX_A57  "-march=armv8-a+crc+crypto+simd -mcpu=cortex-a57")
set(C_FLAGS_TARGET_ARMV8A_ARM_CORTEX_A53  "-march=armv8-a+crc+crypto+simd -mcpu=cortex-a53")
set(C_FLAGS_TARGET_ARMV7A_ARM_CORTEX_A15  "-marm -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a15")
set(C_FLAGS_TARGET_ARMV7A_ARM_CORTEX_A9   "-marm -mfloat-abi=hard -mfpu=neon -mcpu=cortex-a9")
set(C_FLAGS_TARGET_ARMV7A_ARM_CORTEX_A7   "-marm -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a7")
set(C_FLAGS_TARGET_GENERIC                "")

# architecture-specific assembly flags
set(ASM_FLAGS_TARGET_X64_INTEL_HASWELL      "")
set(ASM_FLAGS_TARGET_X64_INTEL_SANDY_BRIDGE "")
set(ASM_FLAGS_TARGET_X64_INTEL_CORE         "")
set(ASM_FLAGS_TARGET_X64_AMD_BULLDOZER      "")
set(ASM_FLAGS_TARGET_ARMV8A_ARM_CORTEX_A57  "")
set(ASM_FLAGS_TARGET_ARMV8A_ARM_CORTEX_A53  "")
set(ASM_FLAGS_TARGET_ARMV7A_ARM_CORTEX_A15  "-mfpu=neon-vfpv4 -mcpu=cortex-a15")
set(ASM_FLAGS_TARGET_ARMV7A_ARM_CORTEX_A9   "-mfpu=neon -mcpu=cortex-a9")
set(ASM_FLAGS_TARGET_ARMV7A_ARM_CORTEX_A7   "-mfpu=neon-vfpv4 -mcpu=cortex-a7")
set(ASM_FLAGS_TARGET_GENERIC                "")


if(${TARGET} MATCHES X64_AUTOMATIC)
	include(${PROJECT_SOURCE_DIR}/cmake/X64AutomaticTargetSelection.cmake)
	X64AutomaticTargetSelection()
	message(STATUS "Detected target ${TARGET}")
else()
	include(${PROJECT_SOURCE_DIR}/cmake/TestSingleTarget.cmake)
	TestSingleTarget()
	message(STATUS "Compiling for target: ${TARGET}")
endif()



if(${TARGET} MATCHES X64_INTEL_HASWELL)
  set(TARGET_NEED_FEATURE_AVX2 1)
  set(TARGET_NEED_FEATURE_FMA  1)
endif()

if(${TARGET} MATCHES X64_INTEL_SANDY_BRIDGE)
  set(TARGET_NEED_FEATURE_AVX 1)
endif()

if(${TARGET} MATCHES X64_INTEL_CORE)
  set(TARGET_NEED_FEATURE_SSE3 1)
endif()

if(${TARGET} MATCHES X64_AMD_BULLDOZER)
  set(TARGET_NEED_FEATURE_AVX 1)
  set(TARGET_NEED_FEATURE_FMA 1)
endif()

if(${TARGET} MATCHES ARMV8A_ARM_CORTEX_A57)
  set(TARGET_NEED_FEATURE_VFPv4 1)
  set(TARGET_NEED_FEATURE_NEONv2 1)
endif()

if(${TARGET} MATCHES ARMV8A_ARM_CORTEX_A53)
  set(TARGET_NEED_FEATURE_VFPv4 1)
  set(TARGET_NEED_FEATURE_NEONv2 1)
endif()

if(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A15)
  set(TARGET_NEED_FEATURE_VFPv3 1)
  set(TARGET_NEED_FEATURE_NEON 1)
endif()

if(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A7)
  set(TARGET_NEED_FEATURE_VFPv3 1)
  set(TARGET_NEED_FEATURE_NEON 1)
endif()

if(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A9)
  set(TARGET_NEED_FEATURE_VFPv3 1)
  set(TARGET_NEED_FEATURE_NEON 1)
endif()


# Create the target.h file with the proper target definition
configure_file(${PROJECT_SOURCE_DIR}/blasfeo_target.h.in
	${CMAKE_CURRENT_SOURCE_DIR}/include/blasfeo_target.h @ONLY)


# Append the appropriate flags based on the architecture and compiler
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTARGET_${TARGET}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DTARGET_${TARGET}")

if(CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_TARGET_${TARGET}}")
	set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${ASM_FLAGS_TARGET_${TARGET}}")
endif()



# source files

file(GLOB AUX_COMMON_SRC
	${PROJECT_SOURCE_DIR}/auxiliary/blasfeo_processor_features.c
	${PROJECT_SOURCE_DIR}/auxiliary/blasfeo_stdlib.c
	${PROJECT_SOURCE_DIR}/auxiliary/memory.c
	${PROJECT_SOURCE_DIR}/auxiliary/d_aux_common.c
	${PROJECT_SOURCE_DIR}/auxiliary/s_aux_common.c
	)

file(GLOB AUX_EXT_DEP_SRC
	${PROJECT_SOURCE_DIR}/auxiliary/d_aux_ext_dep_common.c
	${PROJECT_SOURCE_DIR}/auxiliary/s_aux_ext_dep_common.c
	${PROJECT_SOURCE_DIR}/auxiliary/d_aux_ext_dep.c
	${PROJECT_SOURCE_DIR}/auxiliary/s_aux_ext_dep.c
	${PROJECT_SOURCE_DIR}/auxiliary/v_aux_ext_dep_lib.c
	${PROJECT_SOURCE_DIR}/auxiliary/i_aux_ext_dep_lib.c
	${PROJECT_SOURCE_DIR}/auxiliary/timing.c
	)

file(GLOB AUX_REF_SRC
	${PROJECT_SOURCE_DIR}/auxiliary/d_aux_ref.c
	${PROJECT_SOURCE_DIR}/auxiliary/s_aux_ref.c
	)

file(GLOB BLASFEO_REF_SRC
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas1_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas2_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas2_diag_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas3_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas3_diag_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_lapack_ref.c

	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas1_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas2_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas2_diag_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas3_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas3_diag_ref.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_lapack_ref.c
	)

file(GLOB AUX_HP_CM_SRC
	${PROJECT_SOURCE_DIR}/auxiliary/d_aux_hp_cm.c
	${PROJECT_SOURCE_DIR}/auxiliary/s_aux_hp_cm.c
	)

file(GLOB BLASFEO_HP_CM_SRC
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dgemm.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dsyrk.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dtrsm.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dtrmm.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dsyr2k.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dpotrf.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dgetrf.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dgetr.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dgemv.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dsymv.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/dger.c

	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/sgemm.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/strsm.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_cm/spotrf.c
	)

file(GLOB BLASFEO_HP_CM_REF_SRC
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas1_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas2_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas2_diag_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas3_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas3_diag_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_lapack_hp_cm.c

	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas1_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas2_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas2_diag_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas3_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas3_diag_hp_cm.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_lapack_hp_cm.c
	)

file(GLOB BLASFEO_WR_SRC
	${PROJECT_SOURCE_DIR}/blasfeo_wr/d_blas1_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/d_blas2_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/d_blas2_diag_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/d_blas3_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/d_blas3_diag_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/d_lapack_lib.c

	${PROJECT_SOURCE_DIR}/blasfeo_wr/s_blas1_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/s_blas2_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/s_blas2_diag_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/s_blas3_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/s_blas3_diag_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_wr/s_lapack_lib.c
	)

#list(APPEND BLAS_SRC
file(GLOB BLAS_SRC
	${PROJECT_SOURCE_DIR}/blas_api/dcopy.c
	${PROJECT_SOURCE_DIR}/blas_api/daxpy.c
	${PROJECT_SOURCE_DIR}/blas_api/ddot.c
	${PROJECT_SOURCE_DIR}/blas_api/dgemm_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dsyrk_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dtrmm_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dtrsm_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dsyr2k_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dgesv.c
	${PROJECT_SOURCE_DIR}/blas_api/dgetrf_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dgetrs.c
	${PROJECT_SOURCE_DIR}/blas_api/dlaswp.c
	${PROJECT_SOURCE_DIR}/blas_api/dposv.c
	${PROJECT_SOURCE_DIR}/blas_api/dpotrf_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dpotrs.c
	${PROJECT_SOURCE_DIR}/blas_api/dtrtrs.c
	${PROJECT_SOURCE_DIR}/blas_api/dgetr_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dsymv_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dgemv_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/dger_ref.c # XXX

	${PROJECT_SOURCE_DIR}/blas_api/saxpy.c
	${PROJECT_SOURCE_DIR}/blas_api/sdot.c
	${PROJECT_SOURCE_DIR}/blas_api/sgemm_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/strsm_ref.c # XXX
	${PROJECT_SOURCE_DIR}/blas_api/spotrf_ref.c # XXX
	)

file(GLOB REF_BLAS_SRC
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_blas3_ref_blas.c
	${PROJECT_SOURCE_DIR}/blasfeo_ref/d_lapack_ref_blas.c

	${PROJECT_SOURCE_DIR}/blasfeo_ref/s_blas3_ref_blas.c
	)

file(GLOB BLAS_CM_SRC
	${PROJECT_SOURCE_DIR}/blas_api/dgemm_ref.c
	${PROJECT_SOURCE_DIR}/blas_api/dsyrk_ref.c
	${PROJECT_SOURCE_DIR}/blas_api/dtrmm_ref.c
	${PROJECT_SOURCE_DIR}/blas_api/dtrsm_ref.c
	${PROJECT_SOURCE_DIR}/blas_api/dpotrf_ref.c
	${PROJECT_SOURCE_DIR}/blas_api/dgetrf_ref.c

	${PROJECT_SOURCE_DIR}/blas_api/sgemm_ref.c
	)

if(${TARGET} MATCHES X64_INTEL_HASWELL OR ${TARGET} MATCHES X64_INTEL_SANDY_BRIDGE)

file(GLOB BLASFEO_HP_PM_SRC
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas1_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas2_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas2_diag_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas3_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas3_diag_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_lapack_lib4.c

	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas1_lib8.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas2_lib8.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas2_diag_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas3_lib8.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas3_diag_lib8.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_lapack_lib8.c
	)

file(GLOB AUX_HP_PM_SRC
	${PROJECT_SOURCE_DIR}/auxiliary/d_aux_lib4.c
	${PROJECT_SOURCE_DIR}/auxiliary/s_aux_lib8.c
	${PROJECT_SOURCE_DIR}/auxiliary/m_aux_lib48.c
	)

endif()
if(${TARGET} MATCHES X64_INTEL_CORE OR ${TARGET} MATCHES X64_AMD_BULLDOZER OR ${TARGET} MATCHES ARMV8A_ARM_CORTEX_A57 OR ${TARGET} MATCHES ARMV8A_ARM_CORTEX_A53 OR ${TARGET} MATCHES ARMV7A_ARM_CORTEX_A15 OR ${TARGET} MATCHES ARMV7A_ARM_CORTEX_A7 OR ${TARGET} MATCHES ARMV7A_ARM_CORTEX_A9 OR ${TARGET} MATCHES GENERIC)

file(GLOB BLASFEO_HP_PM_SRC
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas1_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas2_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas2_diag_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas3_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_blas3_diag_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/d_lapack_lib4.c

	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas1_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas2_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas2_diag_lib.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas3_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_blas3_diag_lib4.c
	${PROJECT_SOURCE_DIR}/blasfeo_hp_pm/s_lapack_lib4.c
	)

file(GLOB AUX_HP_PM_SRC
	${PROJECT_SOURCE_DIR}/auxiliary/d_aux_lib4.c
	${PROJECT_SOURCE_DIR}/auxiliary/s_aux_lib4.c
	${PROJECT_SOURCE_DIR}/auxiliary/m_aux_lib44.c
	)

endif()

if(${TARGET} MATCHES X64_INTEL_HASWELL)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgemm_12x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgemm_8x8_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgemv_8_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgemv_4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dsymv_6_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dger_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgebp_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgelqf_4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dpack_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgetr_lib.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_sgemm_24x4_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_sgemm_16x4_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_sgemm_8x8_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx2/kernel_sgemm_8x4_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemm_diag_lib8.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemv_8_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemv_4_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgecpsc_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgead_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgetr_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_spack_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_8x4_lib8.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/sse3/kernel_align_x64.S

	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_d_aux_lib.S
	)

endif(${TARGET} MATCHES X64_INTEL_HASWELL)
if(${TARGET} MATCHES X64_INTEL_SANDY_BRIDGE)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgemm_12x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgemv_12_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgemv_8_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgemv_4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dsymv_6_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dger_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgebp_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dpack_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_dgetr_lib.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemm_16x4_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemm_8x8_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemm_8x4_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemm_diag_lib8.c
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemv_8_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgemv_4_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgecpsc_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgetr_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_sgead_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_spack_lib8.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_8x4_lib8.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/sse3/kernel_align_x64.S

	${PROJECT_SOURCE_DIR}/kernel/avx/kernel_d_aux_lib.S
	)

endif(${TARGET} MATCHES X64_INTEL_SANDY_BRIDGE)
if(${TARGET} MATCHES X64_INTEL_CORE)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/sse3/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/sse3/kernel_dgemv_4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib.c

	${PROJECT_SOURCE_DIR}/kernel/sse3/kernel_sgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ssymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/sse3/kernel_align_x64.S

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_d_aux_lib.c
	)

endif(${TARGET} MATCHES X64_INTEL_CORE)
if(${TARGET} MATCHES X64_AMD_BULLDOZER)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/fma/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ssymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/sse3/kernel_align_x64.S

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_d_aux_lib.c
	)

endif(${TARGET} MATCHES X64_AMD_BULLDOZER)
if(${TARGET} MATCHES ARMV8A_ARM_CORTEX_A57)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dpack_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgetrf_pivot_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgemv_4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgetr_lib.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib.c

	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_16x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_12x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_8x8_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemv_4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_spack_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ssymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_align_generic.c

	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_d_aux_lib.S
	)

endif(${TARGET} MATCHES ARMV8A_ARM_CORTEX_A57)
if(${TARGET} MATCHES ARMV8A_ARM_CORTEX_A53)

file(GLOB KERNEL_SRC

	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgemm_12x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dpack_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgetrf_pivot_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgemv_4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dgetr_lib.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib.c

	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_16x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_12x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_8x8_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_sgemv_4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_spack_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ssymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_align_generic.c

	${PROJECT_SOURCE_DIR}/kernel/armv8a/kernel_d_aux_lib.S
	)

endif(${TARGET} MATCHES ARMV8A_ARM_CORTEX_A53)
if(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A15)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib.c

	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_sgemm_12x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_sgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_sgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ssymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_align_generic.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_d_aux_lib.c
	)

endif(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A15)
if(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A7)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib.c

	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_sgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_sgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ssymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_align_generic.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_d_aux_lib.c
	)

endif(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A7)
if(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A9)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_dgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib.c

	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_sgemm_8x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/armv7a/kernel_sgemm_4x4_lib4.S
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ssymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_align_generic.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_d_aux_lib.c
	)

endif(${TARGET} MATCHES ARMV7A_ARM_CORTEX_A9)
if(${TARGET} MATCHES GENERIC)

file(GLOB KERNEL_SRC
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dsymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgeqrf_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dpack_buffer_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dger_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ddot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_daxpy_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_dgetr_lib.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_4x4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemm_diag_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgemv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_ssymv_4_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetrf_pivot_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgecp_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sgetr_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_spack_lib4.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_sdot_lib.c
	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_saxpy_lib.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_align_generic.c

	${PROJECT_SOURCE_DIR}/kernel/generic/kernel_d_aux_lib.c
	)

endif(${TARGET} MATCHES GENERIC)



set(BLASFEO_SRC ${AUX_COMMON_SRC})

if(${LA} MATCHES HIGH_PERFORMANCE)

	list(APPEND BLASFEO_SRC ${KERNEL_SRC})

	if(${MF} MATCHES PANELMAJ)

		list(APPEND BLASFEO_SRC ${AUX_HP_PM_SRC})
		list(APPEND BLASFEO_SRC ${BLASFEO_HP_PM_SRC})

	else()

		list(APPEND BLASFEO_SRC ${AUX_HP_CM_SRC})
		list(APPEND BLASFEO_SRC ${BLASFEO_HP_CM_SRC})
		list(APPEND BLASFEO_SRC ${BLASFEO_HP_CM_REF_SRC})

	endif()

	if(${BLASFEO_REF_API})

		list(APPEND BLASFEO_SRC ${AUX_REF_SRC})
		list(APPEND BLASFEO_SRC ${BLASFEO_REF_SRC})

	endif()

	if(${BLAS_API})

		if(${MF} MATCHES PANELMAJ)

			list(APPEND BLASFEO_SRC ${BLAS_SRC})
			list(APPEND BLASFEO_SRC ${BLASFEO_HP_CM_SRC})

		else()

			list(APPEND BLASFEO_SRC ${BLAS_CM_SRC})

		endif()

	endif()

endif()
if(${LA} MATCHES REFERENCE)

	list(APPEND BLASFEO_SRC ${AUX_REF_SRC})
	list(APPEND BLASFEO_SRC ${BLASFEO_REF_SRC})

	if(${BLASFEO_HP_API})

		list(APPEND BLASFEO_SRC ${KERNEL_SRC})

		if(${MF} MATCHES PANELMAJ)

			list(APPEND BLASFEO_SRC ${BLASFEO_HP_PM_SRC})

		else()

			list(APPEND BLASFEO_SRC ${BLASFEO_HP_CM_SRC})
			list(APPEND BLASFEO_SRC ${BLASFEO_HP_CM_REF_SRC})

		endif()

	endif()

	if(${BLAS_API})

		list(APPEND BLASFEO_SRC ${BLAS_SRC})

		if(${MF} MATCHES PANELMAJ)

			list(APPEND BLASFEO_SRC ${REF_BLAS_SRC})

		endif()

	endif()

endif()

# TODO WR

if(${EXT_DEP})
	
	list(APPEND BLASFEO_SRC ${AUX_EXT_DEP_SRC})

endif()







#set(BLASFEO_SRC ${CMN_SRC} ${AUX_SRC} ${KERNEL_SRC} ${BLAS_SRC} ${EXT_SRC})

# add library
add_library(blasfeo ${BLASFEO_SRC})

# 1) *XILINX_NONE_ELF*:
#		Xilinx standalone (any core, e.g. A53,R5,..; *untested*)
# 2) *XILINX_ULTRASCALE_NONE_ELF_JAILHOUSE*:
#		special Ultrascale+ targets solution for running standalone as
#		an inmate of a Jailhouse cell (no secure domain registers)
if(SOC MATCHES XILINX)
	message(STATUS "Compiling for SOC: ${SOC}")

	if(NOT XILINX_BSP_PATH)
		message(FATAL_ERROR "XILINX_BSP_PATH not set")
	endif()
	include_directories(${XILINX_BSP_PATH}/include)

	find_library(XIL NAMES libxil.a HINTS ${XILINX_BSP_PATH}/lib)
	if(NOT XIL)
	  message(FATAL_ERROR "libxil.a not found")
	endif()

	if(SOC MATCHES XILINX_ULTRASCALE_NONE_ELF_JAILHOUSE )
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__XILINX_ULTRASCALE_NONE_ELF_JAILHOUSE__")
	elseif(SOC MATCHES XILINX_NONE_ELF )
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__XILINX_NONE_ELF__")
	endif()

	target_link_libraries(blasfeo PUBLIC -Wl,--start-group ${XIL} c gcc -Wl,--end-group)
endif()

target_include_directories(blasfeo
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${BLASFEO_HEADERS_INSTALLATION_DIRECTORY}>)
#		$<INSTALL_INTERFACE:include/blasfeo/include>)


install(TARGETS blasfeo EXPORT blasfeoConfig
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin)


install(EXPORT blasfeoConfig DESTINATION cmake)

file(GLOB_RECURSE BLASFEO_HEADERS "include/*.h")
install(FILES ${BLASFEO_HEADERS} DESTINATION ${BLASFEO_HEADERS_INSTALLATION_DIRECTORY})

# tests
if(BLASFEO_TESTING MATCHES ON)
	add_subdirectory(tests)
endif()

# benchmarks
if(BLASFEO_BENCHMARKS MATCHES ON)
	add_subdirectory(benchmarks)
endif()

# examples
if(BLASFEO_EXAMPLES MATCHES ON)
	add_subdirectory(examples)
endif()
