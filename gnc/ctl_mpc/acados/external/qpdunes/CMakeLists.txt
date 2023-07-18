##
##	This file is part of qp42.
##
##	qp42 -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al.
##	All rights reserved.
##
##	qp42 is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qp42 is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qp42; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##

################################################################################
#
# Description:
#	CMake script for building qpDUNES static libraries, etc.
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013 - 2014.
#
# Usage:
#	- mkdir build && cd build && cmake .. && make && make test
#   - For installation of the library and corr. headers: sudo make install
#
################################################################################

################################################################################
#
# Project settings
#
################################################################################

#
# Minimum required version of cmake
#
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

#
# Project name and programming languages used
#
PROJECT( qpDUNES C CXX )

#
# Folder path for generated executables
#
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

#
# Folder path for generated libraries
#
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )

#
# CMake module(s) path
#
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

################################################################################
#
# User configuration
#
################################################################################

#
# Build type
#
IF( NOT CMAKE_BUILD_TYPE )
	SET(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		FORCE
	)
ENDIF( NOT CMAKE_BUILD_TYPE )

OPTION( QPDUNES_SIMPLE_BOUNDS_ONLY
	"Enable compilation of the library for simple bounds only"
	OFF
)

OPTION( QPDUNES_USE_ASSERTS
	"Check return values of routines rigorously (useful for debugging)"
	OFF
)

OPTION( QPDUNES_SUPPRESS_ALL_OUTPUT
	"No printing"
	OFF
)

OPTION( QPDUNES_SUPPRESS_ALL_WARNINGS
	"Do not display warnings"
	OFF
)


OPTION( QPDUNES_MEASURE_TIMINGS
	"Measure computation times"
	OFF
)

OPTION( QPDUNES_ANALYZE_FACTORIZATION
	"Log inverse Newton Hessian for analysis"
	OFF
)

OPTION( QPDUNES_FULL_LOGGING
	"Enable full logging"
	ON
)

OPTION( QPDUNES_PARALLEL
	"Use openMP parallelization"
	OFF
)

OPTION( QPDUNES_WITH_LAPACK
	"Build qpOASES using original LAPACK routines"
	OFF
)

OPTION( QPDUNES_WITH_BLAS
	"Build qpOASES using original BLAS routines"
	OFF
)

OPTION( QPDUNES_ACADOS
	"Do not build qpDUNES examples"
	ON
)

################################################################################
#
# Compiler settings
#
################################################################################

if(CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -fPIC -Wall -Wextra -pedantic -Wshadow -g -finline-functions -std=c99" )
endif()

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D__DEBUG__")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")

IF ( QPDUNES_ACADOS )
	SET( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -w")
ENDIF()

IF ( QPDUNES_SIMPLE_BOUNDS_ONLY )
   ADD_DEFINITIONS( -D__SIMPLE_BOUNDS_ONLY__ )
ENDIF()

IF ( QPDUNES_USE_ASSERTS )
   ADD_DEFINITIONS( -D__USE_ASSERTS__ )
ENDIF()

IF ( QPDUNES_SUPPRESS_ALL_OUTPUT )
   ADD_DEFINITIONS( -D__SUPPRESS_ALL_OUTPUT__ )
ENDIF()

IF ( QPDUNES_SUPPRESS_ALL_WARNINGS )
   ADD_DEFINITIONS( -D__SUPPRESS_ALL_WARNINGS__ )
ENDIF()

IF ( QPDUNES_MEASURE_TIMINGS )
   ADD_DEFINITIONS( -D__MEASURE_TIMINGS__ )
ENDIF()

IF ( QPDUNES_ANALYZE_FACTORIZATION )
   ADD_DEFINITIONS( -D__ANALYZE_FACTORIZATION__ )
ENDIF()

IF ( QPDUNES_FULL_LOGGING )
   ADD_DEFINITIONS( -D__FULL_LOGGING__ )
ENDIF()

IF ( QPDUNES_PARALLEL )
	FIND_PACKAGE( OpenMP )
ENDIF()

IF( OPENMP_FOUND )
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
	ADD_DEFINITIONS( -D__QPDUNES_PARALLEL__ )
ENDIF()

# This will add the "make test" target
ENABLE_TESTING()

################################################################################
#
# Sources and headers
#
################################################################################

SET( qpDUNES_HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/include/qpDUNES.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/qp/dual_qp.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/qp/matrix_vector.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/qp/setup_qp.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/qp/stage_qp_solver_clipping.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/qp/types.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/qp/qpdunes_utils.h
	# mpcDUNES
	${CMAKE_CURRENT_SOURCE_DIR}/interfaces/mpc/setup_mpc.h
)

SET( qpDUNES_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/dual_qp.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/stage_qp_solver_clipping.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/matrix_vector.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/setup_qp.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/qpdunes_utils.c
	# mpcDUNES
	${CMAKE_CURRENT_SOURCE_DIR}/interfaces/mpc/setup_mpc.c
)

IF ( NOT QPDUNES_SIMPLE_BOUNDS_ONLY )

   	ENABLE_LANGUAGE( CXX )
	if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Wshadow -finline-functions -fPIC" )
	endif()
	IF ( QPDUNES_ACADOS )
	    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w" )
	ENDIF()
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__" )

	SET ( qpDUNES_HEADERS ${qpDUNES_HEADERS}
		${CMAKE_CURRENT_SOURCE_DIR}/include/qp/stage_qp_solver_qpoases.hpp
	)

	SET ( qpDUNES_SOURCES ${qpDUNES_SOURCES}
		${CMAKE_CURRENT_SOURCE_DIR}/src/stage_qp_solver_qpoases.cpp
	)

	SET( QPOASES_SOURCES
    	${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/Bounds.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/Constraints.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/Flipper.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/Indexlist.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/LoggedSQProblem.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/Matrices.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/MessageHandling.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/Options.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/QProblem.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/QProblemB.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/SQProblem.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/SubjectTo.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/Utils.cpp
	)

	FILE( GLOB QPOASES_HEADERS
		${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/include/*.hpp )

	# Find LAPACK
	IF ( QPDUNES_WITH_LAPACK )
	    ENABLE_LANGUAGE( Fortran )

		FIND_PACKAGE( LAPACK )

		IF ( NOT LAPACK_FOUND )
		   SET( QPOASES_SOURCES ${QPOASES_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/LAPACKReplacement.cpp )
		ENDIF()

	ELSE()
		 SET( QPOASES_SOURCES ${QPOASES_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/LAPACKReplacement.cpp )
    ENDIF()

	# Find BLAS
	IF ( QPDUNES_WITH_BLAS )
		ENABLE_LANGUAGE( Fortran )

		FIND_PACKAGE( BLAS )

		IF ( NOT BLAS_FOUND )
		    SET( QPOASES_SOURCES ${QPOASES_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/BLASReplacement.cpp )
		ENDIF()

	ELSE()
		SET( QPOASES_SOURCES ${QPOASES_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/src/BLASReplacement.cpp )
	ENDIF()

	SET( qpDUNES_HEADERS ${qpDUNES_HEADERS} ${QPOASES_HEADERS} )
	SET( qpDUNES_SOURCES ${qpDUNES_SOURCES} ${QPOASES_SOURCES} )

ENDIF()

################################################################################
#
# Build applications, libraries and examples
#
################################################################################

#
# Build the qpDUNES stand-alone library
#

ADD_LIBRARY(
	qpdunes
	# Sources and headers
	# ${qpDUNES_HEADERS}
	${qpDUNES_SOURCES}
)

TARGET_INCLUDE_DIRECTORIES(qpdunes
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/qp>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interfaces>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interfaces/mpc>
		$<INSTALL_INTERFACE:include/qpdunes>)

IF( NOT QPDUNES_SIMPLE_BOUNDS_ONLY )
	TARGET_INCLUDE_DIRECTORIES(qpdunes
		PUBLIC
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/externals/qpOASES-3.0beta/include>)
ENDIF()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	TARGET_LINK_LIBRARIES( qpdunes m )
endif()

IF ( QPDUNES_WITH_LAPACK AND LAPACK_FOUND )
	TARGET_LINK_LIBRARIES(
		qpdunes
		${LAPACK_LIBRARIES}
	)
ENDIF()

IF ( QPDUNES_WITH_BLAS AND BLAS_FOUND )
	TARGET_LINK_LIBRARIES(
		qpdunes
		${BLAS_LIBRARIES}
	)
ENDIF()

#
# Build the examples
# NOTE: Assumption is that all examples are in C and that one example
# is in exactly one file.
#

IF ( NOT QPDUNES_ACADOS )
	FILE( GLOB qpDUNES_EXAMPLES ${CMAKE_CURRENT_SOURCE_DIR}/examples/*.c )
	FOREACH( EXAMPLE ${qpDUNES_EXAMPLES} )
		GET_FILENAME_COMPONENT( EXEC_NAME ${EXAMPLE} NAME_WE )
		ADD_EXECUTABLE( example_${EXEC_NAME} ${EXAMPLE} )
		TARGET_LINK_LIBRARIES(
			example_${EXEC_NAME}
			qpdunes
		)
		SET_TARGET_PROPERTIES( example_${EXEC_NAME}
			PROPERTIES
			OUTPUT_NAME "${EXEC_NAME}"
		)
		ADD_TEST(
			NAME test_${EXEC_NAME}
			WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
			COMMAND ${EXEC_NAME}
		)
	ENDFOREACH()
ENDIF()

################################################################################
#
# Installation rules
#
################################################################################

INSTALL(
    DIRECTORY
	    include/
    DESTINATION
        include/qpdunes
)

INSTALL(
    FILES
	    interfaces/mpc/setup_mpc.h
    DESTINATION
        include/qpdunes/interfaces/mpc
)

INSTALL(
	TARGETS
		qpdunes
	EXPORT
		qpdunesConfig
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	INCLUDES DESTINATION include
)

install(EXPORT qpdunesConfig DESTINATION cmake)

################################################################################
#
# Configuration of the bash script
#
################################################################################

SET( qpDUNES_ENV_INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/include/qp
	${PROJECT_SOURCE_DIR}/interfaces
	${PROJECT_SOURCE_DIR}/interfaces/mpc
)

IF (NOT QPDUNES_SIMPLE_BOUNDS_ONLY )
   	SET( qpDUNES_ENV_INCLUDE_DIRS
	    ${qpDUNES_ENV_INCLUDE_DIRS}
		${PROJECT_SOURCE_DIR}/externals/qpOASES-3.0beta
		${PROJECT_SOURCE_DIR}/externals/qpOASES-3.0beta/include
	)
ENDIF()

SET( qpDUNES_ENV_LIBRARY_DIRS
	${LIBRARY_OUTPUT_PATH}
)

SET( qpDUNES_ENV_STATIC_LIBRARIES
	qpdunes
)

CONFIGURE_FILE(
	"${PROJECT_SOURCE_DIR}/cmake/qpDUNES_env.sh.in"
	"${PROJECT_BINARY_DIR}/qpDUNES_env.sh"
	@ONLY
)
