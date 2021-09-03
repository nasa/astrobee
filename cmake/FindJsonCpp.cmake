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

# Find jsoncpp
#
# Find the jsoncpp includes and library
# 
# if you nee to add a custom library search path, do it via via CMAKE_PREFIX_PATH 
# 
# This module defines
#  JSONCPP_INCLUDE_DIRS, where to find header, etc.
#  JSONCPP_LIBRARIES, the libraries needed to use jsoncpp.
#  JSONCPP_FOUND, If false, do not try to use jsoncpp.
#  JSONCPP_INCLUDE_PREFIX, include prefix for jsoncpp

# only look in default directories
find_path(
	JSONCPP_INCLUDE_DIR 
	NAMES jsoncpp/json/json.h json/json.h
	DOC "jsoncpp include dir"
)
if (EXISTS "${JSONCPP_INCLUDE_DIR}/jsoncpp/json/json.h" )
  set (JSONCPP_INCLUDE_DIR "${JSONCPP_INCLUDE_DIR}/jsoncpp")
endif ()

find_library(
	JSONCPP_LIBRARY
	NAMES jsoncpp
	DOC "jsoncpp library"
)

set(JSONCPP_INCLUDE_DIRS ${JSONCPP_INCLUDE_DIR})
set(JSONCPP_LIBRARIES ${JSONCPP_LIBRARY})

# debug library on windows
# same naming convention as in qt (appending debug library with d)
# boost is using the same "hack" as us with "optimized" and "debug"
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	find_library(
		JSONCPP_LIBRARY_DEBUG
		NAMES jsoncppd
		DOC "jsoncpp debug library"
	)
	
	set(JSONCPP_LIBRARIES optimized ${JSONCPP_LIBRARIES} debug ${JSONCPP_LIBRARY_DEBUG})

endif()

# find JSONCPP_INCLUDE_PREFIX
find_path(
	JSONCPP_INCLUDE_PREFIX
	NAMES json.h
	PATH_SUFFIXES jsoncpp/json json
)

if (${JSONCPP_INCLUDE_PREFIX} MATCHES "jsoncpp")
	set(JSONCPP_INCLUDE_PREFIX "jsoncpp/json")
else()
	set(JSONCPP_INCLUDE_PREFIX "json")
endif()

# handle the QUIETLY and REQUIRED arguments and set JSONCPP_FOUND to TRUE
# if all listed variables are TRUE, hide their existence from configuration view
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(jsoncpp DEFAULT_MSG
  JSONCPP_INCLUDE_DIRS JSONCPP_LIBRARIES)
mark_as_advanced (JSONCPP_INCLUDE_DIRS JSONCPP_LIBRARIES)

