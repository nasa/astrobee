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

# - Find Ceres library
# Find the native Ceres includes and library
# This module defines
#  CERES_INCLUDE_DIRS, where to find ceres.h, Set when
#                      CERES_INCLUDE_DIR is found.
#  CERES_LIBRARIES, libraries to link against to use Ceres.
#  CERES_ROOT_DIR, The base directory to search for Ceres.
#                  This can also be an environment variable.
#  CERES_FOUND, If false, do not try to use Ceres.
#
# also defined, but not for general use are
#  CERES_LIBRARY, where to find the Ceres library.

# If CERES_ROOT_DIR was defined in the environment, use it.
IF(NOT CERES_ROOT_DIR AND NOT $ENV{CERES_ROOT_DIR} STREQUAL "")
  SET(CERES_ROOT_DIR $ENV{CERES_ROOT_DIR})
ENDIF()

SET(_ceres_SEARCH_DIRS
  ${CERES_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/ceres
)

FIND_PATH(CERES_INCLUDE_DIR
  NAMES
    ceres/ceres.h
  HINTS
    ${_ceres_SEARCH_DIRS}
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(CERES_LIBRARY
  NAMES
    ceres
  HINTS
    ${_ceres_SEARCH_DIRS}
  PATH_SUFFIXES
    lib64 lib
  )

# handle the QUIETLY and REQUIRED arguments and set CERES_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ceres DEFAULT_MSG
    CERES_LIBRARY CERES_INCLUDE_DIR)

IF(CERES_FOUND)
  SET(CERES_LIBRARIES ${CERES_LIBRARY})
  SET(CERES_INCLUDE_DIRS ${CERES_INCLUDE_DIR})
ENDIF(CERES_FOUND)

MARK_AS_ADVANCED(
  CERES_INCLUDE_DIR
  CERES_LIBRARY
)
