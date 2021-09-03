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

###########################################################
#                  Find Flann Library
#----------------------------------------------------------

FIND_PATH(FLANN_DIR flann.hpp
    HINTS "${FLANN_ROOT}" "$ENV{FLANN_ROOT}" "${FLANN_INCLUDE_DIR_HINTS}"
    PATHS "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES flann
    DOC "Root directory of FLANN includes")

##====================================================
## Include FLANN library
##----------------------------------------------------
IF(EXISTS "${FLANN_DIR}" AND NOT "${FLANN_DIR}" STREQUAL "")
        SET(FLANN_FOUND TRUE)
        SET(FLANN_INCLUDE_DIRS ${FLANN_DIR})
        SET(FLANN_DIR "${FLANN_DIR}" CACHE PATH "" FORCE)
        MARK_AS_ADVANCED(FLANN_DIR)

        # Extract Flann version from config.h
        SET(FLANN_VERSION_FILE ${FLANN_INCLUDE_DIRS}/config.h)
        IF (NOT EXISTS ${FLANN_VERSION_FILE})
                FLANN_REPORT_NOT_FOUND(
                  "Could not find file: ${FLANN_VERSION_FILE} "
                  "containing version information in Flann install located at: "
                  "${FLANN_INCLUDE_DIRS}.")
        ELSE (NOT EXISTS ${FLANN_VERSION_FILE})
            FILE(READ ${FLANN_VERSION_FILE} FLANN_VERSION_FILE_CONTENTS)
            STRING(REGEX MATCH "#define FLANN_VERSION_ \"([0-9.]+)\""
              FLANN_VERSION "${FLANN_VERSION_FILE_CONTENTS}")
            STRING(REGEX REPLACE "#define FLANN_VERSION_ \"([0-9.]+)\"" "\\1"
              FLANN_VERSION "${FLANN_VERSION}")
        ENDIF (NOT EXISTS ${FLANN_VERSION_FILE})
        SET(FLANN_INCLUDE_DIR ${FLANN_DIR})

        FIND_LIBRARY(FLANN_LIBRARY NAMES flann_cpp)

        # locate Flann libraries
        IF(DEFINED FLANN_LIBRARY)
          SET(FLANN_LIBRARIES ${FLANN_LIBRARY})
        ENDIF()

        MESSAGE(STATUS "Flann ${FLANN_VERSION} found (include: ${FLANN_INCLUDE_DIRS})")
ELSE()
  MESSAGE(FATAL_ERROR "You are attempting to build without Flann. "
          "Please use cmake variable -DFLANN_INCLUDE_DIR_HINTS:STRING=\"PATH\" "
          "or FLANN_INCLUDE_DIR_HINTS env. variable to a valid Flann path. "
          "Or install last Flann version.")
  package_report_not_found(FLANN "Flann cannot be found")
ENDIF()
##====================================================
