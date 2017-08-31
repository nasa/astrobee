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

# Locate Luajit library
# This module defines
#  LUAJIT_FOUND, if false, do not try to link to Lua
#  LUAJIT_LIBRARIES
#  LUAJIT_INCLUDE_DIR, where to find luajit.h
#  LUAJIT_VERSION_STRING, the version of Lua found (since CMake 2.8.8)

find_path(LUAJIT_INCLUDE_DIR luajit.h
  PATH_SUFFIXES include/luajit-2.0 include
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt
  /usr/local
)

find_library(LUAJIT_LIBRARY
  NAMES luajit-5.1
  PATH_SUFFIXES lib
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /sw
  /opt/local
  /opt/csw
  /opt
  /usr/local
)

if(LUAJIT_LIBRARY)
  # include the math library for Unix
  if(UNIX AND NOT APPLE AND NOT BEOS)
    find_library(LUAJIT_MATH_LIBRARY m)
    set( LUAJIT_LIBRARIES "${LUAJIT_LIBRARY};${LUAJIT_MATH_LIBRARY}" CACHE STRING "Lua Libraries")
  # For Windows and Mac, don't need to explicitly include the math library
  else()
    set( LUAJIT_LIBRARIES "${LUAJIT_LIBRARY}" CACHE STRING "Luajit Libraries")
  endif()
endif()

if(LUAJIT_INCLUDE_DIR AND EXISTS "${LUAJIT_INCLUDE_DIR}/luajit.h")
  file(STRINGS "${LUAJIT_INCLUDE_DIR}/luajit.h" luajit_version_str REGEX "^#define[ \t]+LUAJIT_VERSION[ \t]+\"LuaJIT .+\"")

  string(REGEX REPLACE "^#define[ \t]+LUAJIT_VERSION[ \t]+\"LuaJIT ([^\"]+)\".*" "\\1" LUAJIT_VERSION_STRING "${luajit_version_str}")
  unset(luajit_version_str)
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LUA_FOUND to TRUE if
# all listed variables are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Luajit20
                                  REQUIRED_VARS LUAJIT_LIBRARIES 
                                  LUAJIT_INCLUDE_DIR VERSION_VAR 
                                  LUAJIT_VERSION_STRING)

mark_as_advanced(LUAJIT_INCLUDE_DIR LUAJIT_LIBRARIES LUAJIT_LIBRARY 
                  LUAJIT_MATH_LIBRARY)

