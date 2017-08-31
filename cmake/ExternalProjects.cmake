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

include(ExternalProject)

function(CMakeExternalProject)
  cmake_parse_arguments(
    cmake
    # List of Booleans
    "INSTALL"
    # List of Mono Valued Arguments
    "NAME;SOURCE_DIR;URL;URL_HASH;GIT_REPOSITORY;GIT_TAG"
    # List of Multi Valued Arguments
    "CMAKE_ARGS;DEPENDS;UPDATE_COMMAND;PATCH_COMMAND"
    ${ARGN}
    )

  # Apply our CXX and C settings to this repo
  list(APPEND cmake_CMAKE_ARGS "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
  list(APPEND cmake_CMAKE_ARGS "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  list(APPEND cmake_CMAKE_ARGS "-DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}")
  list(APPEND cmake_CMAKE_ARGS "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
  list(APPEND cmake_CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
  list(APPEND cmake_CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
  list(APPEND cmake_CMAKE_ARGS "-DBUILD_SHARED_LIBS=${BUILD_SHARED_LIBS}")

  if (USE_CTC)
    list(APPEND cmake_CMAKE_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  endif (USE_CTC)

  # /usr/local is the default value if the user hasn't set it
  if (NOT ${CMAKE_INSTALL_PREFIX} STREQUAL "/usr/local")
    # Only install if the install prefix has been set and if the
    # INSTALL flag was set.
    list(APPEND cmake_CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
    if (${cmake_INSTALL})
      set(INSTALL_COMMAND $(MAKE) install)
    endif()
  else()
    set(INSTALL_COMMAND "")
  endif()

  ExternalProject_Add(${cmake_NAME}
    DEPENDS ${cmake_DEPENDS}
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    # Download Step
    SOURCE_DIR "${cmake_SOURCE_DIR}"
    DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/downloads/"
    GIT_REPOSITORY ${cmake_GIT_REPOSITORY}
    GIT_TAG ${cmake_GIT_TAG}
    URL ${cmake_URL}
    URL_HASH ${cmake_URL_HASH}
    TIMEOUT 600
    # Update/Patch Step
    UPDATE_COMMAND "${cmake_UPDATE_COMMAND}"
    PATCH_COMMAND ${cmake_PATCH_COMMAND}
    # Configure Step
    CMAKE_ARGS ${cmake_CMAKE_ARGS}
    # Build Step
    BUILD_COMMAND $(MAKE)
    # Test Step
    INSTALL_COMMAND "${INSTALL_COMMAND}"
    )
  ExternalProject_Get_Property(${cmake_NAME} source_dir)
  ExternalProject_Get_Property(${cmake_NAME} binary_dir)
  set(${cmake_NAME}_SOURCE_DIR ${source_dir} PARENT_SCOPE)
  set(${cmake_NAME}_BINARY_DIR ${binary_dir} PARENT_SCOPE)
endfunction(CMakeExternalProject)

function(AutotoolsExternalProject)
endfunction(AutotoolsExternalProject)
