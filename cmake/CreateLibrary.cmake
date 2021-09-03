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

function(create_library)
  cmake_parse_arguments(
    library # Prefix of output variables
    ""      # List of Boolean Variables
    "TARGET;DIR" # List of mono valued arguments
    "LIBS;INC;DEPS;ADD_SRCS;EXCLUDE;DEFINES;OPTIONS" # List of Multi Value Arguments
    ${ARGN}
    )

  # If no directory was given, assume all files in the src dir go in our library
  if(NOT library_DIR)
    SET(library_DIR src)
  endif(NOT library_DIR)

  # Search for all the executables in tool dir
  file(GLOB SRC_FILES_C  "${library_DIR}/*.c")
  file(GLOB SRC_FILES_CC "${library_DIR}/*.cc")
  set(SRC_FILES ${SRC_FILES_C} ${SRC_FILES_CC})

  # Sift through SRC_FILES and remove EXCLUDE
  foreach(SRC ${SRC_FILES})
    foreach (TEST_SRC ${library_EXCLUDE})
      string(FIND ${SRC} ${TEST_SRC} POSITION)
      if (${POSITION} GREATER -1)
	list(REMOVE_ITEM SRC_FILES ${SRC})
      endif()
    endforeach()
  endforeach()

  # Some of our dependencies might be header only, their headers paths are not
  # being added automatically like the targets listed in LIB. Here we'll do it
  # manually.
  if (library_DEPS)
    foreach(DEP ${library_DEPS})
      get_target_property(INC ${DEP} INCLUDE_DIRECTORIES)
      if (INC)
        list(APPEND library_INC ${INC})
      endif (INC)
    endforeach()
    list(REMOVE_DUPLICATES library_INC)
  endif (library_DEPS)

  add_library(${library_TARGET}
    ${SRC_FILES} ${library_ADD_SRCS})
  target_include_directories(${library_TARGET}
    PUBLIC ${library_INC} ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${library_TARGET}
    ${library_LIBS} ${CMAKE_LIBS_COVERAGE})
  if (library_DEFINES)
    target_compile_definitions(${library_TARGET}
      PUBLIC ${library_DEFINES})
  endif (library_DEFINES)
  if (library_OPTIONS)
    target_compile_options(${library_TARGET}
	  PUBLIC ${library_OPTIONS})
  endif (library_OPTIONS)
  if (NOT "${library_DEPS}" STREQUAL "")
    add_dependencies(${library_TARGET}
      ${library_DEPS})
  endif()

  install(TARGETS ${library_TARGET} DESTINATION lib)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
endfunction()
