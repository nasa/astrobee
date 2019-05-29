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

function(create_tool_targets)
  cmake_parse_arguments(
    tool  # Prefix of output variables
    ""    # List of boolean variables
    "DIR" # List of mono valued arguments
    "LIBS;INC;DEPS;EXCLUDE" # List of multi-value arguments
    ${ARGN}
    )

  # Search for all the executables in tool dir
  file(GLOB TOOL_SRC_FILES_C  "${tool_DIR}/*.c")
  file(GLOB TOOL_SRC_FILES_CC "${tool_DIR}/*.cc")
  set(TOOL_SRC_FILES ${TOOL_SRC_FILES_C} ${TOOL_SRC_FILES_CC})

  # Sift through SRC_FILES and remove EXCLUDE
  foreach(SRC ${TOOL_SRC_FILES})
    foreach(TEST_SRC ${tool_EXCLUDE})
      string(FIND ${SRC} ${TEST_SRC} POSITION)
      if (${POSITION} GREATER -1)
      	list(REMOVE_ITEM TOOL_SRC_FILES ${SRC})
      endif()
    endforeach()
  endforeach()

  # Determine our module name
  get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  # Some of our dependencies might be header only, their headers paths are not
  # being added automatically like the targets listed in LIB. Here we'll do it
  # manually.
  if (tool_DEPS)
    foreach(DEP ${tool_DEPS})
      get_target_property(INC ${DEP} INCLUDE_DIRECTORIES)
      if (INC)
        list(APPEND tool_INC ${INC})
      endif (INC)
    endforeach()
    list(REMOVE_DUPLICATES tool_INC)
  endif (tool_DEPS)

  # Link the executables!
  foreach(filename ${TOOL_SRC_FILES})
    string(REGEX REPLACE ".(cc|c)$" "" execname ${filename})
    string(REGEX REPLACE "^[^ ]*/" "" execname ${execname})
    add_executable(${execname} ${filename})
    target_link_libraries(${execname}
      LINK_PUBLIC ${tool_LIBS})
    target_include_directories(${execname} PUBLIC ${tool_INC})
    if (tool_DEPS)
      add_dependencies(${execname} ${tool_DEPS})
    endif (tool_DEPS)
    install(TARGETS ${execname} DESTINATION bin)

    # Create symlinks so our executables can be found with rosrun. This is done
    # inside an 'install' so that it happens after the installation.
    install(CODE "execute_process(
      COMMAND mkdir -p share/${PROJECT_NAME}
      COMMAND ln -s ../../bin/${execname} share/${PROJECT_NAME}/${execname}
      WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
      OUTPUT_QUIET
      ERROR_QUIET
      )")
  endforeach()
endfunction()
