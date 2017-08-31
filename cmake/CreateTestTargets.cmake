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

# I made each test it's own executable. Alternatively, they could all
# be stuff in one large executable. I am unsure what is best. -Z
function(create_test_targets)
  cmake_parse_arguments(
    test  # Prefix of output variables
    ""    # List of Boolean Variables
    "DIR" # List of mono valued arguments
    "LIBS;INC;DEPS;EXCLUDE" # List of Multi Value Arguments
    ${ARGN}
    )
  if (USE_CTC)
    RETURN()
  ENDIF (USE_CTC)

  # Search for all the tests in tool dir
  file(GLOB TEST_SRC_FILES ${test_DIR}/*.cxx)

  # Sift through tool sources and remove EXECLUDE
  foreach(SRC ${TEST_SRC_FILES})
    foreach(TEST_SRC ${test_EXCLUDE})
      string(FIND ${SRC} ${TEST_SRC} POSITION)
      if (${POSITION} GREATER -1)
      	list(REMOVE_ITEM TEST_SRC_FILES ${SRC})
      endif()
    endforeach()
  endforeach()

  foreach(filename ${TEST_SRC_FILES})
    string(REPLACE ".cxx" "" execname ${filename})
    string(REGEX REPLACE "^[^ ]*/" "" execname ${execname})
    add_executable(${execname} ${filename})
    target_link_libraries(${execname}
      LINK_PUBLIC ${GTEST_LIBRARIES} ${test_LIBS})
    target_include_directories(${execname} PUBLIC ${GTEST_INCLUDE_DIRS} ${test_INC})

    # define TEST_DIR so that tests can find their test data
    get_filename_component(ABSOLUTE_TEST_DIR ${test_DIR} ABSOLUTE)
    add_definitions(-DTEST_DIR=\"${ABSOLUTE_TEST_DIR}\")

    add_dependencies(${execname} ${test_DEPS} gtest_noros)
    add_test(
      NAME ${execname}
      COMMAND ../bin/${execname}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing
      )

    # install our tests to a new folder so we can check on robot that
    # code is working.
    install(TARGETS ${execname} DESTINATION test)
  endforeach()
endfunction()

