option(CATKIN_ENABLE_TESTING "Catkin enable testing" ON)
option(CATKIN_SKIP_TESTING "Catkin skip testing" OFF)

# check if testing is explicity skipped
if(CATKIN_SKIP_TESTING)
  set(CATKIN_ENABLE_TESTING OFF)
  #message(STATUS "Using CATKIN_SKIP_TESTING: ${CATKIN_SKIP_TESTING} (implying CATKIN_ENABLE_TESTING=${CATKIN_ENABLE_TESTING})")
else()
  #message(STATUS "Using CATKIN_ENABLE_TESTING: ${CATKIN_ENABLE_TESTING}")
endif()

# creates a dummy function in case testing has been explicitly disabled (and not only skipping)
# which outputs an error message when being invoked
macro(_generate_function_if_testing_is_disabled funcname)
  if(DEFINED CATKIN_ENABLE_TESTING AND NOT CATKIN_ENABLE_TESTING AND NOT CATKIN_SKIP_TESTING)
    function(${funcname})
      message(FATAL_ERROR
        "${funcname}() is not available when tests are not enabled. The CMake code should only use it inside a conditional block which checks that testing is enabled:\n"
        "if(CATKIN_ENABLE_TESTING)\n"
        "  ${funcname}(...)\n"
        "endif()\n")
    endfunction()
    return()
  endif()
endmacro()

# checks if a function has been called while testing is skipped
# and outputs a warning message
macro(_warn_if_skip_testing funcname)
  if(DEFINED CATKIN_ENABLE_TESTING AND NOT CATKIN_ENABLE_TESTING)
    message(WARNING
      "${funcname}() should only be used inside a conditional block which checks that testing is enabled:\n"
      "if(CATKIN_ENABLE_TESTING)\n"
      "  ${funcname}(...)\n"
      "endif()\n")
  endif()
endmacro()

if(DEFINED CATKIN_ENABLE_TESTING AND NOT CATKIN_ENABLE_TESTING AND NOT CATKIN_SKIP_TESTING)
  return()
endif()

# do not enable ctest's on the farm, since they are automatically executed by the current rules files
# and since the tests have not been build rostests would hang forever
if(NOT CATKIN_BUILD_BINARY_PACKAGE)
  # do not enable ctest's for dry packages, since they have a custom test target which must not be overwritten
  if(NOT ROSBUILD_init_called)
    #message(STATUS "Call enable_testing()")
    enable_testing()
  else()
    message(STATUS "Skip enable_testing() for dry packages")
  endif()
else()
  message(STATUS "Skip enable_testing() when building binary package")
endif()

# allow overriding CATKIN_TEST_RESULTS_DIR when explicitly passed to CMake as a command line argument
if(DEFINED CATKIN_TEST_RESULTS_DIR)
  set(CATKIN_TEST_RESULTS_DIR ${CATKIN_TEST_RESULTS_DIR} CACHE INTERNAL "")
else()
  set(CATKIN_TEST_RESULTS_DIR ${CMAKE_BINARY_DIR}/test_results CACHE INTERNAL "")
endif()
#message(STATUS "Using CATKIN_TEST_RESULTS_DIR: ${CATKIN_TEST_RESULTS_DIR}")
file(MAKE_DIRECTORY ${CATKIN_TEST_RESULTS_DIR})

# create target to build tests
if(NOT TARGET tests)
  add_custom_target(tests)
endif()

# create target to run all tests
# it uses the dot-prefixed test targets to depend on building all tests and cleaning test results before the tests are executed
if(NOT TARGET run_tests)
  add_custom_target(run_tests)
endif()

# create target to clean test results
if(NOT TARGET clean_test_results)
  add_custom_target(clean_test_results
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CATKIN_TEST_RESULTS_DIR})
endif()

#
# Create a test target, integrate it with the run_tests infrastructure
# and post-process the junit result.
#
# All test results go under ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/..
#
# This function is only used internally by the various
# catkin_add_*test() functions.
#
function(catkin_run_tests_target type name xunit_filename)
  cmake_parse_arguments(_testing "" "WORKING_DIRECTORY" "COMMAND;DEPENDENCIES" ${ARGN})
  if(_testing_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_run_tests_target() called with unused arguments: ${_testing_UNPARSED_ARGUMENTS}")
  endif()

  # create meta target to trigger all tests of a project
  if(NOT TARGET run_tests_${PROJECT_NAME})
    add_custom_target(run_tests_${PROJECT_NAME})
    # create hidden meta target which depends on hidden test targets which depend on clean_test_results
    add_custom_target(_run_tests_${PROJECT_NAME})
    # run_tests depends on this hidden target hierarchy to clear test results before running all tests
    add_dependencies(run_tests _run_tests_${PROJECT_NAME})
  endif()
  # create meta target to trigger all tests of a specific type of a project
  if(NOT TARGET run_tests_${PROJECT_NAME}_${type})
    add_custom_target(run_tests_${PROJECT_NAME}_${type})
    add_dependencies(run_tests_${PROJECT_NAME} run_tests_${PROJECT_NAME}_${type})
    # hidden meta target which depends on hidden test targets which depend on clean_test_results
    add_custom_target(_run_tests_${PROJECT_NAME}_${type})
    add_dependencies(_run_tests_${PROJECT_NAME} _run_tests_${PROJECT_NAME}_${type})
  endif()
  if(NOT DEFINED CATKIN_ENABLE_TESTING OR CATKIN_ENABLE_TESTING)
    # create target for test execution
    set(results ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/${xunit_filename})
    if (_testing_WORKING_DIRECTORY)
      set(working_dir_arg "--working-dir" ${_testing_WORKING_DIRECTORY})
    endif()
    assert(CATKIN_ENV)
    set(cmd_wrapper ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/test/run_tests.py ${results} ${working_dir_arg})
    # for ctest the command needs to return non-zero if any test failed
    set(cmd ${cmd_wrapper} "--return-code" ${_testing_COMMAND})
    add_test(NAME _ctest_${PROJECT_NAME}_${type}_${name} COMMAND ${cmd})
    # for the run_tests target the command needs to return zero so that testing is not aborted
    set(cmd ${cmd_wrapper} ${_testing_COMMAND})
    add_custom_target(run_tests_${PROJECT_NAME}_${type}_${name}
      COMMAND ${cmd})
  else()
    # create empty dummy target
    set(cmd "${CMAKE_COMMAND}" "-E" "echo" "Skipping test target \\'run_tests_${PROJECT_NAME}_${type}_${name}\\'. Enable testing via -DCATKIN_ENABLE_TESTING.")
    add_custom_target(run_tests_${PROJECT_NAME}_${type}_${name} ${cmd})
  endif()
  add_dependencies(run_tests_${PROJECT_NAME}_${type} run_tests_${PROJECT_NAME}_${type}_${name})
  if(_testing_DEPENDENCIES)
    add_dependencies(run_tests_${PROJECT_NAME}_${type}_${name} ${_testing_DEPENDENCIES})
  endif()
  # hidden test target which depends on building all tests and cleaning test results
  add_custom_target(_run_tests_${PROJECT_NAME}_${type}_${name}
    COMMAND ${cmd})
  add_dependencies(_run_tests_${PROJECT_NAME}_${type} _run_tests_${PROJECT_NAME}_${type}_${name})
  add_dependencies(_run_tests_${PROJECT_NAME}_${type}_${name} clean_test_results tests ${_testing_DEPENDENCIES})
endfunction()
