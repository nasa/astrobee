_generate_function_if_testing_is_disabled("catkin_add_nosetests")

#
# Add Python nose tests.
#
# Nose collects tests from the directory ``dir`` automatically.
#
# .. note:: The test can be executed by calling ``nosetests``
#   directly or using:
#   `` make run_tests_${PROJECT_NAME}_nosetests_${dir}``
#   (where slashes in the ``dir`` are replaced with underscores)
#
# :param path: a relative or absolute directory to search for
#   nosetests in or a relative or absolute file containing tests
# :type path: string
# :param DEPENDENCIES: the targets which must be built before executing
#   the test
# :type DEPENDENCIES: list of strings
# :param TIMEOUT: the timeout for individual tests in seconds
#   (default: 60)
# :type TIMEOUT: integer
# :param WORKING_DIRECTORY: the working directory when executing the
#   tests
# :type WORKING_DIRECTORY: string
#
# @public
#
function(catkin_add_nosetests path)
  _warn_if_skip_testing("catkin_add_nosetests")

  if(NOT NOSETESTS)
    message(STATUS "skipping nosetests(${path}) in project '${PROJECT_NAME}'")
    return()
  endif()

  cmake_parse_arguments(_nose "" "TIMEOUT;WORKING_DIRECTORY" "DEPENDENCIES" ${ARGN})
  if(NOT _nose_TIMEOUT)
    set(_nose_TIMEOUT 60)
  endif()
  if(NOT _nose_TIMEOUT GREATER 0)
    message(FATAL_ERROR "nosetests() TIMEOUT argument must be a valid number of seconds greater than zero")
  endif()

  # check that the directory exists
  set(_path_name _path_name-NOTFOUND)
  if(IS_ABSOLUTE ${path})
    set(_path_name ${path})
  else()
    find_file(_path_name ${path}
      PATHS ${CMAKE_CURRENT_SOURCE_DIR}
      NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    if(NOT _path_name)
      message(FATAL_ERROR "Can't find nosetests path '${path}'")
    endif()
  endif()

  # check if coverage reports are being requested
  if("$ENV{CATKIN_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg " --with-coverage")
  endif()

  # strip PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR prefix from output_file_name
  set(output_file_name ${path})
  _strip_path_prefix(output_file_name "${output_file_name}" "${PROJECT_SOURCE_DIR}")
  _strip_path_prefix(output_file_name "${output_file_name}" "${PROJECT_BINARY_DIR}")
  if("${output_file_name}" STREQUAL "")
    set(output_file_name ".")
  endif()
  string(REPLACE "/" "." output_file_name ${output_file_name})
  string(REPLACE ":" "." output_file_name ${output_file_name})

  set(output_path ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  # make --xunit-file argument an absolute path (https://github.com/nose-devs/nose/issues/779)
  get_filename_component(output_path "${output_path}" ABSOLUTE)
  set(cmd "${CMAKE_COMMAND} -E make_directory ${output_path}")
  if(IS_DIRECTORY ${_path_name})
    set(tests "--where=${_path_name}")
  else()
    set(tests "${_path_name}")
  endif()
  set(cmd ${cmd} "${NOSETESTS} -P --process-timeout=${_nose_TIMEOUT} ${tests} --with-xunit --xunit-file=${output_path}/nosetests-${output_file_name}.xml${_covarg}")
  catkin_run_tests_target("nosetests" ${output_file_name} "nosetests-${output_file_name}.xml" COMMAND ${cmd} DEPENDENCIES ${_nose_DEPENDENCIES} WORKING_DIRECTORY ${_nose_WORKING_DIRECTORY})
endfunction()

find_program(NOSETESTS NAMES
  "nosetests${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
  "nosetests-${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
  "nosetests${PYTHON_VERSION_MAJOR}"
  "nosetests-${PYTHON_VERSION_MAJOR}"
  "nosetests")
if(NOT NOSETESTS)
  if("${PYTHON_VERSION_MAJOR}" STREQUAL "3")
    message(WARNING "nosetests not found, Python tests can not be run (try installing package 'python3-nose')")
  else()
    message(WARNING "nosetests not found, Python tests can not be run (try installing package 'python-nose')")
  endif()
endif()

macro(_strip_path_prefix var value prefix)
  if("${value}" STREQUAL "${prefix}" OR "${value}" STREQUAL "${prefix}/")
    set(${var} "")
  else()
    set(${var} "${value}")
    string(LENGTH "${prefix}/" prefix_length)
    string(LENGTH "${value}" var_length)
    if(${var_length} GREATER ${prefix_length})
      string(SUBSTRING "${value}" 0 ${prefix_length} var_prefix)
      if("${var_prefix}" STREQUAL "${prefix}/")
        # passing length -1 does not work for CMake < 2.8.5
        # http://public.kitware.com/Bug/view.php?id=10740
        string(LENGTH "${value}" _rest)
        math(EXPR _rest "${_rest} - ${prefix_length}")
        string(SUBSTRING "${value}" ${prefix_length} ${_rest} ${var})
      endif()
    endif()
  endif()
endmacro()
