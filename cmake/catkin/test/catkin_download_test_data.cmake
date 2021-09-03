_generate_function_if_testing_is_disabled("catkin_download_test_data")

#
# Download a file containing test data from a URL.
#
# It is commonly used to download larger data files for unit tests
# which should not be stored in the repository.
#
# .. note:: The target will be registered as a dependency
#   of the "tests" target.
#
# .. note:: If the tests should be run on the ROS buildfarm the URL
#   must be publically and reliably accessible.
#
# :param target: the target name
# :type target: string
# :param url: the url to download
# :type url: string

# :param DESTINATION: the directory where the file is downloaded to
#   (default: ${PROJECT_BINARY_DIR})
# :type DESTINATION: string
# :param FILENAME: the filename of the downloaded file
#   (default: the basename of the url)
# :type FILENAME: string
# :param MD5: the expected md5 hash to compare against
#   (default: empty, skipping the check)
# :type MD5: string
#
# @public
function(catkin_download_test_data target url)
  _warn_if_skip_testing("catkin_download_test_data")

  cmake_parse_arguments(ARG "" "DESTINATION;FILENAME;MD5" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_download_test_data() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_DESTINATION)
    set(ARG_DESTINATION ${PROJECT_BINARY_DIR})
  endif()
  if(NOT ARG_FILENAME)
    get_filename_component(ARG_FILENAME ${url} NAME)
  endif()
  set(output "${ARG_DESTINATION}/${ARG_FILENAME}")
  add_custom_command(OUTPUT ${output}
    COMMAND ${PYTHON_EXECUTABLE} ${catkin_EXTRAS_DIR}/test/download_checkmd5.py ${url} ${output} ${ARG_MD5}
    VERBATIM)
  add_custom_target(${target} DEPENDS ${output})
  if(TARGET tests)
    add_dependencies(tests ${target})
  endif()
endfunction()
