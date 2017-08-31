#
# Install Python files and update their shebang lines
# to use a different Python executable.
#
# The signature:
#
#   catkin_install_python(PROGRAMS files... DESTINATION <dir>
#     [OPTIONAL]
#   )
#
# See the documentation for CMake install() function for more information.
#
# @public
#
function(catkin_install_python signature)
  string(TOUPPER "${signature}" signature)
  if(NOT "${signature}" STREQUAL "PROGRAMS")
    message(FATAL_ERROR "catkin_install_python() only supports the PROGRAMS signature (not '${signature}').")
  endif()
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "" ${ARGN})
  if(NOT ARG_DESTINATION)
    message(FATAL_ERROR "catkin_install_python() called without required DESTINATION argument.")
  endif()
  foreach(file ${ARG_UNPARSED_ARGUMENTS})
    if(NOT IS_ABSOLUTE ${file})
      set(file "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
    endif()
    if(EXISTS ${file})
      # read file and check shebang line
      file(READ ${file} data)
      set(regex "^#!/([^\r\n]+)/env python([\r\n])")
      string(REGEX MATCH "${regex}" shebang_line "${data}")
      string(LENGTH "${shebang_line}" length)
      string(SUBSTRING "${data}" 0 ${length} prefix)
      if("${shebang_line}" STREQUAL "${prefix}")
        # write modified file with modified shebang line
        get_filename_component(python_name ${PYTHON_EXECUTABLE} NAME)
        string(REGEX REPLACE "${regex}" "#!/\\1/env ${python_name}\\2" data "${data}")
        get_filename_component(filename ${file} NAME)
        set(file "${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace")
        file(MAKE_DIRECTORY ${file})
        set(file "${file}/${filename}")
        file(WRITE ${file} "${data}")
      endif()
      # install (modified) file to destination
      set(optional_flag "")
      if(ARG_OPTIONAL)
        set(optional_flag "OPTIONAL")
      endif()
      install(PROGRAMS "${file}" DESTINATION "${ARG_DESTINATION}" ${optional_flag})
    elseif(NOT ARG_OPTIONAL)
      message(FATAL_ERROR "catkin_install_python() called with non-existing file '${file}'.")
    endif()
  endforeach()
endfunction()
