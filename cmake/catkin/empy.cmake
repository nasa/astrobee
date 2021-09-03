function(find_python_module module)
  # cribbed from http://www.cmake.org/pipermail/cmake/2011-January/041666.html
  string(TOUPPER ${module} module_upper)
  if(NOT PY_${module_upper})
    if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
      set(${module}_FIND_REQUIRED TRUE)
    endif()
    # A module's location is usually a directory, but for
    # binary modules
    # it's a .so file.
    execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c" "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
      RESULT_VARIABLE _${module}_status 
      OUTPUT_VARIABLE _${module}_location
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _${module}_status)
      set(PY_${module_upper} ${_${module}_location} CACHE STRING "Location of Python module ${module}")
    endif(NOT _${module}_status)
  endif(NOT PY_${module_upper})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(PY_${module} DEFAULT_MSG PY_${module_upper})
endfunction(find_python_module)

if(NOT EMPY_SCRIPT)
  find_program(EMPY_EXECUTABLE empy)
  if(NOT EMPY_EXECUTABLE)
    # On OSX, there's an em.py, but not an executable empy script
    find_python_module(em)
    if(NOT PY_EM)
      message(FATAL_ERROR "Unable to find either executable 'empy' or Python module 'em'... try installing the package 'python-empy'")
    endif()
    # ensure to use cmake-style path separators on Windows
    file(TO_CMAKE_PATH "${PY_EM}" EMPY_SCRIPT)
  else()
    # ensure to use cmake-style path separators on Windows
    file(TO_CMAKE_PATH "${EMPY_EXECUTABLE}" EMPY_SCRIPT)
  endif()
  set(EMPY_SCRIPT "${EMPY_SCRIPT}" CACHE STRING "Empy script" FORCE)
endif()
#message(STATUS "Using empy: ${EMPY_SCRIPT}")
