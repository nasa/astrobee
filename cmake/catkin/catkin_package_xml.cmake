#
# Parse package.xml from ``CMAKE_CURRENT_SOURCE_DIR`` and
# make several information available to CMake.
#
# .. note:: It is called automatically by ``catkin_package()`` if not
#   called manually before.  It must be called once in each package,
#   after calling ``project()`` where the project name must match the
#   package name.  The macro should only be called manually if the
#   variables are use to parameterize ``catkin_package()``.
#
# :param DIRECTORY: the directory of the package.xml (default
#   ``${CMAKE_CURRENT_SOURCE_DIR}``).
# :type DIRECTORY: string
#
# :outvar <packagename>_VERSION: the version number
# :outvar <packagename>_MAINTAINER: the name and email of the
#   maintainer(s)
# :outvar _CATKIN_CURRENT_PACKAGE: the name of the package from the
#   manifest
#
# .. note:: It is calling ``catkin_destinations()`` which will provide
#   additional output variables.
#
# @public
#
macro(catkin_package_xml)
  debug_message(10 "catkin_package_xml()")

  # verify that project() has been called before
  if(NOT PROJECT_NAME)
    message(FATAL_ERROR "catkin_package_xml() PROJECT_NAME is not set. You must call project() before you can call catkin_package_xml().")
  endif()

  # ensure that function is not called multiple times per package
  if(DEFINED _CATKIN_CURRENT_PACKAGE)
    message(FATAL_ERROR "catkin_package_xml(): in '${CMAKE_CURRENT_LIST_FILE}', _CATKIN_CURRENT_PACKAGE is already set (to: ${_CATKIN_CURRENT_PACKAGE}).  Did you called catkin_package_xml() multiple times?")
  endif()

  _catkin_package_xml(${CMAKE_CURRENT_BINARY_DIR}/catkin_generated ${ARGN})

  # verify that the package name from package.xml equals the project() name
  if(NOT _CATKIN_CURRENT_PACKAGE STREQUAL PROJECT_NAME)
    message(FATAL_ERROR "catkin_package_xml() package name '${_CATKIN_CURRENT_PACKAGE}'  in '${_PACKAGE_XML_DIRECTORY}/package.xml' does not match current PROJECT_NAME '${PROJECT_NAME}'.  You must call project() with the same package name before.")
  endif()

  catkin_destinations()
endmacro()

macro(_catkin_package_xml dest_dir)
  cmake_parse_arguments(_PACKAGE_XML "" "DIRECTORY" "" ${ARGN})
  if(_PACKAGE_XML_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_package_xml() called with unused arguments: ${_PACKAGE_XML_UNPARSED_ARGUMENTS}")
  endif()

  # set default directory
  if(NOT _PACKAGE_XML_DIRECTORY)
    set(_PACKAGE_XML_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  # stamp and parse package.xml
  stamp(${_PACKAGE_XML_DIRECTORY}/package.xml)
  file(MAKE_DIRECTORY ${dest_dir})
  safe_execute_process(COMMAND ${PYTHON_EXECUTABLE}
    ${catkin_EXTRAS_DIR}/parse_package_xml.py
    ${_PACKAGE_XML_DIRECTORY}/package.xml
    ${dest_dir}/package.cmake)
  # load extracted variable into cmake
  include(${dest_dir}/package.cmake)
endmacro()
