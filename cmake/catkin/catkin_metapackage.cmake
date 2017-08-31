#
# It installs the package.xml file of a metapackage.
#
# .. note:: It must be called once for each metapackage.  Best
#   practice is to call this macro early in your root CMakeLists.txt,
#   immediately after calling ``project()`` and
#   ``find_package(catkin REQUIRED)``.
#
# :param DIRECTORY: the path to the package.xml file if not in the same
#   location as the CMakeLists.txt file
# :type DIRECTORY: string
#
# @public
#
function(catkin_metapackage)
  cmake_parse_arguments(ARG "" "DIRECTORY" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catkin_metapackage() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # verify that project() has been called before
  if(NOT PROJECT_NAME)
    message(FATAL_ERROR "catkin_metapackage() PROJECT_NAME is not set. You must call project() before calling catkin_metapackage().")
  endif()
  if(PROJECT_NAME STREQUAL "Project")
    message(FATAL_ERROR "catkin_metapackage() PROJECT_NAME is set to 'Project', which is not a valid project name. You must call project() before calling catkin_metapackage().")
  endif()

  debug_message(10 "catkin_metapackage() called in file ${CMAKE_CURRENT_LIST_FILE}")

  if(NOT ARG_DIRECTORY)
    if(${CMAKE_CURRENT_LIST_FILE} STREQUAL ${CMAKE_BINARY_DIR}/catkin_generated/metapackages/${PROJECT_NAME}/CMakeLists.txt)
      set(ARG_DIRECTORY ${CMAKE_SOURCE_DIR}/${path})
    else()
      set(ARG_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
  endif()

  catkin_package_xml(DIRECTORY ${ARG_DIRECTORY})

  # install package.xml
  install(FILES ${ARG_DIRECTORY}/package.xml
    DESTINATION share/${PROJECT_NAME}
  )
endfunction()
