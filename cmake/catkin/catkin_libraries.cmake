set(CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR ":")

#
# Filter libraries based on optional build configuration keywords.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries
# :type ARGN: list of strings
# :param BUILD_TYPE: a keyword for the build type (default:
#   ``CMAKE_BUILD_TYPE``)
# :type BUILD_TYPE: list of strings
#
# @public
#
macro(catkin_filter_libraries_for_build_configuration VAR)
  cmake_parse_arguments(ARG "" "BUILD_TYPE" "" ${ARGN})
  if(NOT ARG_BUILD_TYPE)
    set(ARG_BUILD_TYPE ${CMAKE_BUILD_TYPE})
  endif()
  if(NOT ARG_BUILD_TYPE)
    set(ARG_BUILD_TYPE "Debug")
  endif()
  set(${VAR} "")
  list(LENGTH ARG_UNPARSED_ARGUMENTS _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET ARG_UNPARSED_ARGUMENTS ${_index} lib)

    if("${lib}" STREQUAL "debug")
      if(NOT "${ARG_BUILD_TYPE}" STREQUAL "Debug")
        # skip keyword and debug library for non-debug builds
        math(EXPR _index "${_index} + 1")
        if(${_index} EQUAL ${_count})
          message(FATAL_ERROR "catkin_filter_libraries_for_build_configuration() the list of libraries '${ARG_UNPARSED_ARGUMENTS}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
        endif()
      endif()
    elseif("${lib}" STREQUAL "optimized")
      if("${ARG_BUILD_TYPE}" STREQUAL "Debug")
        # skip keyword and non-debug library for debug builds
        math(EXPR _index "${_index} + 1")
        if(${_index} EQUAL ${_count})
          message(FATAL_ERROR "catkin_filter_libraries_for_build_configuration() the list of libraries '${ARG_UNPARSED_ARGUMENTS}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
        endif()
      endif()
    elseif("${lib}" STREQUAL "general")
      # just consume the keyword
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "catkin_package() the list of libraries '${ARG_UNPARSED_ARGUMENTS}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
  debug_message(10 "catkin_filter_libraries_for_build_configuration(${VAR} ${ARG_UNPARSED_ARGUMENTS} BUILD_TYPE ${ARG_BUILD_TYPE}) ${${VAR}}")
endmacro()

#
# Pack a list of libraries with optional build configuration keywords.
# Each keyword is joined with its library using a separator.
# A packed library list can be deduplicated correctly.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries
# :type ARGN: list of strings
#
# @public
#
macro(catkin_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^debug|optimized|general$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "catkin_pack_libraries_with_build_configuration() the list of libraries '${_argn}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
  #debug_message(10 "catkin_pack_libraries_with_build_configuration(${VAR} ${_argn}) ${${VAR}}")
endmacro()

#
# Unpack a list of libraries with optional build configuration keyword prefixes.
# Libraries prefixed with a keyword are split into the keyword and the library.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries
# :type ARGN: list of strings
#
# @public
#
macro(catkin_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
  #set(_argn ${ARGN})
  #debug_message(10 "catkin_unpack_libraries_with_build_configuration(${VAR} ${_argn}) ${${VAR}}")
endmacro()

#
# Replace imported library target names with the library name.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries
# :type ARGN: list of strings
#
# @public
#
macro(catkin_replace_imported_library_targets VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    if((NOT "${lib}" MATCHES "^debug|optimized|general$") AND TARGET ${lib})
      # sometimes cmake dependencies define imported targets, in which
      # case the imported library information is not the target name, but
      # the information embedded in cmake properties inside the imported library
      get_target_property(${lib}_imported ${lib} IMPORTED)
      if(${${lib}_imported})
        set(imported_libraries)  # empty list
        get_target_property(${lib}_imported_location ${lib} IMPORTED_LOCATION)
        if(${lib}_imported_location)
          list(APPEND imported_libraries ${${lib}_imported_location})
        else()
          get_target_property(${lib}_imported_configurations ${lib} IMPORTED_CONFIGURATIONS)
          foreach(cfg ${${lib}_imported_configurations})
            get_target_property(${lib}_imported_location_${cfg} ${lib} IMPORTED_LOCATION_${cfg})
            if(${lib}_imported_location_${cfg})
              list(APPEND imported_libraries ${${lib}_imported_location_${cfg}})
            endif()
          endforeach()
        endif()
        foreach(imp_lib ${imported_libraries})
          list(APPEND ${VAR} "${imp_lib}")
        endforeach()
      else()
        # not an imported library target
        list(APPEND ${VAR} "${lib}")
      endif()
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
  endforeach()
  #set(_argn ${ARGN})
  #debug_message(10 "catkin_replace_imported_library_targets(${VAR} ${_argn}) ${${VAR}}")
endmacro()
