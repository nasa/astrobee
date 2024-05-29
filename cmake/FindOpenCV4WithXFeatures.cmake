
# Find OpenCV installation
find_package(OpenCV 4.0 REQUIRED)


list(APPEND OpenCV_LIB_COMPONENTS "opencv_xfeatures2d")

if(NOT CMAKE_VERSION VERSION_LESS "2.8.11")
  # Target property INTERFACE_INCLUDE_DIRECTORIES available since 2.8.11:
  # * http://www.cmake.org/cmake/help/v2.8.11/cmake.html#prop_tgt:INTERFACE_INCLUDE_DIRECTORIES
  foreach(__component ${OpenCV_LIB_COMPONENTS})
    if(TARGET ${__component})
      set_target_properties(
          ${__component}
          PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}"
      )
    endif()
  endforeach()
endif()

# ==============================================================
#  Form list of modules (components) to find
# ==============================================================
if(NOT OpenCV_FIND_COMPONENTS)
  set(OpenCV_FIND_COMPONENTS ${OpenCV_LIB_COMPONENTS})
  list(REMOVE_ITEM OpenCV_FIND_COMPONENTS opencv_java)
  if(GTest_FOUND OR GTEST_FOUND)
    list(REMOVE_ITEM OpenCV_FIND_COMPONENTS opencv_ts)
  endif()
endif()

set(OpenCV_WORLD_COMPONENTS )

# expand short module names and see if requested components exist
foreach(__cvcomponent ${OpenCV_FIND_COMPONENTS})
  # Store the name of the original component so we can set the
  # OpenCV_<component>_FOUND variable which can be checked by the user.
  set (__original_cvcomponent ${__cvcomponent})
  if(NOT __cvcomponent MATCHES "^opencv_")
    set(__cvcomponent opencv_${__cvcomponent})
  endif()
  list(FIND OpenCV_LIB_COMPONENTS ${__cvcomponent} __cvcomponentIdx)
  if(__cvcomponentIdx LESS 0)
    if(_OpenCV_HANDLE_COMPONENTS_MANUALLY)
      # Either the component is required or the user did not set any components at
      # all. In the latter case, the OpenCV_FIND_REQUIRED_<component> variable
      # will not be defined since it is not set by this config. So let's assume
      # the implicitly set components are always required.
      if(NOT DEFINED OpenCV_FIND_REQUIRED_${__original_cvcomponent} OR
          OpenCV_FIND_REQUIRED_${__original_cvcomponent})
        message(FATAL_ERROR "${__cvcomponent} is required but was not found")
      elseif(NOT OpenCV_FIND_QUIETLY)
        # The component was marked as optional using OPTIONAL_COMPONENTS
        message(WARNING "Optional component ${__cvcomponent} was not found")
      endif()
    endif(_OpenCV_HANDLE_COMPONENTS_MANUALLY)
    #indicate that module is NOT found
    string(TOUPPER "${__cvcomponent}" __cvcomponentUP)
    set(${__cvcomponentUP}_FOUND "${__cvcomponentUP}_FOUND-NOTFOUND")
    set(OpenCV_${__original_cvcomponent}_FOUND FALSE)
  else()
    # Not using list(APPEND) here, because OpenCV_LIBS may not exist yet.
    # Also not clearing OpenCV_LIBS anywhere, so that multiple calls
    # to find_package(OpenCV) with different component lists add up.
    set(OpenCV_LIBS ${OpenCV_LIBS} "${__cvcomponent}")
    #indicate that module is found
    string(TOUPPER "${__cvcomponent}" __cvcomponentUP)
    set(${__cvcomponentUP}_FOUND 1)
    set(OpenCV_${__original_cvcomponent}_FOUND TRUE)
  endif()
  if(OpenCV_SHARED AND ";${OpenCV_WORLD_COMPONENTS};" MATCHES ";${__cvcomponent};" AND NOT TARGET ${__cvcomponent})
    get_target_property(__implib_dbg opencv_world IMPORTED_IMPLIB_DEBUG)
    get_target_property(__implib_release opencv_world  IMPORTED_IMPLIB_RELEASE)
    get_target_property(__location_dbg opencv_world IMPORTED_LOCATION_DEBUG)
    get_target_property(__location_release opencv_world  IMPORTED_LOCATION_RELEASE)
    get_target_property(__include_dir opencv_world INTERFACE_INCLUDE_DIRECTORIES)
    add_library(${__cvcomponent} SHARED IMPORTED)
    set_target_properties(${__cvcomponent} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${__include_dir}")
    if(__location_dbg)
      set_property(TARGET ${__cvcomponent} APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(${__cvcomponent} PROPERTIES
        IMPORTED_IMPLIB_DEBUG "${__implib_dbg}"
        IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG ""
        IMPORTED_LOCATION_DEBUG "${__location_dbg}"
      )
    endif()
    if(__location_release)
      set_property(TARGET ${__cvcomponent} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(${__cvcomponent} PROPERTIES
        IMPORTED_IMPLIB_RELEASE "${__implib_release}"
        IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE ""
        IMPORTED_LOCATION_RELEASE "${__location_release}"
      )
    endif()
  endif()
  if(TARGET ${__cvcomponent})
    ocv_map_imported_config(${__cvcomponent})
  endif()
endforeach()

if(__remap_warnings AND NOT OpenCV_FIND_QUIETLY)
  message("OpenCV: configurations remap warnings:\n${__remap_warnings}OpenCV: Check variable OPENCV_MAP_IMPORTED_CONFIG=${OPENCV_MAP_IMPORTED_CONFIG}")
endif()

# find_package(OpenCV 4 REQUIRED PATHS ${OpenCV_DIR_LOCAL})
set(OpenCV_LIBRARIES ${OpenCV_LIBS})

