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

# generated from catkin/cmake/catkinConfig.cmake.in
# which overlays the default template catkin/cmake/template/pkgConfig.cmake.in
#
# :outvar catkin_INCLUDE_DIRS: contains the include dirs of all searched components.
#    For use with CMake ``include_directories(${catkin_INCLUDE_DIRS})``.
# :outvar catkin_LIBRARY_DIRS: contains the library dirs of all searched components.
#    For use with CMake ``link_directories(${catkin_INCLUDE_DIRS})``.
# :outvar catkin_LIBRARIES: contains the include dirs of all searched components.
#    For use with CMake ``include_directories(${catkin_INCLUDE_DIRS})``.
# :outvar <comp>_INCLUDE_DIRS/_LIBRARY_DIRS/_LIBRARY:
#    contains the include dirs / library dirs / libraries of the searched component <comp>.

find_package(catkin REQUIRED)  # defines catkin_DIR
if(CATKIN_TOPLEVEL_FIND_PACKAGE OR NOT CATKIN_TOPLEVEL)
  set(catkin_EXTRAS_DIR "${catkin_DIR}")

  # prevent multiple inclusion from repeated find_package() calls in non-workspace context
  # as long as this variable is in the scope the variables from all.cmake are also, so no need to be evaluated again
  if(NOT DEFINED _CATKIN_CONFIG_ALL_INCLUDED_)
    set(_CATKIN_CONFIG_ALL_INCLUDED_ TRUE)
    include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
  endif()
endif()

# skip setting find_package() variables when discovered from toplevel.cmake
if(CATKIN_TOPLEVEL_FIND_PACKAGE)
  return()
endif()

# prevent usage with wrong case
if(CATKIN_FIND_COMPONENTS OR CATKIN_FIND_REQUIRED OR CATKIN_FIND_QUIETLY)
  message(FATAL_ERROR "find_package() only supports lower-case package name 'catkin'")
endif()

# mark as found
if(NOT catkin_FOUND)
  set(catkin_FOUND)
  set(CATKIN_PACKAGE_PREFIX "" CACHE STRING "Prefix to apply to package generated via gendebian")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(catkin_FOUND_CATKIN_PROJECT TRUE)

# XXXX don't overwrite catkin_* variables when being called recursively
if(NOT _CATKIN_FIND_ OR _CATKIN_FIND_ EQUAL 0)
  set(_CATKIN_FIND_ 0)
  if(catkin_FIND_COMPONENTS)
    set(catkin_INCLUDE_DIRS "")
    set(catkin_LIBRARIES "")
    set(catkin_LIBRARY_DIRS "")
    set(catkin_EXPORTED_TARGETS "")
  endif()
endif()

# increment recursion counter
math(EXPR _CATKIN_FIND_ "${_CATKIN_FIND_} + 1")

# find all components
if(catkin2_FIND_COMPONENTS)

  # Reset the variables being set
  set(catkin_LIBRARIES "")
  set(catkin_INCLUDE_DIRS "")

  foreach(component ${catkin2_FIND_COMPONENTS})
    string(TOLOWER "${component}" component_lower)
    # skip catkin since it does not make sense as a component
    if(NOT ${component_lower} STREQUAL "catkin")

      # get search paths from CMAKE_PREFIX_PATH (which includes devel space)
      set(paths "")
      foreach(path ${CMAKE_PREFIX_PATH})
        if(IS_DIRECTORY ${path}/share/${component}/cmake)
          list(APPEND paths ${path}/share/${component}/cmake)
        endif()
      endforeach()

      # find package component
      if(catkin_FIND_REQUIRED)
        # try without REQUIRED first
        find_package(${component} NO_MODULE PATHS ${paths}
          NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
        if(NOT ${component}_FOUND)
          # show better message to help users with the CMake error message coming up
          message(STATUS "Could not find the required component '${component}'. "
            "The following CMake error indicates that you either need to install the package "
            "with the same name or change your environment so that it can be found.")
          find_package(${component} REQUIRED NO_MODULE PATHS ${paths}
            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
        endif()
      elseif(catkin_FIND_QUIETLY)
        find_package(${component} QUIET NO_MODULE PATHS ${paths}
          NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      else()
        find_package(${component} NO_MODULE PATHS ${paths}
          NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      endif()

      # ROS Packages give their library packages via hard coded
      # paths. This makes it impossible to cross compile unless we fix
      # the paths like so below.
      if (USE_CTC)
      	if (${component}_LIBRARIES)
	        set(temp_LIBRARIES)
          foreach(library ${${component}_LIBRARIES})
            string(REGEX REPLACE "^/usr/lib" "${ARM_CHROOT_DIR}/usr/lib" library ${library})
            string(REPLACE "i386-linux-gnu" "arm-linux-gnueabihf" library ${library})
            string(REGEX REPLACE "^/opt/ros/kinetic" "${ARM_CHROOT_DIR}/opt/ros/kinetic" library ${library})
            list(APPEND temp_LIBRARIES ${library})
          endforeach()
          set(${component}_LIBRARIES ${temp_LIBRARIES})
        endif()
      endif(USE_CTC)

      # append component-specific variables to catkin_* variables
      list_append_unique(catkin_INCLUDE_DIRS ${${component}_INCLUDE_DIRS})

      # merge build configuration keywords with library names to correctly deduplicate
      catkin_pack_libraries_with_build_configuration(catkin_LIBRARIES ${catkin_LIBRARIES})
      catkin_pack_libraries_with_build_configuration(_libraries ${${component}_LIBRARIES})
      list_append_deduplicate(catkin_LIBRARIES ${_libraries})
      # undo build configuration keyword merging after deduplication
      catkin_unpack_libraries_with_build_configuration(catkin_LIBRARIES ${catkin_LIBRARIES})
      list(REMOVE_DUPLICATES catkin_LIBRARIES)

      list_append_unique(catkin_LIBRARY_DIRS ${${component}_LIBRARY_DIRS})
      list(APPEND catkin_EXPORTED_TARGETS ${${component}_EXPORTED_TARGETS})
    endif()
  endforeach()
  list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
  list_insert_in_workspace_order(catkin_LIBRARY_DIRS ${catkin_LIBRARY_DIRS})
endif()

# add dummy target to catkin_EXPORTED_TARGETS if empty
if(NOT catkin_EXPORTED_TARGETS)
  if(NOT TARGET _catkin_empty_exported_target)
    add_custom_target(_catkin_empty_exported_target)
  endif()
  list(APPEND catkin_EXPORTED_TARGETS _catkin_empty_exported_target)
endif()

# decrement recursion counter
math(EXPR _CATKIN_FIND_ "${_CATKIN_FIND_} - 1")

if(_CATKIN_FIND_ EQUAL 0)
  # store found components (from the fist level only) for validation in catkin_package() that they are build dependencies
  list(APPEND catkin_ALL_FOUND_COMPONENTS ${catkin2_FIND_COMPONENTS})
endif()
