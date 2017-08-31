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

# Wrap the miro MakeParams executable.
# Generates cpp and h output from a list of .xml input files
#
# If there is an EXPORT_NAME variable defined, it will
# be treated as an ACE Export directive in the generated source.
# Output filesname are stored in MIRO_MAKEPARAMS_GENERATED
#
# ! Depends on the MIRO_MAKE_PARAMS_EXECUTABLE variable pointing
# to the appropriate place.
macro( miro_makeparams )

  # by default, install makeParams configuration files
  # into $INSTALL/etc/MiroParameterDescription for use
  # with ConfigEditor. To disable installation, set
  # MIRO_MAKEPARAMS_CONFIG_INSTALL_DIR to empty string
  #-----------------------------------------------------
  set( MIRO_MAKEPARAMS_CONFIG_INSTALL_DIR "/etc/MiroParameterDescription" CACHE STRING "install dir for MiroParameterDescription xml config files")
  mark_as_advanced( MIRO_MAKEPARAMS_CONFIG_INSTALL_DIR )

  # on unix, set the LD_LIBRARY_PATH explicitly so
  # ACE doesn't have to be in ld path
  #-----------------------------------------------------
  set( MAKEPARAMS_LIB_VAR "" )
  if( UNIX )
    set( LD_PATH_VAR LD_LIBRARY_PATH )
    if( APPLE ) # "think different", indeed
      set( LD_PATH_VAR DYLD_LIBRARY_PATH )
    endif( APPLE )
    set( ORIGINAL_LD_PATH $ENV{${LD_PATH_VAR}} )
    set( MAKEPARAMS_LIB_VAR "${LD_PATH_VAR}=${ORIGINAL_LD_PATH}:${ACE_ROOT_DIR}/lib" )
  endif( UNIX )

  # in-source files need to see out-of-source files
  include_directories( "${CMAKE_CURRENT_BINARY_DIR}" )

  # the generated files need to reference the *_Export files,
  # so copy them to the out-of-source tree to avoid nasty
  # include path referencing mess
  #-----------------------------------------------------
  if( EXPORT_FILE )
    message(STATUS "Copying ${EXPORT_FILE} file to out-of-source tree... (GenerateMiroMakeParams)" )
    exec_program("${CMAKE_COMMAND}" ARGS
      -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/${EXPORT_FILE}"
      "${CMAKE_CURRENT_BINARY_DIR}/${EXPORT_FILE}"
    )
  endif( EXPORT_FILE )

  set( MIRO_MAKEPARAMS_HEADERS "" )
  set( MIRO_MAKEPARAMS_SOURCES "" )
  set( MIRO_MAKEPARAMS_GENERATED "" )

  foreach( PARAMS_FILENAME in ${ARGN} )
    # skip first parameter which is "in"
    if( NOT PARAMS_FILENAME STREQUAL "in" )

      ## Generate the Parameters.* files
      ## ------------------------------------------------------
      get_filename_component( PARAMS_BASE ${PARAMS_FILENAME} NAME_WE )
      get_filename_component( PARAMS_PATH ${PARAMS_FILENAME} PATH )
      if(PARAMS_PATH)
        # makeParams silently fails if the target directory doesn't exist
        # this should be fixed in makeParams, but for now we will
        # proactively create the subdirectory
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${PARAMS_PATH}")
        set(PARAMS_PATH "${PARAMS_PATH}/")
      endif(PARAMS_PATH)

      set( MIRO_MAKEPARAMS_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/${PARAMS_PATH}${PARAMS_BASE}.h   ${MIRO_MAKEPARAMS_HEADERS} )
      set( MIRO_MAKEPARAMS_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/${PARAMS_PATH}${PARAMS_BASE}.cpp ${MIRO_MAKEPARAMS_SOURCES} )
      set( MIRO_MAKEPARAMS_GENERATED ${MIRO_MAKEPARAMS_HEADERS} ${MIRO_MAKEPARAMS_SOURCES} )

      set( PARAMS_BASE_FULL_PATH ${CMAKE_CURRENT_BINARY_DIR}/${PARAMS_PATH}${PARAMS_BASE} )

      set( PARAMS_OUTPUT
        ${PARAMS_BASE_FULL_PATH}.h
        ${PARAMS_BASE_FULL_PATH}.cpp
      )

      set( EXTRA_MIRO_PARAMS_ARGS "" )
      if( EXPORT_NAME )
        set( EXTRA_MIRO_PARAMS_ARGS -x ${EXPORT_NAME} )
      endif( EXPORT_NAME )

      add_custom_command(
        OUTPUT  ${PARAMS_OUTPUT}
        DEPENDS ${PARAMS_FILENAME}
        COMMAND ${MAKEPARAMS_LIB_VAR} ${MIRO_MAKEPARAMS_EXECUTABLE}
        ARGS
          -f ${CMAKE_CURRENT_SOURCE_DIR}/${PARAMS_FILENAME}
          -n ${PARAMS_BASE_FULL_PATH} ${MIRO_PARAMS_ARGS}
          ${EXTRA_MIRO_PARAMS_ARGS}
      )

      set_source_files_properties(
        ${PARAMS_OUTPUT}
        PROPERTIES
        GENERATED True
      )

      # install source xml config
      if( MIRO_MAKEPARAMS_CONFIG_INSTALL_DIR )
        install_files( ${MIRO_MAKEPARAMS_CONFIG_INSTALL_DIR} FILES ${PARAMS_FILENAME} )
      endif( MIRO_MAKEPARAMS_CONFIG_INSTALL_DIR )

    endif( NOT PARAMS_FILENAME STREQUAL "in" )

  endforeach( PARAMS_FILENAME )

endmacro( miro_makeparams )

