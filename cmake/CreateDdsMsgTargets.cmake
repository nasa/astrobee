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

## RTIDDS IDL
# Define a function to generate IDL output
#
# 2013/06/03 - mallan: RTI added support for custom dll export
#              macros through the -dllExportMacroSuffix command line
#              option.
#
# 2015/06/11 - tfmorse: Stop stripping the full path from IDL files
#              and convert from a macro into a function
#
# Upon completion of this function, the following variables will be set
# in the parent's scope
#
# ${NAME}_GENERATED - all files generated (add this to your target)
#
# ${NAME}_GENERATED_HEADERS - generated header files (used for include
#                             file install)
#
#############################################################
function(create_dds_msg_targets)
  cmake_parse_arguments(
    msg           # Prefix of output variables
    ""            # List of booleans
    "DIR;NAME"    # List of mono valued arguments
    "INCLUDES;EXTRA_ARGS"    # List of multi-value arguments
    ${ARGN})

  set( EXTRA_ENVIRONMENT "" )

  file( GLOB IDL_FILES ${msg_DIR}/*.idl )

  if( APPLE )
    # rtiddsgen is broken on macs that don't include xalan as a system jar,
    # symptom is when running rtiddsgen, you get the following output:
    ###> Running rtiddsgen version 4.5e, please wait ...
    ###> ERROR: 'The first argument to the non-static Java function 'parseScopedName' is not a valid object reference.'
    ###> FATAL ERROR: 'Could not compile stylesheet'
    # To fix this, download xalan (http://xml.apache.org/xalan-j/) from here:
    # ftp://apache.mirrors.pair.com/xml/xalan-j/xalan-j_2_7_1-bin.zip
    # and unzip into ${NDDSHOME}/class
    set( EXTRA_ENVIRONMENT "XALANHOME=${NDDSHOME}/class" )
    message(STATUS "** GenerateRtiDdsIdl.cmake: adding ${EXTRA_ENVIRONMENT} to idl compiler environment because rtiddsgen is broken on OSX")

    # we also require an explicit path to the C pre processor on OSX because the default cpp
    # is a script apparently and RTI can't pass arguments to it properly
    set( EXPLICIT_CPP -ppPath /usr/bin/cpp-${GCC_MINORVERSION})
    set( EXTRA_RTIDDS_IDL_ARGS ${EXTRA_RTIDDS_IDL_ARGS} ${EXPLICIT_CPP} )
    message(STATUS "** GenerateRtiDdsIdl.cmake: adding ${EXPLICIT_CPP} to idl compiler command line because rtiddsgen is broken on OSX")
  endif( APPLE )

  set( EXT_HDR         ".h" )
  set( EXT_SRC         ".cxx" )
  set( EXT_Support_HDR "Support.h" )
  set( EXT_Support_SRC "Support.cxx" )
  set( EXT_Plugin_HDR  "Plugin.h" )
  set( EXT_Plugin_SRC  "Plugin.cxx" )
  set( EXT_XML         ".xml" )

  set( RTIDDS_EXPORT "NDDS_USER_DLL_EXPORT" )
  set( RTIDDS_IDL_FLAGS -language C++ -replace -namespace -convertToXml)
  if( WIN32 )
    if( EXPORT_SUFFIX )
      set(  RTIDDS_EXPORT "NDDS_USER_DLL_EXPORT_${EXPORT_SUFFIX}" )
      set( RTIDDS_IDL_FLAGS ${RTIDDS_IDL_FLAGS} -dllExportMacroSuffix ${EXPORT_SUFFIX} )
    endif( EXPORT_SUFFIX )
  endif( WIN32 )

  set( OOSDIR ${CMAKE_CURRENT_BINARY_DIR} )

  # start with clean vars
  set( RTIDDS_IDL_GENERATED "" )
  set( RTIDDS_IDL_GENERATED_HEADERS "" )
  set( RTIDDS_IDL_GENERATED_XMLS "" )

  # add a custom command set for idl files
  #-----------------------------------------------------
  foreach( IDL_FILENAME ${IDL_FILES} )

    # get the basename (i.e. "NAME Without Extension")
    get_filename_component( IDL_BASE ${IDL_FILENAME} NAME_WE )
    get_filename_component( IDL_DIR ${IDL_FILENAME} DIRECTORY )

    set( IDL_OUTPUT_HEADERS
      ${OOSDIR}/${IDL_BASE}${EXT_HDR}
      ${OOSDIR}/${IDL_BASE}${EXT_Support_HDR}
      ${OOSDIR}/${IDL_BASE}${EXT_Plugin_HDR}
    )

    set( IDL_OUTPUT_SOURCES
      ${OOSDIR}/${IDL_BASE}${EXT_SRC}
      ${OOSDIR}/${IDL_BASE}${EXT_Support_SRC}
      ${OOSDIR}/${IDL_BASE}${EXT_Plugin_SRC}
    )

    set( IDL_TYPE_XML
      ${OOSDIR}/${IDL_BASE}${EXT_XML}
    )

    set( IDL_OUTPUT_FILES 
      ${IDL_OUTPUT_HEADERS}
      ${IDL_OUTPUT_SOURCES}
    )

    # output files depend on at least the corresponding idl
    set( DEPEND_FILE_LIST ${IDL_FILENAME} )

    # load the contents of the idl file
    file( READ ${IDL_FILENAME} IDL_FILE_CONTENTS )

    # look for other dependencies
    ################################################################
    ## FIXME mallan 12/5/2009 : this method of finding dependencies
    ## is broken... idl filenames that are not in include statements
    ## (e.g. a filename in a comment...) that will be listed
    ## as a dependency. As this was a cut and paste of the TAO
    ## IDL script, that script is broken, too
    ################################################################
    foreach( IDL_DEP_FULL_FILENAME ${IDL_FILES} )
      get_filename_component( IDL_DEP_NAME ${IDL_DEP_FULL_FILENAME} NAME )
      get_filename_component( IDL_DEP_BASE ${IDL_DEP_FULL_FILENAME} NAME_WE )

      if( IDL_FILE_CONTENTS MATCHES ${IDL_DEP_NAME} AND
          NOT IDL_DEP_FULL_FILENAME STREQUAL IDL_FILENAME)

        # Target will need to depend on the output file, not the idl,
        # so that included included dependencies work correctly.
        set( DEPEND_FILE_LIST ${DEPEND_FILE_LIST}
             ${OOSDIR}/${IDL_DEP_BASE}${EXT_HDR} )

      endif( IDL_FILE_CONTENTS MATCHES ${IDL_DEP_NAME} AND
             NOT IDL_DEP_FULL_FILENAME STREQUAL IDL_FILENAME)
    endforeach( IDL_DEP_FULL_FILENAME ${ARGN} )

    if( DEBUG_IDL_DEPENDENCIES)
      message("${IDL_OUTPUT_FILES} depends on ${DEPEND_FILE_LIST}\n")
    endif (DEBUG_IDL_DEPENDENCIES)

    #message( STATUS "COMMAND = ${EXTRA_ENVIRONMENT} ${RTIDDS_IDL_COMMAND}" )
    #message( STATUS "   ARGS = ${RTIDDS_IDL_FLAGS} ${EXTRA_RTIDDS_IDL_ARGS}" )
    #message( STATUS "          -I${SRCDIR} ${RTIDDS_IDL_INCLUDES} " )
    #message( STATUS "          -d ${OOSDIR} ${SRCDIR}/${IDL_BASE}.idl" )

    # setup the command
    #-----------------------------------------------------
    add_custom_command(
      OUTPUT   ${IDL_OUTPUT_FILES}
      DEPENDS  ${DEPEND_FILE_LIST}
      COMMAND  ${EXTRA_ENVIRONMENT} ${RTIDDS_IDL_COMMAND}
      ARGS ${RTIDDS_IDL_FLAGS} ${msg_EXTRA_ARGS}
          -I${IDL_DIR} ${msg_INCLUDES}
          -d ${OOSDIR} ${IDL_FILENAME}
    )

    set( RTIDDS_IDL_GENERATED_HEADERS
      ${IDL_OUTPUT_HEADERS}
      ${RTIDDS_IDL_GENERATED_HEADERS}
    )

    set( RTIDDS_IDL_GENERATED
      ${IDL_OUTPUT_FILES}
      ${RTIDDS_IDL_GENERATED}
    )

    set( RTIDDS_IDL_GENERATED_XMLS
      ${IDL_TYPE_XML}
      ${RTIDDS_IDL_GENERATED_XMLS}
    )

  endforeach( IDL_FILENAME )

  SET(${msg_NAME}_HEADERS
      ${RTIDDS_IDL_GENERATED_HEADERS} PARENT_SCOPE)
  SET(${msg_NAME}_GENERATED
      ${RTIDDS_IDL_GENERATED} PARENT_SCOPE)

endfunction()
