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
# Define a macro to generate IDL output
# After this macro is called, 
#
# 2013/06/03 - mallan: RTI added support for custom dll export 
#              macros through the -dllExportMacroSuffix command line
#              option. 
#
# Upon completion of this macro, the following variables will exist
#
# RTIDDS_IDL_GENERATED - all files generated (add this to your target)
#   
# RTIDDS_IDL_GENERATED_HEADERS - generated header files (used for include
#                                file install)
#
# RTIDDS_EXPORT - if EXPORT_SUFFIX is defined when this macro is called, 
#                 the export variable will be NDDS_USER_DLL_EXPORT_${EXPORT_SUFFIX}
#                 otherwise, it will be set to the default NDDS_USER_DLL_EXPORT
#
# To add additional includes to the idl compiler command line, add them
# to RTIDDS_IDL_INCLUDES before calling the macro:
# set( RTIDDS_IDL_INCLUDES -I/my/foo/include ${RTIDDS_IDL_INCLUDES )
#
#############################################################
macro( rtidds_wrap_idl )

  set( EXTRA_ENVIRONMENT "" )
  
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

  # in-source files need to see out-of-source files
  include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
        
  set( EXT_HDR         ".h" )
  set( EXT_SRC         ".cxx" )
  set( EXT_Support_HDR "Support.h" )
  set( EXT_Support_SRC "Support.cxx" )
  set( EXT_Plugin_HDR  "Plugin.h" )
  set( EXT_Plugin_SRC  "Plugin.cxx" )
  set( EXT_XML         ".xml" )

  set( RTIDDS_EXPORT "NDDS_USER_DLL_EXPORT" )
  set( RTIDDS_IDL_FLAGS -language C++ -replace -namespace )
  set( RTIDDS_XML_FLAGS -replace -namespace -convertToXml)
  if( WIN32 )
    if( EXPORT_SUFFIX )
      set(  RTIDDS_EXPORT "NDDS_USER_DLL_EXPORT_${EXPORT_SUFFIX}" )
       set( RTIDDS_IDL_FLAGS ${RTIDDS_IDL_FLAGS} -dllExportMacroSuffix ${EXPORT_SUFFIX} )
    endif( EXPORT_SUFFIX )
  endif( WIN32 )

  set( SRCDIR ${CMAKE_CURRENT_SOURCE_DIR} )
  set( OOSDIR ${CMAKE_CURRENT_BINARY_DIR} )

  # start with clean vars
  set( RTIDDS_IDL_GENERATED "" )
  set( RTIDDS_IDL_GENERATED_HEADERS "" )
  set( RTIDDS_IDL_GENERATED_XMLS "" )
  
  get_filename_component(IDL_COMMAND_PATH ${RTIDDS_IDL_COMMAND} DIRECTORY)
  find_file(RTIDDS_IDL_SERVER  
            NAMES rtiddsgen_server
            HINTS ${IDL_COMMAND_PATH}
            DOC "Path to RTI DDS IDL compiler (server mode)"
  )
  set(_RTIDDS_IDL_COMMAND ${RTIDDS_IDL_COMMAND})
  if(RTIDDS_IDL_SERVER)
    message(STATUS "Found ${RTIDDS_IDL_SERVER}: will use that instead, to speed up build")
    set(_RTIDDS_IDL_COMMAND ${RTIDDS_IDL_SERVER})
  endif(RTIDDS_IDL_SERVER)
  
  
  # add a custom command set for idl files
  #-----------------------------------------------------
  foreach( IDL_FILENAME ${ARGN} )

    # get the basename (i.e. "NAME Without Extension")
    get_filename_component( IDL_BASE ${IDL_FILENAME} NAME_WE )

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
    set( DEPEND_FILE_LIST ${SRCDIR}/${IDL_BASE}.idl )
    
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
    foreach( IDL_DEP_FULL_FILENAME ${ARGN} )
      get_filename_component( IDL_DEP_BASE ${IDL_DEP_FULL_FILENAME} NAME_WE )

      if( IDL_FILE_CONTENTS MATCHES ${IDL_DEP_BASE}\\.idl AND 
          NOT IDL_DEP_FULL_FILENAME STREQUAL IDL_FILENAME)

        # Target will need to depend on the output file, not the idl,
        # so that included included dependencies work correctly.
        set( DEPEND_FILE_LIST ${DEPEND_FILE_LIST} ${OOSDIR}/${IDL_DEP_BASE}${EXT_HDR} )

      endif( IDL_FILE_CONTENTS MATCHES ${IDL_DEP_BASE}\\.idl AND 
             NOT IDL_DEP_FULL_FILENAME STREQUAL IDL_FILENAME)
    endforeach( IDL_DEP_FULL_FILENAME ${ARGN} )

    if( DEBUG_IDL_DEPENDENCIES)
      message("${IDL_OUTPUT_FILES} depends on ${DEPEND_FILE_LIST}\n")
    endif (DEBUG_IDL_DEPENDENCIES)


    #message( STATUS "COMMAND = ${EXTRA_ENVIRONMENT} ${_RTIDDS_IDL_COMMAND}" )
    #message( STATUS "   ARGS = ${RTIDDS_IDL_FLAGS} ${EXTRA_RTIDDS_IDL_ARGS}" )
    #message( STATUS "          -I${SRCDIR} ${RTIDDS_IDL_INCLUDES} " )
    #message( STATUS "          -d ${OOSDIR} ${SRCDIR}/${IDL_BASE}.idl" )
    
    # setup the command
    # RTI DDS 5.2 IDL compiler will no longer generate c++ and xml 
    # with the same invocation of rtiddsgen, so we have to run it twice. 
    #-----------------------------------------------------
    if(RTIDDS_VERSION VERSION_LESS 5.2.0)
       add_custom_command(
        OUTPUT   ${IDL_OUTPUT_FILES}
        DEPENDS  ${DEPEND_FILE_LIST}
        COMMAND  ${EXTRA_ENVIRONMENT} ${_RTIDDS_IDL_COMMAND}
        ARGS ${RTIDDS_IDL_FLAGS} ${EXTRA_RTIDDS_IDL_ARGS} -convertToXml
            -I${SRCDIR} ${RTIDDS_IDL_INCLUDES} 
            -d ${OOSDIR} ${SRCDIR}/${IDL_BASE}.idl
      )
    else(RTIDDS_VERSION VERSION_LESS 5.2.0)
      add_custom_command(
        OUTPUT   ${IDL_OUTPUT_FILES}
        DEPENDS  ${DEPEND_FILE_LIST}
        COMMAND  ${EXTRA_ENVIRONMENT} ${_RTIDDS_IDL_COMMAND}
        ARGS ${RTIDDS_IDL_FLAGS} ${EXTRA_RTIDDS_IDL_ARGS}
            -I${SRCDIR} ${RTIDDS_IDL_INCLUDES} 
            -d ${OOSDIR} ${SRCDIR}/${IDL_BASE}.idl
        COMMAND  ${EXTRA_ENVIRONMENT} ${_RTIDDS_IDL_COMMAND}
        ARGS ${RTIDDS_XML_FLAGS} ${EXTRA_RTIDDS_IDL_ARGS}
            -I${SRCDIR} ${RTIDDS_IDL_INCLUDES} 
            -d ${OOSDIR} ${SRCDIR}/${IDL_BASE}.idl
      )
    endif(RTIDDS_VERSION VERSION_LESS 5.2.0)
        
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
    
    ## copy header to devel/include if we are using catkin
    ## and header is under ${PROJECT}/src/...
    ##---------------------------------------------------
    string( REGEX MATCH "${CMAKE_SOURCE_DIR}/.*/src/.*" IS_SRC_MODULE ${CMAKE_CURRENT_SOURCE_DIR} )
    if(catkin_FOUND AND IS_SRC_MODULE )
      # extract "module" path. Requires that directories are named ${PROJECT}/src/${MODULE}
      string( REGEX REPLACE "${CMAKE_SOURCE_DIR}/.*/src/" "" MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" )

      set( RTIDDSIDL_OUTPUT_TARGET RtiDdsIdl_${IDL_BASE} )
      add_custom_target( ${RTIDDSIDL_OUTPUT_TARGET} ALL DEPENDS ${IDL_OUTPUT_FILES} )

      set( RTIDDSIDL_DEVEL_DEST ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${MODULE_PATH} )
      foreach( IDL_GENERATED_HEADER ${IDL_OUTPUT_HEADERS} )
        get_filename_component( IDL_HEADER_FILENAME ${IDL_GENERATED_HEADER} NAME )
        add_custom_command(
          TARGET ${RTIDDSIDL_OUTPUT_TARGET}
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_if_different ${IDL_GENERATED_HEADER} ${RTIDDSIDL_DEVEL_DEST}/${IDL_HEADER_FILENAME}
        )
      endforeach()
    endif( catkin_FOUND AND IS_SRC_MODULE )
  endforeach( IDL_FILENAME )

endmacro( rtidds_wrap_idl )


