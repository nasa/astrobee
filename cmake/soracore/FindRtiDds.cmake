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

######################################################################
# Find script for RTI DDS 
#
#
# Includes the GenerateRtiIdl macro for IDL generation
# and finds the rtiddsgen executable
#
# Standard environment variable for RTI DDS is NDDSHOME
# 
# Note that RTI DDS *requires* some defines to be passed to the 
# compiler. This script will define the RTIDDS_DEFINES variable
# for this purpose - you should call 
#  add_srcdir_definitions(${RTIDDS_DEFINES}) 
# in your CMakeLists.txt file. 
#
# The RTIDDS_DEFINE_FLAGS variable is deprecated because the 
# add_definitions command is not scoped to current directory so it
# caused problems with nested directory structures. 
#
# Output Variables:
# -----------------------
# RTIDDS_FOUND          : TRUE if search succeded
# RTIDDS_DEFINES        : Necessary platform defines (put add_srcdir_definitions(${RTIDDS_DEFINES}) in your CMakeLists))
# RTIDDS_INCLUDE_DIR    : include directories
# RTIDDS_LIBRARIES      : all libraries in one variable (use this in your CMakeLists)
# RTIDDS_LIBRARY_DIR    : library path
# RTIDDS_IDL_COMMAND    : full path to IDL compiler
# RTIDDS_DEFINE_FLAGS   : (DEPRECATED) Necessary platform defines (put add_definitions(${RTIDDS_DEFINE_FLAGS}) in your CMakeLists))
#
# Limited Bandwidth Plugin Variables:
# -----------------------
# RTIDDS_LB_FOUND       : true if Limited Bandwidth plugin found
# RTIDDS_LB_LIBRARIES   : all Limited Bandwidth plugin libraries
# RTIDDS_LB_INCLUDE_DIR : include path for Limited Bandwidth plugin
#
# Monitoring Variables:
# -----------------------
# RTIDDS_MON_FOUND       : true if Monitoring library found
# RTIDDS_MON_LIBRARIES   : all Monitoring libraries
#
# Distributed Logger Variables:
# -----------------------
# RTIDDS_DLOGGER_FOUND       : true if Distributed Logger library found
# RTIDDS_DLOGGER_LIBRARIES   : all Distributed Logger libraries
#
######################################################################

include( GetPackageLibSearchPath )
include( GetLibraryList )
include( TestBigEndian )

# include the idl generation macro in the Find 
# script so we don't have to explicitly include 
# it elsewhere
#--------------------------------------------------
include( GenerateRtiDdsIdl )

# init some variables
set( RTIDDS_LB_FOUND FALSE )
set( RTIDDS_MON_FOUND FALSE )

# extra environment variables for idl compiler, if necessary
set( EXTRA_ENVIRONMENT "" )

# platform defines
#--------------------------------------------------
if( WIN32 )
  set( IDL_COMMAND_FILENAME rtiddsgen.bat )
  set( RTIDDS_DEFINE_FLAGS -DRTI_WIN32 -DNDDS_DLL_VARIABLE )
  set( RTIDDS_DEFINES       RTI_WIN32 NDDS_DLL_VARIABLE )
elseif( APPLE )
  set( IDL_COMMAND_FILENAME rtiddsgen )
  set( RTIDDS_DEFINE_FLAGS -DRTI_UNIX -DRTI_DARWIN )
  set( RTIDDS_DEFINES       RTI_UNIX RTI_DARWIN )
else ( WIN32 )
  set( IDL_COMMAND_FILENAME rtiddsgen )
  set( RTIDDS_DEFINE_FLAGS -DRTI_UNIX -DRTI_LINUX )
  set( RTIDDS_DEFINES       RTI_UNIX RTI_LINUX )
endif( WIN32 )

test_big_endian(BIGENDIAN)
if( NOT BIGENDIAN ) 
  set( RTIDDS_DEFINE_FLAGS ${RTIDDS_DEFINE_FLAGS} -DRTI_ENDIAN_LITTLE )
  set( RTIDDS_DEFINES      ${RTIDDS_DEFINES} RTI_ENDIAN_LITTLE )
endif( NOT BIGENDIAN )

# set the scripts/bin search path 
#--------------------------------------------------
set( SCRIPTS_SEARCH_PATH "" )
if( RTIDDS_ROOT_DIR ) 

  set( SCRIPTS_SEARCH_PATH ${RTIDDS_ROOT_DIR}/bin 
                           ${RTIDDS_ROOT_DIR}/scripts 
  )
  
else( RTIDDS_ROOT_DIR ) 

  set( PKG_DIR_NAME ndds )
  # default search path
  set( SCRIPTS_SEARCH_PATH ${SCRIPTS_SEARCH_PATH}
    ${IRG_PACKAGES_DIR}/${PKG_DIR_NAME}/bin
    ${IRG_PACKAGES_DIR}/${PKG_DIR_NAME}/scripts
    /usr/local/${PKG_DIR_NAME}/bin
    /usr/local/${PKG_DIR_NAME}/scripts
    /usr/local/bin
    /usr/bin
    c:/devel/${PKG_DIR_NAME}/bin
    c:/devel/${PKG_DIR_NAME}/scripts
  )
  
  # We'll check a couple of env vars, because
  # RTI likes NDDSHOME, but IRG convention is <PACKAGE>_ROOT
  set( ENV_VAR_NAMES RTIDDS_ROOT NDDSHOME )
  foreach( ENV_VAR_NAME ${ENV_VAR_NAMES} )
    set( ENV_VAR_VALUE $ENV{${ENV_VAR_NAME}} )
    if( ENV_VAR_VALUE )
        set( SCRIPTS_SEARCH_PATH ${ENV_VAR_VALUE}/bin  
                                 ${ENV_VAR_VALUE}/scripts 
        )
    endif( ENV_VAR_VALUE )
  endforeach( ENV_VAR_NAME ${ENV_VAR_NAMES} )
  if (NOT DEFINED ENV{NDDSHOME})
    set(SCRIPTS_SEARCH_PATH /opt/rti/ndds/bin
                               /opt/rti/ndds/scripts)
  endif (NOT DEFINED ENV{NDDSHOME})
  
endif( RTIDDS_ROOT_DIR ) 

# look for rtiddsgen script
#--------------------------------------------------
find_file(RTIDDS_IDL_COMMAND  ${IDL_COMMAND_FILENAME}
  HINTS ${SCRIPTS_SEARCH_PATH}
  DOC "Path to RTI DDS IDL compiler"
)

###################################################
## If IDL compiler was found, we can proceed
###################################################
if( NOT RTIDDS_IDL_COMMAND-NOTFOUND )

  string(REGEX REPLACE "/[^/]*/[^/]*$" "" _RTIDDS_ROOT_DIR ${RTIDDS_IDL_COMMAND})
  # resolve any symlinks
  get_filename_component(RTIDDS_ROOT_DIR ${_RTIDDS_ROOT_DIR} REALPATH)
  set( NDDSHOME ${RTIDDS_ROOT_DIR} )
  
  set( ENV_NDDSARCH $ENV{NDDSARCH} )
  # guess the rti architecture string
  # this is absolutely horrible... even in their own 
  # scripts they have a fragile way to guess their
  # architecture string. So, we do our best...
  #--------------------------------------------------
  if( ENV_NDDSARCH ) 
    set(RTIDDS_ARCHITECTURE ${ENV_NDDSARCH})
  else( $ENV{NDDSARCH} )
    file( GLOB RTILIB_SUBDIRS ${RTIDDS_ROOT_DIR}/lib/* )
    foreach( SUBDIR ${RTILIB_SUBDIRS} )
      if( NOT ${SUBDIR} MATCHES "jdk$" ) # we don't want the JDK dir
        set( RTILIB_SUBDIR ${SUBDIR} )
      endif( NOT ${SUBDIR} MATCHES "jdk$" )
    endforeach( SUBDIR ${RTILIB_SUBDIRS} )
    if( NOT RTILIB_SUBDIR ) 
      file( GLOB RTILIB_SUBDIR_CONTENTS ${RTIDDS_ROOT_DIR}/lib/* )
      message(STATUS "  ERROR!! The script cannot determine the RTI architecture string.")
      message(STATUS "  ERROR!! This may indicate that you have a mismatch between your machine architecture and the installed RTI libraries.")
      message(STATUS "    FindRtiDds.cmake DEBUG INFORMATION:")
      message(STATUS "          RTIDDS_ROOT_DIR=${RTIDDS_ROOT_DIR}")
      message(STATUS "           RTILIB_SUBDIRS=${RTILIB_SUBDIRS}")
      message(STATUS "            RTILIB_SUBDIR=${RTILIB_SUBDIR}")
      message(STATUS "                 ARCH_CPU=${ARCH_CPU}")
      message(STATUS "                  RTI_CPU=${RTI_CPU} (expected)")
      message(STATUS "                  ARCH_OS=${ARCH_OS}")
      message(STATUS "                   RTI_OS=${RTI_OS} (expected)")
      message(STATUS "          contents of ${RTIDDS_ROOT_DIR}/lib/:")
      message(STATUS "              ${RTILIB_SUBDIR_CONTENTS}")
      message(STATUS "")
      message(STATUS "  ...the next command WILL FAIL: ")
      message(STATUS "")
    endif( NOT RTILIB_SUBDIR )
    string(REGEX MATCH "[^/]*$" RTIDDS_ARCHITECTURE ${RTILIB_SUBDIR})    
  endif( ENV_NDDSARCH )

  set( RTIDDS_INCLUDE_DIR  ${RTIDDS_ROOT_DIR}/include ${RTIDDS_ROOT_DIR}/include/ndds )
  set( RTIDDS_LIBRARY_DIR  ${RTIDDS_ROOT_DIR}/lib/${RTIDDS_ARCHITECTURE})
  
  # find full paths to all the libraries
  #--------------------------------------------------
  set( RTIDDS_LIBRARY_NAMES
    nddsc
    nddscore
    nddscpp
  )
  get_library_list(RTIDDS ${RTIDDS_LIBRARY_DIR} "d" "${RTIDDS_LIBRARY_NAMES}" TRUE)
  # include dirs not set there
  foreach( LIBRARY_NAME ${RTIDDS_LIBRARIES} ) 
    set_property(TARGET ${LIBRARY_NAME} PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${RTIDDS_INCLUDE_DIR}")
  endforeach( LIBRARY_NAME ${RTIDDS_LIBRARIES} ) 
  
  # Find NDDS version by looking at ndds_version.h
  #--------------------------------------------------
  find_file(NDDS_VERSION_H "ndds_version.h"
	  HINTS ${RTIDDS_ROOT_DIR}/include/ndds
	  NO_DEFAULT_PATH
	  NO_CMAKE_FIND_ROOT_PATH
          )
  
  set(RTIDDS_VERSION "")
  if(NDDS_VERSION_H)
    file(STRINGS ${NDDS_VERSION_H} VERSIONS_TMP REGEX "^#define RTI_DDS_VERSION_[A-Z]+[ \t]+['a-h0-9]+$")
    string(REGEX REPLACE ".*#define RTI_DDS_VERSION_MAJOR[ \t]+([0-9]+).*" "\\1" RTIDDS_VERSION_MAJOR ${VERSIONS_TMP})
    string(REGEX REPLACE ".*#define RTI_DDS_VERSION_MINOR[ \t]+([0-9]+).*" "\\1" RTIDDS_VERSION_MINOR ${VERSIONS_TMP})
    if(RTIDDS_VERSION_MAJOR LESS 5) # as of 5.0.0, RTI is using numbers instead of letters as the release version
      string(REGEX REPLACE ".*#define RTI_DDS_VERSION_RELEASE[ \t]+'([a-z]+)'.*" "\\1" RTIDDS_VERSION_RELEASE ${VERSIONS_TMP})
    else(RTIDDS_VERSION_MAJOR LESS 5)
      string(REGEX REPLACE ".*#define RTI_DDS_VERSION_RELEASE[ \t]+([0-9]+).*" "\\1" RTIDDS_VERSION_RELEASE ${VERSIONS_TMP})
    endif(RTIDDS_VERSION_MAJOR LESS 5)
    string(REGEX REPLACE ".*#define RTI_DDS_VERSION_REVISION[ \t]+([0-9]+).*" "\\1" RTIDDS_VERSION_REVISION ${VERSIONS_TMP})
    set(RTIDDS_VERSION ${RTIDDS_VERSION_MAJOR}.${RTIDDS_VERSION_MINOR}.${RTIDDS_VERSION_RELEASE} CACHE STRING "" FORCE)
  else(NDDS_VERSION_H)
    message( STATUS "  Could not find ndds_version.h, probably a pre-4.5e version")
  endif(NDDS_VERSION_H)
  
  if( RTIDDS_MISSING_LIBRARIES )
    message( STATUS "  Could not find the following RTI/DDS libraries:\n  ${RTIDDS_MISSING_LIBRARIES}")
  else( RTIDDS_MISSING_LIBRARIES )
    set( RtiDds_FOUND TRUE )
    #message(STATUS "  Found RTI DDS version ${RTIDDS_VERSION} r${RTIDDS_VERSION_REVISION} in ${RTIDDS_ROOT_DIR}")
  endif( RTIDDS_MISSING_LIBRARIES )
  
  # RHWS5 needs libdl, so find it and add to library list
  #--------------------------------------------------
  if(UNIX)
    find_library( RTIDDS_LIBDL dl )
    if( RTIDDS_LIBDL )
      add_library(RTIDDS_dl UNKNOWN IMPORTED )
      set_property(TARGET RTIDDS_dl PROPERTY IMPORTED_LOCATION "${RTIDDS_LIBDL}" )
      set( RTIDDS_LIBRARIES ${RTIDDS_LIBRARIES} RTIDDS_dl )
    endif( RTIDDS_LIBDL )
  endif(UNIX)
  
  ###################################################
  ## RTI Limited Bandwidth Plugin
  ##
  ## On Windows, only DLLs are provided (for loading
  ## through XML), so we cannot link directly to 
  ## the LB plugins. 
  ## TODO: Create a check for Windows that lets us
  ## know whether the LB plugin is present 
  ###################################################
  if( WIN32 )
    message(STATUS "  FIXME: RTI DDS Limited Bandwidth Plugin detection doesn't work on windows")
  endif( WIN32 )
  if(RTIDDS_VERSION) 
    # 4.5e (first RTI DDS to have a version header) has proper names
    set( RTIDDS_LB_LIBRARY_NAMES
      #lbsm   #simulation manager
      #lbsmz  #simulation manager
      #rtilbst
      rtilbedisc
      rtilbpdisc
      rtilbrtps 
      rtizrtps
    )  
    
  else(RTIDDS_VERSION)
    set( RTIDDS_LB_BETA_VERSION TRUE )
    add_definitions( -DRTIDDS_LB_BETA_VERSION )
    # the beta release libraries (for 4.5d) had stupid names. 
    # Proactively find the debug libraries, then
    # use get_library_list to find the rest. 
    #--------------------------------------------------
    find_library( RTIDDS_LB_LBEDiscoveryPlugin++_LIBRARY_DEBUG  LBEDiscoveryPlugind++  ${RTIDDS_LIBRARY_DIR} )
    find_library( RTIDDS_LB_LBPDiscoveryPlugin++_LIBRARY_DEBUG  LBPDiscoveryPlugind++  ${RTIDDS_LIBRARY_DIR} )
    set( RTIDDS_LB_LIBRARY_NAMES
      LBEDiscoveryPlugin
      LBEDiscoveryPlugin++
      LBPDiscoveryPlugin
      LBPDiscoveryPlugin++
    )
  
  endif(RTIDDS_VERSION)
  
  get_library_list(RTIDDS_LB ${RTIDDS_LIBRARY_DIR} "d" "${RTIDDS_LB_LIBRARY_NAMES}" TRUE)
  # include dirs not set there
  foreach( LIBRARY_NAME ${RTIDDS_LB_LIBRARIES} ) 
    set_property(TARGET ${LIBRARY_NAME} PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${RTIDDS_INCLUDE_DIR}")
  endforeach( LIBRARY_NAME ${RTIDDS_LB_LIBRARIES} ) 
  
  if( RTIDDS_LB_MISSING_LIBRARIES )
    set( RTIDDS_LB_FOUND FALSE )
    #message(STATUS "  RTI DDS Limited Bandwidth Plugin was NOT found.")
  else( RTIDDS_LB_MISSING_LIBRARIES ) 
  
    if(RTIDDS_LB_BETA_VERSION) # more hackish stuff for the 4.5d beta plugin. This should be removed at some point. 
      # find the header paths
      find_file( RTIDDS_LBEDiscoveryPlugin_H   "LBEDiscoveryPlugin/LBEDiscoveryPlugin.h" ${RTIDDS_INCLUDE_DIR} )
      find_file( RTIDDS_LBEDiscoveryPluginPP_H "LBEDiscoveryPlugin/wrappers/C++/LBEDiscoveryPlugin++.h" ${RTIDDS_INCLUDE_DIR} )
      find_file( RTIDDS_LBPDiscoveryPlugin_H   "LBPDiscoveryPlugin/LBPDiscoveryPlugin.h" ${RTIDDS_INCLUDE_DIR} )
      find_file( RTIDDS_LBPDiscoveryPluginPP_H "LBPDiscoveryPlugin/wrappers/C++/LBPDiscoveryPlugin++.h" ${RTIDDS_INCLUDE_DIR} )
      mark_as_advanced(RTIDDS_LBEDiscoveryPlugin_H)
      mark_as_advanced(RTIDDS_LBEDiscoveryPluginPP_H)
      mark_as_advanced(RTIDDS_LBPDiscoveryPlugin_H)
      mark_as_advanced(RTIDDS_LBPDiscoveryPluginPP_H)
      string(REGEX REPLACE "/[^/]*$" "" INCLUDE1 ${RTIDDS_LBEDiscoveryPlugin_H})
      string(REGEX REPLACE "/[^/]*$" "" INCLUDE2 ${RTIDDS_LBEDiscoveryPluginPP_H})
      string(REGEX REPLACE "/[^/]*$" "" INCLUDE3 ${RTIDDS_LBPDiscoveryPlugin_H})
      string(REGEX REPLACE "/[^/]*$" "" INCLUDE4 ${RTIDDS_LBPDiscoveryPluginPP_H})
      set( RTIDDS_LB_INCLUDE_DIR  ${INCLUDE1} ${INCLUDE2} ${INCLUDE3} ${INCLUDE4} )
    endif(RTIDDS_LB_BETA_VERSION)
    
    set( RTIDDS_LB_FOUND TRUE )
    #message(STATUS "  RTI DDS Limited Bandwidth Plugin found in ${RTIDDS_ROOT_DIR}")
    
  endif( RTIDDS_LB_MISSING_LIBRARIES )
  
  ###################################################
  ## RTI Monitor Library
  ###################################################
  set( RTIDDS_MON_LIBRARY_NAMES
    rtimonitoring
  )
  get_library_list(RTIDDS_MON ${RTIDDS_LIBRARY_DIR} "d" "${RTIDDS_MON_LIBRARY_NAMES}" TRUE)
  # include dirs not set there
  foreach( LIBRARY_NAME ${RTIDDS_MON_LIBRARIES} ) 
    set_property(TARGET ${LIBRARY_NAME} PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${RTIDDS_INCLUDE_DIR}")
  endforeach( LIBRARY_NAME ${RTIDDS_MON_LIBRARIES} ) 
  
  if( RTIDDS_MON_MISSING_LIBRARIES )
    set( RTIDDS_MON_FOUND FALSE )
    #message(STATUS "  RTI DDS Monitoring Library was NOT found.")
  else( RTIDDS_MON_MISSING_LIBRARIES ) 
    set( RTIDDS_MON_FOUND TRUE )
    #message(STATUS "  RTI DDS Monitoring Library found in ${RTIDDS_ROOT_DIR}")
  endif( RTIDDS_MON_MISSING_LIBRARIES )
  
  
  ###################################################
  ## RTI Distributed Logger Library
  ###################################################
  set( RTIDDS_DLOGGER_LIBRARY_NAMES
    rtidlc
    rtidlcpp
  )
  get_library_list(RTIDDS_DLOGGER ${RTIDDS_LIBRARY_DIR} "d" "${RTIDDS_DLOGGER_LIBRARY_NAMES}" TRUE)
  # include dirs not set there
  foreach( LIBRARY_NAME ${RTIDDS_LOGGER_LIBRARIES} ) 
    set_property(TARGET ${LIBRARY_NAME} PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${RTIDDS_INCLUDE_DIR}")
  endforeach( LIBRARY_NAME ${RTIDDS_LOGGER_LIBRARIES} ) 
  
  if( RTIDDS_DLOGGER_MISSING_LIBRARIES )
    set( RTIDDS_DLOGGER_FOUND FALSE )
    #message(STATUS "  RTI DDS Distributed Logging Library was NOT found.")
  else( RTIDDS_DLOGGER_MISSING_LIBRARIES ) 
    set( RTIDDS_DLOGGER_FOUND TRUE )
    #message(STATUS "  RTI DDS Distributed Logging Library found in ${RTIDDS_ROOT_DIR}")
  endif( RTIDDS_DLOGGER_MISSING_LIBRARIES )
else( NOT RTIDDS_IDL_COMMAND-NOTFOUND )
  set( RtiDds_FOUND FALSE )
endif( NOT RTIDDS_IDL_COMMAND-NOTFOUND )

#message(STATUS "RTIDDS_nddscpp_LIBRARY = ${RTIDDS_nddscpp_LIBRARY}" )
