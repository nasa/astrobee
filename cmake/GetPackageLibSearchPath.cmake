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

# get_package_lib_search_path( PKG_DIR_NAME PKG_ROOT_DIR (PKG_ENV_VAR) )
######################################################################
# 
# This maro constructs an appropriate search path and 
# stores it in LIB_SEARCH_PATH
# The arguments are
# PKG_NAME      - name of the package (e.g. "CSpice")
# PKG_DIR_NAME  - name of the package directory (e.g. "cspice")
# PKG_ROOT_DIR  - the CMake root var name (e.g. "CSPICE_ROOT_DIR")
# PKG_ENV_VAR   - (optional) the env var that (may) contain the root path
#
# If PKG_ROOT_DIR is defined before this macro is called, 
# NO OTHER PATHS ARE SEARCHED. This is a way of "forcing" 
# the build to use a specified path.
# 
# If PKG_ENV_VAR contains a value, 
# NO OTHER PATHS ARE SEARCHED. 
# 
# Otherwise, a search path is constructed. See below for the 
# search path order. 
#
# It is assumed that IRG_PREFIX has been defined before calling
# 
# This macro does not actually do the search, it just constructs the path.
#
# useage:
# get_package_lib_search_path( CSpice cspice-1.0 CSPICE_ROOT_DIR CSPICE_ROOT )
#
# Output Variables:
# -----------------
# LIB_SEARCH_PATH : the path(s) that should be searched for a library
# LIB_SEARCH_ERROR_MESSAGE : a useful message to print if the search fails
#
######################################################################

macro( get_package_lib_search_path PKG_NAME PKG_DIR_NAME PKG_ROOT_DIR )

set(LIB_SEARCH_PATH "" ) # make sure we've got a clean variable

## If PKG_ROOT_DIR is defined, set 
## to that value and return
##########################################
if( ${PKG_ROOT_DIR} )

  set( LIB_SEARCH_PATH ${${PKG_ROOT_DIR}}/lib )
  set( LIB_SEARCH_ERROR_MESSAGE "  ${PKG_NAME} NOT found!!! ${PKG_ROOT_DIR} is set to \"${${PKG_ROOT_DIR}}\", but library test failed.")
  
else( ${PKG_ROOT_DIR}  )
  
  ##
  ## Default search path.
  ## Look in $INSTALL/lib for local 
  ## dependencies, then check irg and
  ## system paths
  ##########################################
  set( LIB_SEARCH_PATH ${LIB_SEARCH_PATH}
    ${CMAKE_INSTALL_PREFIX}/lib
  )
  foreach( ONE_DIR_NAME ${PKG_DIR_NAME} )
    set( LIB_SEARCH_PATH ${LIB_SEARCH_PATH}
      /usr/local/${ONE_DIR_NAME}/lib
    )
  endforeach( ONE_DIR_NAME )
  set( LIB_SEARCH_PATH ${LIB_SEARCH_PATH}
    /usr/local/lib
    /opt/local/lib
    /usr/lib
    /lib
  )
  foreach( ONE_DIR_NAME ${PKG_DIR_NAME} )
    set( LIB_SEARCH_PATH ${LIB_SEARCH_PATH}
      c:/devel/${ONE_DIR_NAME}/lib
    )
  endforeach( ONE_DIR_NAME )
  set( LIB_SEARCH_PATH ${LIB_SEARCH_PATH}
    c:/devel/lib
  )
  set( LIB_SEARCH_ERROR_MESSAGE "  ${PKG_NAME} NOT found!!! Try passing -D${PKG_ROOT_DIR}=<path> to cmake command")
  ##
  ## the fourth argument is the (optional) 
  ## environment variable name
  ##
  ## if the environment variable is set, 
  ## look only in that directory.
  ##########################################
  if( ${ARGC} EQUAL 4 ) 
    
    set( ENV_VAR_VALUE $ENV{${ARGV3}} )
    
    if( ENV_VAR_VALUE )
    
      # overwrite the default system search
      # path with the env var value
      #####################################
      message( STATUS "  ${ARGV3} environment variable is set to ${ENV_VAR_VALUE}" )
      
      set( LIB_SEARCH_PATH ${ENV_VAR_VALUE}/lib )
      set( LIB_SEARCH_ERROR_MESSAGE "  ${PKG_NAME} NOT found!!! ${ARGV3} env var is set to \"${ENV_VAR_VALUE}\", but library test failed.")
      
    else( ENV_VAR_VALUE )

      set( LIB_SEARCH_ERROR_MESSAGE "${LIB_SEARCH_ERROR_MESSAGE}, or setting the ${ARGV3} environment variable")
    
    endif( ENV_VAR_VALUE )
    
  endif( ${ARGC} EQUAL 4 ) 
        
endif( ${PKG_ROOT_DIR}  )

endmacro( get_package_lib_search_path )









