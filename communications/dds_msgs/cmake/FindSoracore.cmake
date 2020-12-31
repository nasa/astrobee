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

# Set parent directory as a search location
string(REGEX REPLACE "/[^/]*$" "" PROJ_SRC_PARENT ${PROJECT_SOURCE_DIR})

if( NOT SORACORE_ROOT_DIR )
  set( SORACORE_ROOT_DIR "/usr" )
endif( NOT SORACORE_ROOT_DIR )

set( SORACORE_LIBRARY_DIR ${SORACORE_ROOT_DIR}/lib )
set( SORACORE_INCLUDE_DIRS ${SORACORE_ROOT_DIR}/include 
                          ${SORACORE_ROOT_DIR}/include/rapidDds 
                          ${SORACORE_ROOT_DIR}/include/rapidExtDds 
                          ${SORACORE_ROOT_DIR}/include/rapidExtArcDds )
set( SORACORE_IDL_DIR     ${SORACORE_ROOT_DIR}/idl )

set( LIBRARY_NAMES
  irgUtmll
  irgSha1
  knShare
  knMath
  knGeometry
  knFrameStore
  knMotorShare
  knDds
  knDdsUtil
  rapidFrameStore
  rapidDds
  rapidExtDds
  rapidExtArcDds
  rapidCommanding
  rapidUtil
  rapidIo
  rapidExtIo
  rapidExtArcIo
  knFrameStoreSvc
  knFetchPool
  knSystemInfo
  knSystemInfoSvc
  knRaft
  knProcessManager
  rapidExtTraclabsDds
  rapidExtTraclabsIo
)
#get_library_list(SORACORE ${SORACORE_LIBRARY_DIR} "d" "${LIBRARY_NAMES}")
get_library_imports(soracore "${SORACORE_LIBRARY_DIR}" "${LIBRARY_NAMES}")

set( SORACORE_FOUND TRUE )

