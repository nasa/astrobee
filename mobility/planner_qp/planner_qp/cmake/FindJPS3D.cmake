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

# - Try to find JPS3D
# Will define
# JPS3D_FOUND - If Succeful
# JPS3D_INCLUDE_DIRS - The JPS3D include directories
# JPS3D_LIBRARIES - The JPS3D libraty

find_path(JPS3D_INCLUDE_DIR
  NAMES "jps3d/planner/jps_3d_util.h"
  PATHS /usr/include /usr/local/include
  )

find_library(JPS3D_LIBRARY
  NAMES jps_lib
  PATHS /usr/lib /usr/local/lib
  )

set(JPS3D_INCLUDE_DIRS "${JPS3D_INCLUDE_DIR}")
set(JPS3D_LIBRARIES "${JPS3D_LIBRARY}" )


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(JPS3D DEFAULT_MSG JPS3D_LIBRARY JPS3D_INCLUDE_DIR)
mark_as_advanced(JPS3D_LIBRARY JPS3D_INCLUDE_DIR)
