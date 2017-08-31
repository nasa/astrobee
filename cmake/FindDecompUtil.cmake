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

# - Try to find DECOMP_UTIL
# Will define
# DECOMP_UTIL_FOUND - If Succeful
# DECOMP_UTIL_INCLUDE_DIRS - The DECOMP_UTIL include directories
# DECOMP_UTIL_LIBRARIES - The DECOMP_UTIL libraty

find_path(DECOMP_UTIL_INCLUDE_DIR
  NAMES "decomp_util/data_type.h"
  PATHS /usr/include /usr/local/include
  )

find_library(DECOMP_UTIL_GEOMETRY_LIBRARY
  NAMES geometry_utils
  PATHS /usr/lib /usr/local/lib
  )
find_library(DECOMP_UTIL_ELLIPSOID_LIBRARY
  NAMES ellipsoid_utils
  PATHS /usr/lib /usr/local/lib
  )
find_library(DECOMP_UTIL_DECOMP_LIBRARY
  NAMES ellipse_decomp
  PATHS /usr/lib /usr/local/lib
  )
find_library(DECOMP_UTIL_ITERATIVE_LIBRARY
  NAMES iterative_decomp
  PATHS /usr/lib /usr/local/lib
  )

set(DECOMP_UTIL_INCLUDE_DIRS "${DECOMP_UTIL_INCLUDE_DIR}")
set(DECOMP_UTIL_LIBRARIES "${DECOMP_UTIL_GEOMETRY_LIBRARY};${DECOMP_UTIL_ELLIPSOID_LIBRARY};${DECOMP_UTIL_ITERATIVE_LIBRARY};${DECOMP_UTIL_DECOMP_LIBRARY}" )


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DECOMP_UTIL DEFAULT_MSG DECOMP_UTIL_GEOMETRY_LIBRARY DECOMP_UTIL_DECOMP_LIBRARY DECOMP_UTIL_ELLIPSOID_LIBRARY DECOMP_UTIL_ITERATIVE_LIBRARY DECOMP_UTIL_INCLUDE_DIR)
mark_as_advanced(DECOMP_UTIL_GEOMETRY_LIBRARY DECOMP_UTIL_DECOMP_LIBRARY DECOMP_UTIL_ELLIPSOID_LIBRARY DECOMP_UTIL_ITERATIVE_LIBRARY DECOMP_UTIL_INCLUDE_DIR)
