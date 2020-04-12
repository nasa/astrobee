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

# - Try to find ARGTABLE2
# Will define
# ARGTABLE2_FOUND - If Succeful
# ARGTABLE2_INCLUDE_DIRS - The ARGTABLE2 include directories
# ARGTABLE2_LIBRARIES - The ARGTABLE2 libraty

find_path(ARGTABLE2_INCLUDE_DIR
  NAMES "argtable2.h"
  PATHS /usr/include /usr/local/include
  )

find_library(ARGTABLE2_LIBRARY
  NAMES argtable2
  PATHS /usr/lib /usr/local/lib
  )

set(ARGTABLE2_INCLUDE_DIRS "${ARGTABLE2_INCLUDE_DIR}")
set(ARGTABLE2_LIBRARIES "${ARGTABLE2_LIBRARY}" )


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ARGTABLE2 DEFAULT_MSG ARGTABLE2_LIBRARY ARGTABLE2_INCLUDE_DIR)
mark_as_advanced(ARGTABLE2_LIBRARY ARGTABLE2_INCLUDE_DIR)
