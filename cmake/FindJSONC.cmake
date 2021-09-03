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

# - Try to find JSONC
# Will define
# JSONC_FOUND - If Succeful
# JSONC_INCLUDE_DIRS - The JSONC include directories
# JSONC_LIBRARIES - The JSONC libraty

find_path(JSONC_INCLUDE_DIR
  NAMES "json-c/json.h"
  PATHS /usr/include /usr/local/include
  )

find_library(JSONC_LIBRARY
  NAMES "json-c"
  PATHS /usr/lib /usr/local/lib
  )

set(JSONC_INCLUDE_DIRS "${JSONC_INCLUDE_DIR}")
set(JSONC_LIBRARIES "${JSONC_LIBRARY}" )


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(JSONC DEFAULT_MSG JSONC_LIBRARY JSONC_INCLUDE_DIR)
mark_as_advanced(JSONC_LIBRARY JSONC_INCLUDE_DIR)
