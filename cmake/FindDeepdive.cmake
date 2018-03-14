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

# - Try to find libdeepdive
# Will define
# DEEPDIVE_FOUND
# DEEPDIVE_INCLUDE_DIRS
# DEEPDIVE_LIBRARIES

find_path(DEEPDIVE_INCLUDE_DIRS deepdive/deepdive.h
  HINTS ${DEEPDIVE_INCLUDEDIR})

find_library(DEEPDIVE_LIBRARIES
  NAMES deepdive
  HINTS ${DEEPDIVE_LIBDIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(deepdive DEFAULT_MSG DEEPDIVE_LIBRARIES DEEPDIVE_INCLUDE_DIRS)
mark_as_advanced(DEEPDIVE_LIBRARIES DEEPDIVE_INCLUDE_DIRS)
