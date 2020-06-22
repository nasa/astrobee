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

# Check to see if correct version of media has been downloaded into tree. We
# dont do this if we are cross-compiling, as there is no need for it. We must
# also make sure the media is copied in a native install to a simulator.

# Add the things that need to be on both the robot and the simulation
add_subdirectory(description)

# Optional: install only for simulated context
if ((NOT USE_CTC) AND (NOT IS_BAMBOO_BUILD))
  set(FETCHCONTENT_QUIET OFF)
  FetchContent_Declare(media
    URL https://github.com/nasa/astrobee_media/archive/v0.0.7.tar.gz
    URL_HASH SHA1=5ad19f0e64d039d852a5beb21ad07442d61f5790
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/media
  )
  FetchContent_GetProperties(media)
  if(NOT media_POPULATED)
    FetchContent_Populate(media)
    add_subdirectory(${media_SOURCE_DIR}/astrobee_granite)
    add_subdirectory(${media_SOURCE_DIR}/astrobee_dock)
    add_subdirectory(${media_SOURCE_DIR}/astrobee_iss)
    add_subdirectory(${media_SOURCE_DIR}/astrobee_handrail_8_5)
    add_subdirectory(${media_SOURCE_DIR}/astrobee_handrail_21_5)
    add_subdirectory(${media_SOURCE_DIR}/astrobee_handrail_30)
    add_subdirectory(${media_SOURCE_DIR}/astrobee_handrail_41_5)
    add_subdirectory(${media_SOURCE_DIR}/astrobee_freeflyer)
  endif()
endif ((NOT USE_CTC) AND (NOT IS_BAMBOO_BUILD))
