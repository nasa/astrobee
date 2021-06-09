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

set(GTEST_SRC_DIR "/usr/src/gtest")
if (USE_CTC)
  set(GTEST_SRC_DIR "${ARM_CHROOT_DIR}/usr/src/gtest")
endif (USE_CTC)

# libgtest-dev must be installed!!
CMakeExternalProject(
  NAME gtest_noros
  SOURCE_DIR ${GTEST_SRC_DIR}
  )

set(GTEST_INCLUDE_DIRS ${gtest_noros_SOURCE_DIR}/include PARENT_SCOPE)
if (BUILD_SHARED_LIBS)
  if(EXISTS "${gtest_noros_BINARY_DIR}/libgtest${CMAKE_SHARED_LIBRARY_SUFFIX}")
      set(GTEST_LIBRARIES ${gtest_noros_BINARY_DIR}/libgtest${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${gtest_noros_BINARY_DIR}/libgtest_main${CMAKE_SHARED_LIBRARY_SUFFIX})
  else()
      set(GTEST_LIBRARIES ${gtest_noros_BINARY_DIR}/lib/libgtest${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${gtest_noros_BINARY_DIR}/lib/libgtest_main${CMAKE_SHARED_LIBRARY_SUFFIX})
  endif()

else (BUILD_SHARED_LIBS)
  set(GTEST_LIBRARIES ${gtest_noros_BINARY_DIR}/libgtest.a
    ${gtest_noros_BINARY_DIR}/libgtest_main.a)
endif (BUILD_SHARED_LIBS)

# We need GTEST_LIBRARIES both in the current scope and for the parent
# as well.  
set(GTEST_LIBRARIES ${GTEST_LIBRARIES} PARENT_SCOPE)
install(FILES ${GTEST_LIBRARIES} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

