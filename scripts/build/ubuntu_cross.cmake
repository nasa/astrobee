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

# adds support for the ubuntu cross compile toolchain

# this one is important
SET(CMAKE_SYSTEM_NAME Linux)

# this one is needed by picoflexx drivers
SET(CMAKE_SYSTEM_PROCESSOR "armv7")

# this one not so much
SET(CMAKE_SYSTEM_VERSION 1)

if (DEFINED ENV{ARMHF_CHROOT_DIR})
  SET(ARM_CHROOT_DIR $ENV{ARMHF_CHROOT_DIR})
else()
  SET(ARM_CHROOT_DIR $ENV{HOME}/armhf_xenial_chroot)
endif()

SET(CMAKE_SYSROOT ${ARM_CHROOT_DIR})

SET(USE_CTC ON CACHE INTERNAL "" FORCE)
SET(CROSS_PREFIX arm-linux-gnueabihf)

# specify the cross compiler
SET(CMAKE_C_COMPILER  $ENV{ARMHF_TOOLCHAIN}/bin/${CROSS_PREFIX}-gcc)
SET(CMAKE_CXX_COMPILER  $ENV{ARMHF_TOOLCHAIN}/bin/${CROSS_PREFIX}-g++)

SET(CMAKE_LIBRARY_ARCHITECTURE ${CROSS_PREFIX})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# look for libraries, headers and packages in the target directories
# Specifically, make sure it finds the host's copy of the ARM RTI libraries.
# If this is not done, the resulting rpath will not look for them in the chroot
# environment.
# Also, RTI DDS is included once for the libraries and once for the headers.
execute_process(COMMAND catkin locate -i OUTPUT_VARIABLE CATKIN_INSTALL_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND catkin locate -d OUTPUT_VARIABLE CATKIN_DEVEL_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
SET(CMAKE_FIND_ROOT_PATH
  ${ARM_CHROOT_DIR} ${CATKIN_INSTALL_PATH} ${CATKIN_DEVEL_PATH})

SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
SET(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# fix pkg-config to only look in the chroot as well
SET(ENV{PKG_CONFIG_SYSROOT_DIR} ${ARM_CHROOT_DIR})
SET(ENV{PKG_CONFIG_LIBDIR} "${ARM_CHROOT_DIR}/usr/local/lib/arm-linux-gnueabihf/pkgconfig:${ARM_CHROOT_DIR}/usr/local/lib/pkgconfig:${ARM_CHROOT_DIR}/usr/local/share/pkgconfig:${ARM_CHROOT_DIR}/usr/lib/arm-linux-gnueabihf/pkgconfig:${ARM_CHROOT_DIR}/usr/lib/pkgconfig:${ARM_CHROOT_DIR}/usr/share/pkgconfig")

# extra places to look, for example.. RTI DDS
IF( DEFINED EXTRA_ROOT_PATH )
  SET(CMAKE_FIND_ROOT_PATH ${EXTRA_ROOT_PATH} ${CMAKE_FIND_ROOT_PATH})
ENDIF( DEFINED EXTRA_ROOT_PATH )

execute_process(COMMAND catkin locate -s OUTPUT_VARIABLE CATKIN_SRC_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
SET(catkin2_DIR ${CATKIN_SRC_PATH}/cmake)

# needed for gflag to compile...
SET( THREADS_PTHREAD_ARG 
    "PLEASE_FILL_OUT-FAILED_TO_RUN"
    CACHE STRING "Result from TRY_RUN" FORCE)

# needed to compile eigen, forces result that we are not 64 bit
SET( run_res 
     "1"
     CACHE STRING "Result from TRY_RUN" FORCE)

SET( run_res__TRYRUN_OUTPUT 
     ""
     CACHE STRING "Output from TRY_RUN" FORCE)

# IRG's SetArchitecture will fail because it uses the host's uname
# program to determine this, so we hardcode it.
SET( ARCHITECTURE "arm7l_linux_gcc4.8" CACHE STRING "Architecture" )

# NDDS cannot handle finding an architecture than is not i686 or x86_64,
# so we must force it to use an ARM target architecture. Unfortunately,
# they only look at environment variables, so we override the environment var.
SET( ENV{NDDSARCH} "armv6vfphLinux3.xgcc4.7.2" )

# Setup some custom compiler flags.
# --no-as-needed: forces all libraries, even unnecessary ones to be linked.
# -allow-multiple-definition: because GCC ARM sets every reference as strong.
# -rpath-link: so ros can find itself in the darkness
SET( CMAKE_EXE_LINKER_FLAGS
  "-Wl,-rpath-link,${ARM_CHROOT_DIR}/opt/ros/kinetic/lib:${ARM_CHROOT_DIR}/lib/${CROSS_PREFIX}:${ARM_CHROOT_DIR}/usr/lib/${CROSS_PREFIX}:${ARM_CHROOT_DIR}/usr/lib/${CROSS_PREFIX}/mesa-egl:${ARM_CHROOT_DIR}/usr/lib/${CROSS_PREFIX}/mesa:${CROSS_PREFIX}/opt/rti/ndds/lib/armv6vfphLinux3.xgcc4.7.2" 
	 CACHE STRING "" FORCE )
