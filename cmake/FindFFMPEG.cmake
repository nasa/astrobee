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

# - Try to find FFMPEG
# Will define
# FFMPEG_FOUND
# FFMPEG_INCLUDE_DIRS
# FFMPEG_LIBRARIES

find_package(PkgConfig QUIET)
pkg_check_modules(PC_AVCODEC QUIET libavcodec)
pkg_check_modules(PC_AVFORMAT QUIET libavformat)
pkg_check_modules(PC_AVUTIL QUIET libavutil)
pkg_check_modules(PC_SWSCALE QUIET libswscale)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h
  HINTS ${PC_AVCODEC_INCLUDEDIR} ${PC_AVOCDEC_INCLUDE_DIRS}
  )
find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h
  HINTS ${PC_AVFORMAT_INCLUDEDIR} ${PC_AVFORMAT_INCLUDE_DIRS}
  )
find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h
  HINTS ${PC_AVUTIL_INCLUDEDIR} ${PC_AVUTIL_INCLUDE_DIRS}
  )
find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h
  HINTS ${PC_SWSCALE_INCLUDEDIR} ${PC_SWSCALE_INCLUDE_DIRS}
  )

find_library(AVCODEC_LIBRARY
  NAMES avcodec libavcodec
  HINTS ${PC_AVOCDEC_LIBDIR} ${PC_AVCODEC_LIBRARY_DIRS}
  )
find_library(AVFORMAT_LIBRARY
  NAMES avformat libavformat
  HINTS ${PC_AVFORMAT_LIBDIR} ${PC_AVFORMAT_LIBRARY_DIRS}
  )
find_library(AVUTIL_LIBRARY
  NAMES avutil libavutil
  HINTS ${PC_AVUTIL_LIBDIR} ${PC_AVUTIL_LIBRARY_DIRS}
  )
find_library(SWSCALE_LIBRARY
  NAMES swscale libswscale
  HINTS ${PC_SWSCALE_LIBDIR} ${PC_SWSCALE_LIBRARY_DIRS}
  )

set(FFMPEG_INCLUDE_DIRS "${AVFORMAT_INCLUDE_DIR};${AVCODEC_INCLUDE_DIR};${AVUTIL_INCLUDE_DIR};${SWSCALE_INCLUDE_DIR}")
set(FFMPEG_LIBRARIES "${AVFORMAT_LIBRARY};${AVCODEC_LIBRARY};${AVUTIL_LIBRARY};${SWSCALE_LIBRARY}" )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FFMPEG DEFAULT_MSG FFMPEG_LIBRARIES FFMPEG_INCLUDE_DIRS)
mark_as_advanced(FFMPEG_LIBRARIES FFMPEG_INCLUDE_DIRS)
