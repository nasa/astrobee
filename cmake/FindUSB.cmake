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

# - Try to find USB
# Will define
# USB_FOUND - If Succeful
# USB_INCLUDE_DIRS - The USB include directories
# USB_LIBRARIES - The USB libraty

find_path(USB_INCLUDE_DIR
  NAMES "libusb-1.0/libusb.h"
  PATHS /usr/include /usr/local/include
  )

find_library(USB_LIBRARY
  NAMES "usb-1.0"
  PATHS /usr/lib /usr/local/lib
  )

set(USB_INCLUDE_DIRS "${USB_INCLUDE_DIR}")
set(USB_LIBRARIES "${USB_LIBRARY}" )


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(USB DEFAULT_MSG USB_LIBRARY USB_INCLUDE_DIR)
mark_as_advanced(USB_LIBRARY USB_INCLUDE_DIR)
