/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "ros_log.h"

#ifndef MATLAB_MEX_FILE
#include <ros/console.h>
#else
#include <stdio.h>
#endif

void ros_log(int level, unsigned char* t) {
  char* s = (char*)t;  // matlab wants to pass unsigned char
#ifdef MATLAB_MEX_FILE
  puts(s);
#else
  switch (level) {
    case LOG_LEVEL_INFO:
      ROS_INFO(s);
      break;
    case LOG_LEVEL_WARN:
      ROS_WARN(s);
      break;
    case LOG_LEVEL_ERROR:
      ROS_ERROR(s);
      break;
    default:
      ROS_DEBUG(s);
      break;
  }
#endif
}

