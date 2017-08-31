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

#include "fault_clear.h"

#include <stdio.h>
#include <string>

#ifndef MATLAB_MEX_FILE
#include <ff_util/ff_nodelet.h>
#include <ros/console.h>
// nodelet defined
#include "fault_assert.h"
#endif

void fault_clear(unsigned char* key) {
  char* k = (char*)key;
#ifdef MATLAB_MEX_FILE
  printf("Cleared Fault %s.\n", k);
#else
  ff_util::FreeFlyerNodelet* nodelet = get_global_nodelet();
  if (nodelet == NULL) {
    ROS_FATAL("Cannot clear fault, global_gnc_nodelet not set.\n");
  } else {
    nodelet->ClearFault(std::string(k));
  }
#endif
}

