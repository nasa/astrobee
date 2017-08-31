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

#include "fault_assert.h"

#include <stdio.h>
#include <string>

#ifndef MATLAB_MEX_FILE
#include <ff_util/ff_nodelet.h>
#include <ros/console.h>
ff_util::FreeFlyerNodelet* global_gnc_nodelet = NULL;

ff_util::FreeFlyerNodelet* get_global_nodelet(void) {return global_gnc_nodelet;}
void set_global_nodelet(ff_util::FreeFlyerNodelet* n) {global_gnc_nodelet = n;}
#endif

void fault_assert(unsigned char* key, unsigned char* t) {
  char* k = (char*)key;  // matlab wants to pass unsigned char
  char* s = (char*)t;  // matlab wants to pass unsigned char
#ifdef MATLAB_MEX_FILE
  printf("Triggered Fault %s: %s\n", key, t);
#else
  if (global_gnc_nodelet == NULL) {
    ROS_FATAL("Cannot trigger fault, global_gnc_nodelet not set.");
  } else {
    global_gnc_nodelet->AssertFault(std::string(k), std::string(s));
  }
#endif
}

