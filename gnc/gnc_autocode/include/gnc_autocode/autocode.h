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

#ifndef GNC_AUTOCODE_AUTOCODE_H_
#define GNC_AUTOCODE_AUTOCODE_H_

#include <fault_assert.h>

// this needs to be called so that we can assert faults from the autocode
namespace gnc_autocode {
  void InitializeAutocode(ff_util::FreeFlyerNodelet* nh);
}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_AUTOCODE_H_
