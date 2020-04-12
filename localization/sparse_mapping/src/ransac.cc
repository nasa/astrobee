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

#include <glog/logging.h>
#include <sparse_mapping/ransac.h>

#include <iostream>
#include <set>
#include <functional>

namespace sparse_mapping {

// Get a given number of unique random integers within a given range
// (inclusive). It is very important that the generator be stored
// outside, so we don't always initialize it to the same value. That
// would result in same random numbers each time this function is
// called which is very undesirable.
void get_n_unique_integers(int min_val, int max_val, int num,
                             std::mt19937 * generator, std::vector<int> * values) {
  // Sanity check
  if (max_val - min_val + 1 < num) {
    LOG(FATAL) << "Cannot get " << num << " unique integers in ["
               << min_val << ", " << max_val << "]" << std::endl;
  }

  values->clear();
  std::set<int> done;

  std::uniform_int_distribution < int> distribution(min_val, max_val);

  while (1) {
    if (values->size() == static_cast<size_t>(num))
      break;

    int val = distribution(*generator);

    // If this was encountered, skip it
    if (done.find(val) != done.end()) continue;

    done.insert(val);
    values->push_back(val);
  }
}

}  // namespace sparse_mapping
