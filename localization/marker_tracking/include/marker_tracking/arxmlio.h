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

#ifndef MARKER_TRACKING_ARXMLIO_H_
#define MARKER_TRACKING_ARXMLIO_H_

#include <Eigen/Core>

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <memory>

namespace marker_tracking {

  // In this structure. The order of the rows are:
  // 0 - BL
  // 1 - BR
  // 2 - TR
  // 3 - TL
  typedef std::map<int, Eigen::Matrix<float, 4, 3>,
                   std::less<int>,
                   Eigen::aligned_allocator<std::pair<int const, Eigen::Matrix<float, 4, 3>> > >
    ARTagMap;
  void LoadARTagLocation(std::string const& filename,
                         ARTagMap* ar_corner_world_location);

}  // end namespace marker_tracking

#endif  // MARKER_TRACKING_ARXMLIO_H_
