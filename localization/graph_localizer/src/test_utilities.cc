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
#include <graph_localizer/test_utilities.h>
#include <localization_common/test_utilities.h>

#include <gtsam/geometry/Point3.h>

namespace graph_localizer {
namespace lc = localization_common;

localization_measurements::Plane RandomPlane() {
  gtsam::Point3 point = lc::RandomVector();
  gtsam::Vector3 normal = lc::RandomVector().normalized();
  return localization_measurements::Plane(point, normal);
}
}  // namespace graph_localizer
