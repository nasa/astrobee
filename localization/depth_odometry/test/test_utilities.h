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
#ifndef DEPTH_ODOMETRY_TEST_UTILITIES_H_  // NOLINT
#define DEPTH_ODOMETRY_TEST_UTILITIES_H_  // NOLINT

#include <localization_measurements/depth_image_measurement.h>

namespace depth_odometry {
localization_measurements::DepthImageMeasurement DefaultDepthImageMeasurement(
  const localization_common::Time timestamp);

}
#endif  // DEPTH_ODOMETRY_TEST_UTILITIES_H_ // NOLINT
