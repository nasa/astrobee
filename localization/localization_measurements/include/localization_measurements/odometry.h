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

#ifndef LOCALIZATION_MEASUREMENTS_ODOMETRY_H_
#define LOCALIZATION_MEASUREMENTS_ODOMETRY_H_

#include <localization_common/pose_with_covariance.h>
#include <localization_common/time.h>
#include <localization_measurements/measurement.h>
#include <localization_measurements/odometry.h>

namespace localization_measurements {
struct Odometry {
  localization_common::Time source_time;
  localization_common::Time target_time;
  localization_common::PoseWithCovariance sensor_F_source_T_target;
  localization_common::PoseWithCovariance body_F_source_T_target;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_ODOMETRY_H_
