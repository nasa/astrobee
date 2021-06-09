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

#include <localization_common/logger.h>
#include <localization_measurements/measurement_conversions.h>
#include <localization_measurements/plane.h>

#include <gtsam/geometry/Pose3.h>

#include <gtest/gtest.h>

namespace lm = localization_measurements;

TEST(MeasurementsConversionTester, PointToPlaneDistanceMakeHandrailPlane) {
  const gtsam::Pose3 world_T_handrail;
  const double distance_to_wall = 0.5;
  const auto world_T_handrail_plane = lm::MakeHandrailPlane(world_T_handrail, distance_to_wall);
  // In front of plane
  {
    const gtsam::Point3 test_point(1, 0, 0);
    const double distance = world_T_handrail_plane.Distance(test_point);
    EXPECT_NEAR(1.5, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(1, 2, 3);
    const double distance = world_T_handrail_plane.Distance(test_point);
    EXPECT_NEAR(1.5, distance, 1e-6);
  }
  // Behind plane
  {
    const gtsam::Point3 test_point(-1, 0, 0);
    const double distance = world_T_handrail_plane.Distance(test_point);
    EXPECT_NEAR(-0.5, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(-1, 2, 3);
    const double distance = world_T_handrail_plane.Distance(test_point);
    EXPECT_NEAR(-0.5, distance, 1e-6);
  }
  // On plane
  {
    const gtsam::Point3 test_point(-0.5, 0, 0);
    const double distance = world_T_handrail_plane.Distance(test_point);
    EXPECT_NEAR(0, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(-0.5, 2, 3);
    const double distance = world_T_handrail_plane.Distance(test_point);
    EXPECT_NEAR(0, distance, 1e-6);
  }
}
