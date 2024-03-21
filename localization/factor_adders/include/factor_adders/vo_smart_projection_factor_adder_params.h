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

#ifndef FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_PARAMS_H_
#define FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_PARAMS_H_

#include <factor_adders/factor_adder_params.h>
#include <vision_common/spaced_feature_tracker_params.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/SmartFactorParams.h>

namespace factor_adders {
struct VoSmartProjectionFactorAdderParams : public FactorAdderParams {
  vision_common::SpacedFeatureTrackerParams spaced_feature_tracker;
  // Maximum number of smart factors to include in a graph at a time.
  int max_num_factors;
  // Minimum number of points for a feature track to be used for a smart factor.
  int min_num_points_per_factor;
  // Maximum number of points in a feature track to include in a smart factor.
  int max_num_points_per_factor;
  // Minimum average deviation for points in a feature track to be used.
  // A higher deviation provides a larger baseline between points and is less likely
  // to yield numerical errors during triangulation and optimization.
  double min_avg_distance_from_mean;
  // Use a robust loss for the factor.
  bool robust;
  // Minimum distance in image space between the latest measurement in a feature track
  // and already included latest measurements in other feature tracks for the feature track
  // to be used as a smart factor.
  /* double feature_track_min_separation;*/
  // If triangulation fails, use a rotation-only version of the smart factor.
  // Otherwise, the smart factor is disabled.
  bool rotation_only_fallback;
  // Attempt to fix smart factors that are invalid by removing individual measurements
  // and if this fails measurement sequences.
  bool fix_invalid_factors;
  // Scale the noise with the number of points, as a longer track the relies on a
  // longer history of poses tends to have higher error
  bool scale_noise_with_num_points;
  // Relative noise scale for all smart factors.
  double noise_scale;
  // Camera extrinsics.
  gtsam::Pose3 body_T_cam;
  // Camera intrinsics.
  boost::shared_ptr<gtsam::Cal3_S2> cam_intrinsics;
  // Camera noise.
  gtsam::SharedIsotropic cam_noise;
  // GTSAM Smart factor params, see GTSAM documentation and below for more details.
  // enableEPI: Refine triangulation using Levenberg-Marquardt Optimization.
  // verboseCheirality: Print error on cheirality errors
  // landmarkDistanceThreshold: Maximum valid distance for a triangulated point.
  // dynamicOutlierRejectionThreshold: Maximum valid reprojection error for a valid triangulated point.
  // retriangulationThreshold: Equality threshold for poses in a smart factor to trigger retriangulation
  // of a landmark point. If any of the poses vary by more than this threshold
  // compared to their previous value, retriangulation is triggered.
  gtsam::SmartProjectionParams smart_factor;
};
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_PARAMS_H_
