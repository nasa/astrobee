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

#ifndef LOCALIZATION_COMMON_MARGINALS_POSE_COVARIANCE_INTERPOLATER_H_
#define LOCALIZATION_COMMON_MARGINALS_POSE_COVARIANCE_INTERPOLATER_H_

#include <localization_common/pose_covariance_interpolater.h>
#include <localization_common/utilities.h>

#include <boost/optional.hpp>
#include <gtsam/nonlinear/Marginals.h>

namespace localization_common {
template <typename NodesT>
class MarginalsPoseCovarianceInterpolater : public PoseCovarianceInterpolater {
 public:
  MarginalsPoseCovarianceInterpolater(const std::shared_ptr<const NodesT>& nodes,
                                      const boost::optional<const gtsam::Marginals&>& marginals = boost::none);
  PoseCovariance Interpolate(const PoseWithCovariance& a, const PoseWithCovariance& b, const Time& timestamp_a,
                             const Time& timestamp_b) final;

 private:
  boost::optional<const gtsam::Marginals&> marginals_;
  std::shared_ptr<const NodesT> nodes_;
};

// Implementation
template <typename NodesT>
MarginalsPoseCovarianceInterpolater<NodesT>::MarginalsPoseCovarianceInterpolater(
  const std::shared_ptr<const NodesT>& nodes, const boost::optional<const gtsam::Marginals&>& marginals)
    : nodes_(nodes), marginals_(marginals) {}

template <typename NodesT>
PoseCovariance MarginalsPoseCovarianceInterpolater<NodesT>::Interpolate(const PoseWithCovariance& a,
                                                                        const PoseWithCovariance& b,
                                                                        const Time& timestamp_a,
                                                                        const Time& timestamp_b) {
  boost::optional<PoseCovariance> covariance_a_b;
  // Set covariance_a_b if available
  if (marginals_ && nodes_) {
    const auto closest_t_a = nodes_->ClosestTimestamp(timestamp_a);
    const auto closest_t_b = nodes_->ClosestTimestamp(timestamp_b);
    if (!closest_t_a || !closest_t_b) {
      LogError("Interpolate: Failed to get closest timestamps.");
    } else {
      // TODO(rsoussan): Add fallback covariance/relative covariance if gap too large
      if (std::abs(*closest_t_a - timestamp_a) > 3.0 || std::abs(*closest_t_b - timestamp_b) > 3.0) {
        LogWarning("Interpolate: Timestamps far from closest timestamps available.");
      }
      const auto keys_a = nodes_->Keys(*closest_t_a);
      const auto keys_b = nodes_->Keys(*closest_t_b);
      covariance_a_b = marginals_->jointMarginalCovariance({keys_a[0], keys_b[0]}).fullMatrix();
    }
  }
  return RelativePoseCovariance(a, b, covariance_a_b);
}
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_MARGINALS_POSE_COVARIANCE_INTERPOLATER_H_
