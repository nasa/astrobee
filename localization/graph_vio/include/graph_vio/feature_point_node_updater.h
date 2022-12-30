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

#ifndef GRAPH_VIO_FEATURE_POINT_NODE_UPDATER_H_
#define GRAPH_VIO_FEATURE_POINT_NODE_UPDATER_H_

#include <graph_optimizer/node_updater_with_priors.h>
#include <graph_vio/feature_point_graph_values.h>
#include <graph_vio/feature_point_node_updater_params.h>
#include <localization_common/feature_point_3d.h>

namespace graph_vio {
class FeaturePointNodeUpdater
    : public graph_optimizer::NodeUpdaterWithPriors<localization_common::FeaturePoint3d,
                                                    localization_common::FeaturePoint3dNoise> {
 public:
  FeaturePointNodeUpdater(const FeaturePointNodeUpdaterParams& params, std::shared_ptr<gtsam::Values> values);
  void AddInitialValuesAndPriors(const localization_common::FeaturePoint3d& global_N_body,
                                 const localization_common::FeaturePoint3dNoise& noise,
                                 gtsam::NonlinearFactorGraph& factors) final;

  void AddPriors(const localization_common::FeaturePoint3d& global_N_body,
                 const localization_common::FeaturePoint3dNoise& noise, gtsam::NonlinearFactorGraph& factors) final;

  bool Update(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<gtsam::Marginals>& marginals, const gtsam::KeyVector& old_keys,
                   const double huber_k, gtsam::NonlinearFactorGraph& factors) final;

  void UpdatePointPriors(const gtsam::Marginals& marginals, gtsam::NonlinearFactorGraph& factors);

  graph_optimizer::NodeUpdaterType type() const final;

  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const final;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final;

  boost::optional<gtsam::Key> GetKey(graph_optimizer::KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const final;

  boost::optional<localization_common::Time> OldestTimestamp() const final;

  boost::optional<localization_common::Time> LatestTimestamp() const final;

  int NumFeatures() const;

  std::shared_ptr<FeaturePointGraphValues> shared_feature_point_graph_values();

  const FeaturePointGraphValues& feature_point_graph_values() const;

 private:
  FeaturePointNodeUpdaterParams params_;
  std::shared_ptr<FeaturePointGraphValues> feature_point_graph_values_;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_FEATURE_POINT_NODE_UPDATER_H_
