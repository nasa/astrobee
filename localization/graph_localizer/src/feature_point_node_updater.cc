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

#include <graph_localizer/feature_point_node_updater.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace sym = gtsam::symbol_shorthand;

FeaturePointNodeUpdater::FeaturePointNodeUpdater(std::shared_ptr<gtsam::Values> values)
    : feature_point_graph_values_(new FeaturePointGraphValues(std::move(values))) {}

void FeaturePointNodeUpdater::AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& factors) {}

void FeaturePointNodeUpdater::AddInitialValuesAndPriors(const lc::FeaturePoint3d& global_t_point,
                                                        const lc::FeaturePoint3dNoise& noise,
                                                        gtsam::NonlinearFactorGraph& factors) {}

void FeaturePointNodeUpdater::AddPriors(const lc::FeaturePoint3d& global_t_point, const lc::FeaturePoint3dNoise& noise,
                                        gtsam::NonlinearFactorGraph& factors) {}

bool FeaturePointNodeUpdater::SlideWindow(const lc::Time oldest_allowed_timestamp,
                                          const boost::optional<gtsam::Marginals>& marginals,
                                          const gtsam::KeyVector& old_keys, const double huber_k,
                                          gtsam::NonlinearFactorGraph& factors) {}

go::NodeUpdaterType FeaturePointNodeUpdater::type() const { return go::NodeUpdaterType::FeaturePoint3d; }

bool FeaturePointNodeUpdater::Update(const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {}

boost::optional<lc::Time> FeaturePointNodeUpdater::SlideWindowNewOldestTime() const {
  return feature_point_graph_values_->SlideWindowOldestTime();
}

gtsam::KeyVector FeaturePointNodeUpdater::OldKeys(const localization_common::Time oldest_allowed_time,
                                                  const gtsam::NonlinearFactorGraph& graph) const {
  return feature_point_graph_values_->OldKeys(oldest_allowed_time, graph);
}

boost::optional<gtsam::Key> FeaturePointNodeUpdater::GetKey(KeyCreatorFunction key_creator_function,
                                                            const localization_common::Time timestamp) const {
  return feature_point_graph_values_->GetKey(key_creator_function, timestamp);
}

boost::optional<localization_common::Time> FeaturePointNodeUpdater::OldestTimestamp() const {
  return feature_point_graph_values_->OldestTimestamp();
}

boost::optional<localization_common::Time> FeaturePointNodeUpdater::LatestTimestamp() const {
  return feature_point_graph_values_->LatestTimestamp();
}

}  // namespace graph_localizer
