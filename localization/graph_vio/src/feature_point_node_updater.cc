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

#include <graph_optimizer/utilities.h>
#include <graph_vio/feature_point_node_updater.h>
#include <graph_vio/utilities.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

namespace graph_vio {
namespace go = graph_optimizer;
namespace gv = graph_values;
namespace lc = localization_common;
namespace sym = gtsam::symbol_shorthand;

// TODO(rsoussan): Make new node updater base class that doesn't have functions left empty here?
FeaturePointNodeUpdater::FeaturePointNodeUpdater(const FeaturePointNodeUpdaterParams& params,
                                                 std::shared_ptr<gtsam::Values> values)
    : params_(params), feature_point_graph_values_(new gv::FeaturePointGraphValues(std::move(values))) {}

void FeaturePointNodeUpdater::AddInitialValuesAndPriors(const lc::FeaturePoint3d& global_t_point,
                                                        const lc::FeaturePoint3dNoise& noise,
                                                        gtsam::NonlinearFactorGraph& factors) {}

void FeaturePointNodeUpdater::AddPriors(const lc::FeaturePoint3d& global_t_point, const lc::FeaturePoint3dNoise& noise,
                                        gtsam::NonlinearFactorGraph& factors) {}

bool FeaturePointNodeUpdater::SlideWindow(const lc::Time oldest_allowed_timestamp,
                                          const boost::optional<gtsam::Marginals>& marginals,
                                          const gtsam::KeyVector& old_keys, const double huber_k,
                                          gtsam::NonlinearFactorGraph& factors) {
  feature_point_graph_values_->RemoveOldFeatures(old_keys);
  if (marginals) UpdatePointPriors(*marginals, factors);
  return true;
}

void FeaturePointNodeUpdater::UpdatePointPriors(const gtsam::Marginals& marginals,
                                                gtsam::NonlinearFactorGraph& factors) {
  const auto feature_keys = feature_point_graph_values_->FeatureKeys();
  for (const auto& feature_key : feature_keys) {
    const auto world_t_point = feature_point_graph_values_->at<gtsam::Point3>(feature_key);
    if (!world_t_point) {
      LogError("UpdatePointPriors: Failed to get world_t_point.");
      continue;
    }
    for (auto factor_it = factors.begin(); factor_it != factors.end();) {
      const auto point_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Point3>*>(factor_it->get());
      if (point_prior_factor && (point_prior_factor->key() == feature_key)) {
        // Erase old prior
        factor_it = factors.erase(factor_it);
        // Add updated one
        const auto point_prior_noise = go::Robust(
          gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(feature_key)), params_.huber_k);
        const gtsam::PriorFactor<gtsam::Point3> point_prior_factor(feature_key, *world_t_point, point_prior_noise);
        factors.push_back(point_prior_factor);
        // Only one point prior per feature
        break;
      } else {
        ++factor_it;
      }
    }
  }
}

go::NodeUpdaterType FeaturePointNodeUpdater::type() const { return go::NodeUpdaterType::FeaturePoint; }

bool FeaturePointNodeUpdater::Update(const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) { return true; }

boost::optional<lc::Time> FeaturePointNodeUpdater::SlideWindowNewOldestTime() const {
  return feature_point_graph_values_->SlideWindowNewOldestTime();
}

gtsam::KeyVector FeaturePointNodeUpdater::OldKeys(const localization_common::Time oldest_allowed_time,
                                                  const gtsam::NonlinearFactorGraph& graph) const {
  return feature_point_graph_values_->OldKeys(oldest_allowed_time, graph);
}

boost::optional<gtsam::Key> FeaturePointNodeUpdater::GetKey(go::KeyCreatorFunction key_creator_function,
                                                            const localization_common::Time timestamp) const {
  return feature_point_graph_values_->GetKey(key_creator_function, timestamp);
}

boost::optional<localization_common::Time> FeaturePointNodeUpdater::OldestTimestamp() const {
  return feature_point_graph_values_->OldestTimestamp();
}

boost::optional<localization_common::Time> FeaturePointNodeUpdater::LatestTimestamp() const {
  return feature_point_graph_values_->LatestTimestamp();
}

int FeaturePointNodeUpdater::NumFeatures() const { return feature_point_graph_values_->NumFeatures(); }

std::shared_ptr<gv::FeaturePointGraphValues> FeaturePointNodeUpdater::shared_feature_point_graph_values() {
  return feature_point_graph_values_;
}

const gv::FeaturePointGraphValues& FeaturePointNodeUpdater::feature_point_graph_values() const {
  return *feature_point_graph_values_;
}
}  // namespace graph_vio
