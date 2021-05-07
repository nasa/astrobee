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

#include <graph_optimizer/feature_point_graph_values.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <iomanip>

namespace graph_optimizer {
namespace lc = localization_common;
namespace lm = localization_measurements;
FeaturePointGraphValues::FeaturePointGraphValues(std::shared_ptr<gtsam::Values> values)
    : GraphValues(std::move(values)), feature_key_index_(0) {}

boost::optional<gtsam::Key> FeaturePointGraphValues::GetKey(KeyCreatorFunction key_creator_function,
                                                            const localization_common::Time timestamp) const {
  return boost::none;
}

boost::optional<lc::Time> FeaturePointGraphValues::OldestTimestamp() const { return boost::none; }

boost::optional<lc::Time> FeaturePointGraphValues::LatestTimestamp() const { return boost::none; }

boost::optional<lc::Time> FeaturePointGraphValues::SlideWindowNewOldestTime() const { return boost::none; }

bool FeaturePointGraphValues::HasFeature(const lm::FeatureId id) const { return (feature_id_key_map_.count(id) > 0); }

boost::optional<gtsam::Key> FeaturePointGraphValues::FeatureKey(const lm::FeatureId id) const {
  if (!HasFeature(id)) return boost::none;
  return feature_id_key_map_.at(id);
}

gtsam::Key FeaturePointGraphValues::CreateFeatureKey() const { return sym::F(++feature_key_index_); }

gtsam::KeyVector FeaturePointGraphValues::FeatureKeys() const {
  gtsam::KeyVector feature_keys;
  for (const auto& feature_id_key_pair : feature_id_key_map_) {
    feature_keys.emplace_back(feature_id_key_pair.second);
  }
  return feature_keys;
}

bool FeaturePointGraphValues::AddFeature(const lm::FeatureId id, const gtsam::Point3& feature_point,
                                         const gtsam::Key& key) {
  if (HasFeature(id)) {
    LogError("AddFeature: Feature already exists.");
    return false;
  }

  if (values().exists(key)) {
    LogError("AddFeature: Key already exists in values.");
  }

  feature_id_key_map_.emplace(id, key);
  values().insert(key, feature_point);
  return true;
}

int FeaturePointGraphValues::NumFeatures() const { return feature_id_key_map_.size(); }

gtsam::KeyVector FeaturePointGraphValues::OldKeys(const localization_common::Time oldest_allowed_time,
                                                  const gtsam::NonlinearFactorGraph& factors) const {
  using ProjectionFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>;
  gtsam::KeyVector old_features;
  for (const auto& feature_id_key_pair : feature_id_key_map_) {
    const auto& key = feature_id_key_pair.second;
    int num_factors = 0;
    for (const auto& factor : factors) {
      // Only consider projection factors for min num feature factors
      const auto projection_factor = dynamic_cast<const ProjectionFactor*>(factor.get());
      if (!projection_factor) continue;
      if (factor->find(key) != factor->end()) {
        ++num_factors;
        // if (num_factors >= params_.min_num_factors_per_feature) break;
      }
    }

    // if (num_factors < params_.min_num_factors_per_feature) {
    if (num_factors <= 0) {
      old_features.emplace_back(key);
    }
  }
  return old_features;
}

void FeaturePointGraphValues::RemoveOldFeatures(const gtsam::KeyVector& old_keys) {
  for (const auto& key : old_keys) {
    // TODO(rsoussan): test this
    if (gtsam::Symbol(key).chr() != 'F') continue;
    values().erase(key);
    for (auto feature_id_key_it = feature_id_key_map_.begin(); feature_id_key_it != feature_id_key_map_.end();) {
      if (feature_id_key_it->second == key) {
        feature_id_key_it = feature_id_key_map_.erase(feature_id_key_it);
      } else {
        ++feature_id_key_it;
      }
    }
  }
}

}  // namespace graph_optimizer
