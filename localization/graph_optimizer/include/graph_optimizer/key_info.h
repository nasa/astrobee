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

#ifndef GRAPH_OPTIMIZER_KEY_INFO_H_
#define GRAPH_OPTIMIZER_KEY_INFO_H_

#include <graph_localizer/node_updater_type.h>
#include <localization_common/time.h>

#include <gtsam/inference/Key.h>

#include <functional>
#include <vector>

namespace graph_localizer {

using KeyCreatorFunction = std::function<gtsam::Key(std::uint64_t)>;
class KeyInfo {
 public:
  KeyInfo(KeyCreatorFunction key_creator_function, const NodeUpdaterType node_updater_type,
          const localization_common::Time timestamp)
      : key_creator_function_(key_creator_function),
        node_updater_type_(node_updater_type),
        timestamp_(timestamp),
        static_(false) {}
  // Use for static keys
  // TODO(rsoussan): Clean up this interface, use inheritance instead?
  KeyInfo(KeyCreatorFunction key_creator_function, const NodeUpdaterType node_updater_type, const int id)
      : key_creator_function_(key_creator_function),
        node_updater_type_(node_updater_type),
        timestamp_(0),
        id_(id),
        static_(true) {}
  gtsam::Key UninitializedKey() const { return key_creator_function_(0); }
  gtsam::Key MakeKey(const std::uint64_t key_index) const { return key_creator_function_(key_index); }
  static gtsam::Key UninitializedKey(KeyCreatorFunction key_creator_function) { return key_creator_function(0); }
  KeyCreatorFunction key_creator_function() const { return key_creator_function_; }
  localization_common::Time timestamp() const { return timestamp_; }
  bool is_static() const { return static_; }
  // TODO(rsoussan): This is only available for static keys, clean this up
  int id() const { return id_; }
  NodeUpdaterType node_updater_type() const { return node_updater_type_; }

 private:
  KeyCreatorFunction key_creator_function_;
  NodeUpdaterType node_updater_type_;
  localization_common::Time timestamp_;
  int id_;
  // Static (not changing with time) key
  bool static_;
};

using KeyInfos = std::vector<KeyInfo>;
}  // namespace graph_localizer

#endif  // GRAPH_OPTIMIZER_KEY_INFO_H_
