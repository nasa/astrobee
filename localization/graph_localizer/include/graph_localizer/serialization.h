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

#ifndef GRAPH_LOCALIZER_SERIALIZATION_H_
#define GRAPH_LOCALIZER_SERIALIZATION_H_

#include <graph_localizer/graph_localizer.h>

#include <string>

/*namespace boost {
namespace serialization {
template <class Archive>
void serialize(Archive& ar, graph_localizer::GraphLocalizer& graph_localizer, const unsigned int version) {
  split_free(ar, graph_localizer, version);
}

template <class Archive>
void save(Archive& ar, const graph_localizer::GraphLocalizer& graph_localizer, const unsigned int version) {
  // Workaround to removed combined imu factors before serializing.
  // This avoids a bug in gtsam for serializing combined imu factors.
  // Remove when new stable gtsam version is available.
  gtsam::NonlinearFactorGraph graph_copy = graph_localizer.graph_;
  for (auto factor_it = graph_copy.begin(); factor_it != graph_copy.end();) {
    if (dynamic_cast<gtsam::CombinedImuFactor*>(factor_it->get())) {
      factor_it = graph_copy.erase(factor_it);
      continue;
    }
    ++factor_it;
  }

  ar << graph_copy;
  ar << graph_localizer.graph_values_;
}

template <class Archive>
void load(Archive& ar, graph_localizer::GraphLocalizer& graph_localizer, const unsigned int version) {
  ar >> graph_localizer.graph_;
  ar >> graph_localizer.graph_values_;
}
}  // namespace serialization
}  // namespace boost */

namespace graph_localizer {

std::string SerializeBinary(const GraphLocalizer& graph_localizer);
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_SERIALIZATION_H_
