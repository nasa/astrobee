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

#ifndef GRAPH_LOCALIZER_SEMANTIC_OBJECT_TRACKER_H_
#define GRAPH_LOCALIZER_SEMANTIC_OBJECT_TRACKER_H_

#include <graph_localizer/semantic_object_tracker_params.h>
#include <localization_common/time.h>
#include <localization_measurements/semantic_det.h>
#include <graph_localizer/semantic_object_track.h>

#include <set>

namespace graph_localizer {
class SemanticObjectTracker {
  public:
    explicit SemanticObjectTracker(const SemanticObjectTrackerParams& params = SemanticObjectTrackerParams());
    void UpdateObjectTracks(const localization_measurements::SemanticDets& semantic_dets);
    void RemoveOldObjectsAndSlideWindow(
      boost::optional<localization_common::Time> oldest_allowed_time = boost::none);
    std::vector<std::shared_ptr<const SemanticObjectTrack>> Tracks() const;
    void visualize();

  private:
    SemanticObjectTrackerParams params_;
    std::set<std::shared_ptr<SemanticObjectTrack>> tracks_;
};
} // namespace graph_localizer

#endif // GRAPH_LOCALZER_SEMANTIC_OBJECT_TRACKER_H_
