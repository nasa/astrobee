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

#ifndef GRAPH_LOCALIZER_SEMANTIC_OBJECT_TRACK_H_
#define GRAPH_LOCALIZER_SEMANTIC_OBJECT_TRACK_H_

#include <localization_measurements/semantic_det.h>

#include <map>

namespace graph_localizer {
class SemanticObjectTrack {
  public:
    using Detections = std::map<localization_common::Time, localization_measurements::SemanticDet>;
    explicit SemanticObjectTrack(const localization_measurements::SemanticDet& semantic_det);
    bool empty() const;
    int size() const;

    void AddMeasurement(const localization_measurements::SemanticDet& semantic_det);
    void RemoveOldMeasurements(const localization_common::Time oldest_allowed_timestamp);

    float Distance(const localization_measurements::SemanticDet& semantic_det);
    boost::optional<localization_measurements::SemanticDet> LatestDet() const;
    std::vector<localization_measurements::SemanticDet> LatestDets() const;
    boost::optional<localization_common::Time> LatestTimestamp() const;
    boost::optional<localization_common::Time> OldestTimestamp() const;
  private:
    localization_measurements::ClassId cls_type_;
    Detections dets_;
};
} // namespace graph_localizer

#endif // GRAPH_LOCALIZER_SEMANTIC_OBJECT_TRACK_H_
