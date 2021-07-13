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

#include <graph_localizer/semantic_object_tracker.h>
#include <localization_common/logger.h>

#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc.hpp> 

namespace graph_localizer {
namespace lc = localization_common;
namespace lm = localization_measurements;
SemanticObjectTracker::SemanticObjectTracker(const SemanticObjectTrackerParams& params) : params_(params) {}

void SemanticObjectTracker::UpdateObjectTracks(const lm::SemanticDets& semantic_dets) {
  std::set<std::shared_ptr<SemanticObjectTrack>> used_tracks;
  for (const auto& det : semantic_dets) {
    float min_dist = std::numeric_limits<float>::max();
    std::shared_ptr<SemanticObjectTrack> best_track;
    for (const auto& track : tracks_) {
      const float dist = track->Distance(det);
      // TODO (iandm) Likely need better association scheme than this
      if (dist >= 0 && dist < params_.min_det_dist_thresh &&
          dist < min_dist && used_tracks.count(track) == 0) {
        min_dist = dist;
        best_track = track;
      }
    }

    if (best_track) {
      used_tracks.insert(best_track);
      best_track->AddMeasurement(det);
    } else {
      //No good track, so create new
      const auto new_track = std::make_shared<SemanticObjectTrack>(det);
      tracks_.insert(new_track);
    }
  }

  LogDebug("SemanticObjectTracker: Updated tracks. Now " << tracks_.size() << " tracks");
}

std::vector<std::shared_ptr<const SemanticObjectTrack>> SemanticObjectTracker::Tracks() const {
  std::vector<std::shared_ptr<const SemanticObjectTrack>> track_vec;
  for (const auto& track : tracks_) {
    track_vec.push_back(track);
  }
  return track_vec;
}

void SemanticObjectTracker::visualize() {
  cv::Mat viz(960, 1280, CV_8UC1, cv::Scalar(0));
  for (const auto& track : tracks_) {
    const auto latest_dets = track->LatestDets();
    std::vector<cv::Point> line;
    for (const auto& det : latest_dets) {
      line.push_back(cv::Point(static_cast<int>(det.image_point.x()), 
                               static_cast<int>(det.image_point.y())));
      //LogDebug("SemanticObjectTracker: Image point: " << det.image_point);
    }
    cv::polylines(viz, line, false, cv::Scalar(255));
  } 
  cv::imshow("feature tracks", viz);
  cv::waitKey(2);
}

void SemanticObjectTracker::RemoveOldObjectsAndSlideWindow(boost::optional<lc::Time> oldest_allowed_time) {
  auto it = tracks_.begin();
  while (it != tracks_.end()) {
    if ((*it)->LatestTimestamp() < oldest_allowed_time) {
      //Retire objects that haven't been detected for a while
      it = tracks_.erase(it);
    } else {
      it++;
    }
  }

  //Remove old points from current tracks
  for (const auto& track : tracks_) {
    track->RemoveOldMeasurements(*oldest_allowed_time);
  }
}
} // namespace graph_localizer
