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

#include <marker_tracking/marker_detector.h>
#include <marker_tracking/arxmlio.h>

// ALVAR stuff
#include <alvar/Marker.h>
#include <alvar/Camera.h>
#include <alvar/ConnectedComponents.h>
#include <glog/logging.h>

#include <vector>

namespace marker_tracking {

MarkerCornerDetector::MarkerCornerDetector(camera::CameraParameters const& camera) :
  labeling_(camera) {
  markers_ = new std::vector<alvar::MarkerData>();
  markers_old_ = new std::vector<alvar::MarkerData>();
  alvar_cam_ = new alvar::Camera();
  alvar_cam_->SetSimpleCalib(camera.GetUndistortedSize()[0],
                             camera.GetUndistortedSize()[1], 1);
}

MarkerCornerDetector::~MarkerCornerDetector() {
  if (markers_) delete markers_;
  if (markers_old_) delete markers_old_;
  if (alvar_cam_) delete alvar_cam_;
}

void MarkerCornerDetector::Detect(IplImage *image,
                                  float max_new_marker_error,
                                  float max_track_error) {
  assert(image->origin == 0);
  std::swap(markers_, markers_old_);
  markers_->clear();

  labeling_.LabelSquares(image);
  std::vector<std::vector<alvar::PointDouble> >& blob_corners = labeling_.blob_corners;

  int orientation;
  double error;

  // Identify markers that are close to ones we already found, so we
  // can save ourselves time
  for (size_t m = 0; m < markers_old_->size(); m++) {
    alvar::MarkerData* marker = &markers_old_->operator[](m);

    // Was it decoded before? If not, skip it.
    if (marker->GetError(alvar::Marker::DECODE_ERROR | alvar::Marker::MARGIN_ERROR) > 0) continue;

    double track_error = std::numeric_limits<double>::max();
    size_t track_b = std::numeric_limits<size_t>::max();
    int track_orientation = 0;

    // Compare against all found corners
    for (size_t b = 0; b < blob_corners.size(); b++) {
      if (blob_corners[b].empty()) continue;

      marker->CompareCorners(blob_corners[b], &orientation, &error);
      if (error < track_error) {
        track_b = b;
        track_orientation = orientation;
        track_error = error;
      }
    }

    // Did we actually find something, then put it in the final vector
    if (track_error <= max_track_error) {
      marker->SetError(alvar::Marker::DECODE_ERROR, 0);
      marker->SetError(alvar::Marker::MARGIN_ERROR, 0);
      marker->SetError(alvar::Marker::TRACK_ERROR, track_error);
      marker->UpdatePose(blob_corners[track_b], NULL, track_orientation, 0 /*frame num*/, false/*update pose*/);
      markers_->push_back(*marker);  // copy I guess
      blob_corners[track_b].clear();
    }
  }

  // Identify the new markers that we haven't seen before.
  for (size_t i = 0; i < blob_corners.size(); i++) {
    if (blob_corners[i].empty()) continue;

    alvar::MarkerData marker(1 /*edge length*/, 5 /*resolution*/, 2 /*margin*/);
    if (marker.UpdateContent(blob_corners[i], image, alvar_cam_) &&
        marker.DecodeContent(&orientation) &&
        (marker.GetError(alvar::Marker::MARGIN_ERROR |
                         alvar::Marker::DECODE_ERROR) <= max_new_marker_error)) {
      marker.UpdatePose(blob_corners[i], NULL, orientation, 0 /*frame num*/, false /*update pose*/);

      // Verify that we haven't seen this ID before.
      bool bad = false;
      for (size_t j = 0; j < markers_->size(); j++) {
        if (markers_->operator[](j).GetId() == marker.GetId()) {
          bad = true;
          break;
        }
      }
      if (bad) continue;

      markers_->push_back(marker);
    }
  }
}

size_t MarkerCornerDetector::NumMarkers() const {
  return markers_->size();
}

alvar::MarkerData& MarkerCornerDetector::GetMarker(size_t i) const {
  return markers_->at(i);
}

}  // end namespace marker_tracking
