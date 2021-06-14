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

#ifndef MARKER_TRACKING_MARKER_DETECTOR_H_
#define MARKER_TRACKING_MARKER_DETECTOR_H_

// Needed by ALVAR headers
#ifndef CV__ENABLE_C_API_CTORS
#define CV__ENABLE_C_API_CTORS
#endif
#include <opencv2/imgproc.hpp>

// A simplified version of the marker detector that doesn't calculate
// C_T_AR.
//
// This also saves us from exposure to all the ALVAR headers inside
// marker_tracking_node.
#include <marker_tracking/labelling_method.h>

#include <Eigen/Core>

#include <opencv2/core/core_c.h>  // I don't need all of OpenCV
#include <alvar/Marker.h>

#include <vector>

namespace alvar {
  class Camera;
}

namespace marker_tracking {
  class MarkerCornerDetector {
   protected:
    std::vector<alvar::MarkerData> *markers_, *markers_old_;
    marker_tracking::LabelingCvSeq labeling_;
    alvar::Camera* alvar_cam_;

   public:
    explicit MarkerCornerDetector(camera::CameraParameters const& camera);
    ~MarkerCornerDetector();

    void Detect(IplImage *image,
                float max_new_marker_error,
                float max_track_error);

    size_t NumMarkers() const;
    alvar::MarkerData& GetMarker(size_t i) const;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

}  // end namespace marker_tracking

#endif  // MARKER_TRACKING_MARKER_DETECTOR_H_
