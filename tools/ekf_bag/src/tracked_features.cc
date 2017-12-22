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

#include <ekf_bag/tracked_features.h>

#include <camera/camera_model.h>

#include <Eigen/Core>

namespace ekf_bag {

void TrackedOFFeatures::UpdateFeatures(const ff_msgs::Feature2dArray & of, float time) {
  std::map<int, ff_msgs::Feature2d > cur_features;
  for (unsigned int i = 0; i < of.feature_array.size(); i++) {
    cur_features.insert(std::pair<int, ff_msgs::Feature2d >(of.feature_array[i].id, of.feature_array[i]));
  }

  for (std::map<int, struct OFFeature>::iterator it = f_.begin(); it != f_.end(); it++) {
    int count = cur_features.count(it->first);
    if (count == 0) {
      it = f_.erase(it);
      it--;  // erase returns the next element, so we would skip it
    }
  }
  for (std::map<int, ff_msgs::Feature2d >::const_iterator it = cur_features.begin(); it != cur_features.end(); it++) {
    Eigen::Vector2d input(it->second.x, it->second.y);
    Eigen::Vector2d output;
    params_.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>(input, &output);
    struct OFFeature f = {it->second.id, time, static_cast<float>(output.x()), static_cast<float>(output.y())};
    auto t = f_.emplace(f.id, f);
    // if is already present
    t.first->second.x = output.x();
    t.first->second.y = output.y();
  }
}

void TrackedSMFeatures::UpdateFeatures(const ff_msgs::VisualLandmarks & vl, float time) {
  f_.clear();

  for (unsigned int i = 0; i < vl.landmarks.size(); i++) {
    auto & l = vl.landmarks[i];
    struct SMFeature f;
    f.pos.x() = l.x; f.pos.y() = l.y; f.pos.z() = l.z;
    Eigen::Vector2d input(l.u, l.v), output;
    params_.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>(input, &output);
    f.pixel.x() = output.x(); f.pixel.y() = output.y();
    f.time = time;
    f_.push_back(f);
  }
}

void TrackedSMFeatures::UpdatePose(const geometry_msgs::Pose & pose) {
  pose_transform_ = Eigen::Affine3d(
           Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z)) *
           Eigen::Affine3d(Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                              pose.orientation.y, pose.orientation.z));
}

Eigen::Vector2d TrackedSMFeatures::FeatureToCurrentPixel(const struct SMFeature & f) {
  Eigen::Vector3d rel = f.pos.cast<double>();
  camera::CameraModel c(body_to_camera_ * pose_transform_.inverse(), params_);
  Eigen::Vector2d output;
  params_.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>(c.ImageCoordinates(rel), &output);
  return output;
}

}  // namespace ekf_bag

