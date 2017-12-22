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

#ifndef EKF_BAG_TRACKED_FEATURES_H_
#define EKF_BAG_TRACKED_FEATURES_H_

#include <camera/camera_params.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/VisualLandmarks.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <vector>

namespace ekf_bag {

struct OFFeature {
  int id;
  float time;
  float x;
  float y;
};

struct SMFeature {
  Eigen::Vector3f pos;
  Eigen::Vector2f pixel;
  float time;
};

class TrackedOFFeatures {
 public:
  TrackedOFFeatures(void) :
                 params_(Eigen::Vector2i(-1, -1), Eigen::Vector2d::Constant(-1),
                 Eigen::Vector2d(-1, -1)) {}
  ~TrackedOFFeatures(void) {}

  void SetCameraParameters(const camera::CameraParameters & params) {params_ = params;}

  void UpdateFeatures(const ff_msgs::Feature2dArray & of, float time);
  std::map<int, struct OFFeature>::iterator begin() {return f_.begin();}
  std::map<int, struct OFFeature>::iterator end() {return f_.end();}

 private:
  std::map<int, struct OFFeature> f_;
  camera::CameraParameters params_;
};

class TrackedSMFeatures {
 public:
  TrackedSMFeatures(void) :
                 params_(Eigen::Vector2i(-1, -1), Eigen::Vector2d::Constant(-1),
                 Eigen::Vector2d(-1, -1)) {}
  ~TrackedSMFeatures(void) {}

  void SetCameraParameters(const camera::CameraParameters & params) {params_ = params;}
  void SetCameraToBody(const Eigen::Affine3d cam_to_body) {camera_to_body_ = cam_to_body;
                                            body_to_camera_ = cam_to_body.inverse();}

  void UpdateFeatures(const ff_msgs::VisualLandmarks & vl, float time);
  void UpdatePose(const geometry_msgs::Pose & pose);

  std::vector<struct SMFeature>::iterator begin() {return f_.begin();}
  std::vector<struct SMFeature>::iterator end() {return f_.end();}
  Eigen::Vector2d FeatureToCurrentPixel(const struct SMFeature & f);

 private:
  std::vector<struct SMFeature> f_;
  camera::CameraParameters params_;
  Eigen::Affine3d camera_to_body_, body_to_camera_, pose_transform_;
};

}  // end namespace ekf_bag

#endif  // EKF_BAG_TRACKED_FEATURES_H_

