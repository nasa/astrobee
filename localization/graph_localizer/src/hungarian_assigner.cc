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

#include <graph_localizer/hungarian_assigner.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <localization_common/logger.h>

namespace graph_localizer {
namespace lm = localization_measurements;
namespace lc = localization_common;

HungarianAssigner::HungarianAssigner(const gtsam::Pose3& body_T_cam, const gtsam::Cal3_S2& cam_intrinsics) :
    body_T_cam_(body_T_cam), cam_intrinsics_(cam_intrinsics) {
  config_reader::ConfigReader object_loc_config;
  object_loc_config.AddFile("semantic_objects.config");
  if (!object_loc_config.ReadFiles()) {
    LogError("Failed to load semantic object locations.");
  }

  // note config reader is indexed from 1 because lua
  config_reader::ConfigReader::Table objects(&object_loc_config, "objects");
  int N = objects.GetSize();
  int obj_cls;
  for (int i=1; i<=N; i++) {
    config_reader::ConfigReader::Table object(&objects, i);
    object.GetInt("class", &obj_cls);
    if (object_poses_.count(obj_cls) == 0) {
      object_poses_[obj_cls] = std::vector<Eigen::Isometry3d>();
    }

    config_reader::ConfigReader::Table pos(&object, "pos");
    config_reader::ConfigReader::Table rot(&object, "rot");

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pos.GetReal(1, &(pose.translation().x()));
    pos.GetReal(2, &(pose.translation().y()));
    pos.GetReal(3, &(pose.translation().z()));
    Eigen::Quaterniond quat;
    rot.GetReal(1, &(quat.x())); 
    rot.GetReal(2, &(quat.y())); 
    rot.GetReal(3, &(quat.z())); 
    rot.GetReal(4, &(quat.w())); 
    pose.rotate(quat);

    // unneccessary copy here, but in init not a big deal
    object_poses_[obj_cls].push_back(pose);
  }
}

void HungarianAssigner::assign(const Eigen::Isometry3d& world_T_body, const lm::SemanticDets &dets) {
  for (const auto& classes : object_poses_) {
    int cls = classes.first;
    std::vector<Eigen::Vector2d> cam_objs_px;
    for (const auto& world_T_obj : classes.second) {
      try {
        const auto world_T_cam = lc::GtPose(world_T_body).compose(body_T_cam_);
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(world_T_cam, cam_intrinsics_);
        cam_objs_px.push_back(camera.project(world_T_obj.translation()));
      } catch (gtsam::CheiralityException e) {
        // Point behind camera
        continue;
      }
    }
  }
}
} // namespace graph_localizer
