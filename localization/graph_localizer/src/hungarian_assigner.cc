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

HungarianAssigner::AssignmentSet HungarianAssigner::tryAssignment(const Eigen::ArrayXXd& cost_matrix) {
  auto is_zero = (cost_matrix < 0.001).cast<int>();
  
}

// Hungarian algorithm implementation here
HungarianAssigner::AssignmentSet HungarianAssigner::solve(Eigen::ArrayXXd& cost_matrix) {
  if (cost_matrix.cols() == 0 || cost_matrix.rows() == 0) return {};

  // Trivial case
  if (cost_matrix.cols() == 1 || cost_matrix.rows() == 1) {
    Eigen::ArrayXXd::Index min_row, min_col;
    cost_matrix.minCoeff(&min_row, &min_col);
    return {Assignment(min_row, min_col)};
  }

  for (size_t row=0; row<cost_matrix.rows(); row++) {
    cost_matrix.row(row) -= cost_matrix.row(row).minCoeff();
  }

  for (size_t col=0; col<cost_matrix.cols(); col++) {
    cost_matrix.col(col) -= cost_matrix.col(col).minCoeff();
  }
}

void HungarianAssigner::assign(const Eigen::Isometry3d& world_T_body, const lm::SemanticDets &dets) {
  for (const auto& classes : object_poses_) {
    int cls = classes.first;
    // Build the cost matrix
    std::vector<Eigen::Vector2d> cam_objs_px;
    std::vector<const Eigen::Isometry3d*> associated_indices;
    for (const auto& world_T_obj : classes.second) {
      try {
        const auto world_T_cam = lc::GtPose(world_T_body).compose(body_T_cam_);
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(world_T_cam, cam_intrinsics_);
        cam_objs_px.push_back(camera.project(world_T_obj.translation()));
        associated_indices.push_back(&world_T_obj);
      } catch (gtsam::CheiralityException e) {
        // Point behind camera
        continue;
      }
    }

    Eigen::ArrayXXd det_locs_px(2,0);
    for (const auto& det : dets) {
      if (det.class_id == cls) {
        det_locs_px.conservativeResize(det_locs_px.rows(), det_locs_px.cols()+1);
        det_locs_px.col(det_locs_px.cols()-1) = det.image_point;
      }
    }

    Eigen::ArrayXXd cost_matrix(det_locs_px.cols(), cam_objs_px.size());
    size_t ind = 0;
    for (const auto& obj_loc_px : cam_objs_px) {
      auto diff = det_locs_px.colwise() - obj_loc_px.array();
      cost_matrix.col(ind++) = (diff.row(0).pow(2) + diff.row(1).pow(2)).sqrt();
    }

    //// Now solve the optimization problem with the built matrix
    solve(cost_matrix);    
  }
}
} // namespace graph_localizer
