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

#include <graph_localizer/semantic_loc_factor_adder.h>
#include <graph_localizer/loc_pose_factor.h>
#include <graph_localizer/loc_projection_factor.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <gtsam/base/Vector.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace lc = localization_common;
namespace sym = gtsam::symbol_shorthand;
SemanticLocFactorAdder::SemanticLocFactorAdder(const LocFactorAdderParams& params,
                                               const go::GraphActionCompleterType graph_action_completer_type)
    : LocFactorAdder(params, graph_action_completer_type) {
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

std::vector<go::FactorsToAdd> SemanticLocFactorAdder::AddFactors(
  const lm::SemanticDetsMeasurement& semantic_dets, const boost::optional<lc::CombinedNavState>& cur_state) {
  if (!cur_state) {
    return std::vector<go::FactorsToAdd>(); // return empty set, no factors
  }

  last_matches_.clear();

  // Convert state from gtsam to Eigen
  Eigen::Isometry3d world_T_body = lc::EigenPose(*cur_state);

  lm::MatchedProjectionsMeasurement matched_projections_measurement;
  matched_projections_measurement.timestamp = semantic_dets.timestamp;
  // Outer loop through objects so we only do transform/projection once
  LogError("SemanticLoc: adding sem loc factors");
  std::set<const lm::SemanticDet*> used_dets;
  for (const auto& classes : object_poses_) {
    int cls = classes.first;
    for (const auto& world_T_obj : classes.second) {
      Eigen::Vector2d cam_obj_px;
      try {
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(lc::GtPose(world_T_body).compose(params().body_T_cam), *params().cam_intrinsics);
        cam_obj_px = camera.project(world_T_obj.translation());
      } catch (gtsam::CheiralityException e) {
        // Point behind camera
        continue;
      }

      float best_dist = std::numeric_limits<float>::max();
      float second_best_dist = std::numeric_limits<float>::max();
      const lm::SemanticDet *best_det = nullptr;

      for (const auto& det : semantic_dets.semantic_dets) {
        // gtsam::Point2 is an Eigen::Vector2d typedef
        float dist = ((cam_obj_px - det.image_point).array()/det.bounding_box.array()).matrix().norm();
        if (dist < best_dist && cls == det.class_id && cls != 2 && used_dets.count(&det) == 0) {
          second_best_dist = best_dist;
          best_dist = dist;
          best_det = &det;
        }
      }

      // have second_best_dist requirement to avoid any ambiguity
      if (best_det && best_dist < 1 && second_best_dist > 1.5) {
        used_dets.insert(best_det);
        last_matches_.push_back(SemanticMatch(cls, cam_obj_px, best_det->image_point, best_det->bounding_box));

        LogError("SemanticLoc: adding matched proj measurement with dist: " << best_dist);
        lm::MatchedProjection mp(best_det->image_point, world_T_obj.translation(), semantic_dets.timestamp);
        matched_projections_measurement.matched_projections.push_back(mp);
      } else {
        last_matches_.push_back(SemanticMatch(cls, cam_obj_px));
      }
    }
  }

  // only for visualizing unmatched detections
  for (const auto& det : semantic_dets.semantic_dets) {
    if (used_dets.count(&det) == 0) {
      last_matches_.push_back(SemanticMatch(det.class_id, det.image_point, det.bounding_box));
    }
  }

  // Call superclass function from general LocFactorAdder
  return LocFactorAdder::AddFactors(matched_projections_measurement);
}

go::GraphActionCompleterType SemanticLocFactorAdder::type() const { return graph_action_completer_type_; }
}  // namespace graph_localizer
