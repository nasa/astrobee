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

HungarianAssigner::HungarianAssigner(const SemanticLocFactorAdderParams& params) :
    params_(params) {
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
      object_poses_[obj_cls] = std::vector<std::shared_ptr<Eigen::Isometry3d>>();
    }
    if (obj_cls == 2) continue; // ignore handrail

    config_reader::ConfigReader::Table pos(&object, "pos");
    config_reader::ConfigReader::Table rot(&object, "rot");

    Eigen::aligned_allocator<Eigen::Isometry3d> alloc;
    std::shared_ptr<Eigen::Isometry3d> pose = std::allocate_shared<Eigen::Isometry3d>(alloc);
    *pose = Eigen::Isometry3d::Identity();
    pos.GetReal(1, &(pose->translation().x()));
    pos.GetReal(2, &(pose->translation().y()));
    pos.GetReal(3, &(pose->translation().z()));
    Eigen::Quaterniond quat;
    rot.GetReal(1, &(quat.x())); 
    rot.GetReal(2, &(quat.y())); 
    rot.GetReal(3, &(quat.z())); 
    rot.GetReal(4, &(quat.w())); 
    pose->rotate(quat);

    // unneccessary copy here, but in init not a big deal
    object_poses_[obj_cls].push_back(pose);
  }
}

HungarianAssigner::AssignmentIndSet HungarianAssigner::tryAssignment(const Eigen::ArrayXXd& cost_matrix, Eigen::ArrayXXi &cell_state, int num_actual_rows) {
  cell_state = (cost_matrix > 0.001).cast<int>();
  LogDebug(cost_matrix);

  AssignmentIndSet assignment_set;

  int row_zero_count_min = 1;
  while (true) {
    Eigen::ArrayXi row_zero_count = (cell_state == 0).cast<int>().rowwise().sum();
    for (int row=0; row<cell_state.rows(); row++) {
      if (row_zero_count(row) == row_zero_count_min) {
        // make assignment
        Eigen::ArrayXi::Index col, ind;
        cell_state.row(row).minCoeff(&col);
        cell_state(row, col) = 2; // Mark as no longer 0, but assigned
        if (row < num_actual_rows) {
          assignment_set.push_back(AssignmentInd(row, col));
        }
        row_zero_count(row) = 0;
        while (cell_state.col(col).minCoeff(&ind) == 0) {
          cell_state(ind, col) = 3; // Mark as "scratched out" 0
          row_zero_count(ind)--;
        }
        for (int col=0; col<cell_state.cols(); col++) {
          if (cell_state(row, col) == 0) {
            cell_state(row, col) = 3;
          }
        }
        row_zero_count_min = 1; // We removed a column, reduce zero count
      }
    }
    if ((row_zero_count != row_zero_count_min).all()) {
      if (row_zero_count_min >= cell_state.cols()) {
        break;
      }
      row_zero_count_min++;
    }
  }

  LogDebug(cell_state);

  return assignment_set;
}

// Hungarian algorithm implementation here
HungarianAssigner::AssignmentIndSet HungarianAssigner::solve(const Eigen::ArrayXXd& cost_matrix_in) {
  // deep copy
  Eigen::ArrayXXd cost_matrix = cost_matrix_in;
  if (cost_matrix.cols() == 0 || cost_matrix.rows() == 0) return {};

  // Trivial case
  if (cost_matrix.cols() == 1 || cost_matrix.rows() == 1) {
    Eigen::ArrayXXd::Index min_row, min_col;
    cost_matrix.minCoeff(&min_row, &min_col);
    LogDebug("Trivial case");
    return {AssignmentInd(min_row, min_col)};
  }

  for (int row=0; row<cost_matrix.rows(); row++) {
    cost_matrix.row(row) -= cost_matrix.row(row).minCoeff();
  }

  size_t old_rows = cost_matrix.rows();
  if (cost_matrix.rows() < cost_matrix.cols()) {
    cost_matrix.conservativeResize(cost_matrix.cols(), cost_matrix.cols());
    cost_matrix.block(old_rows, 0, cost_matrix.cols()-old_rows, cost_matrix.cols()) = 0;
  }
  LogDebug("Subbed rows");

  Eigen::ArrayXXi cell_state;
  auto assignment = tryAssignment(cost_matrix, cell_state, old_rows);
  if (assignment.size() == old_rows) {
    return assignment;
  }

  for (int col=0; col<cost_matrix.cols(); col++) {
    cost_matrix.col(col) -= cost_matrix.col(col).minCoeff();
  }
  LogDebug("Subbed cols");

  while (true) {
    assignment = tryAssignment(cost_matrix, cell_state, old_rows);
    LogDebug("Assigned " << assignment.size());
    LogDebug(old_rows << " actual rows");
    if (assignment.size() == old_rows) {
      break;
    }
    std::set<int> marked_rows, marked_cols;

    // mark rows having no assignments
    for (int row=0; row<cell_state.rows(); row++) {
      if ((cell_state.row(row) != 2).all()) {
        marked_rows.insert(row);
      }
    }

    size_t marked_row_cnt = 0;
    size_t marked_col_cnt = 0;
    while (marked_rows.size() != marked_row_cnt || marked_cols.size() != marked_col_cnt) {
      marked_row_cnt = marked_rows.size();
      marked_col_cnt = marked_cols.size();
      // mark columns having zeros in newly marked rows
      for (const auto& marked_row : marked_rows) {
        for (int col=0; col<cell_state.cols(); col++) {
          if (cell_state(marked_row, col) != 1) {
            if (marked_cols.count(col) == 0) {
              marked_cols.insert(col);
            }
          }
        }
      }
      // mark rows having assignments in newly marked cols
      for (const auto& marked_col : marked_cols) {
        for (int row=0; row<cell_state.rows(); row++) {
          if (cell_state(row, marked_col) == 2) {
            if (marked_rows.count(row) == 0) {
              marked_rows.insert(row);
            }
          }
        }
      }
    }

    LogDebug("Marked rows: " << marked_row_cnt);
    LogDebug("Marked cols: " << marked_col_cnt);
    
    // find lowest value of unmarked cols, marked rows
    double lowest_val = std::numeric_limits<double>::max();
    for (int row=0; row<cell_state.rows(); row++) {
      if (marked_rows.count(row) != 0) {
        for (int col=0; col<cell_state.cols(); col++) {
          if (marked_cols.count(col) == 0 && cost_matrix(row, col) < lowest_val) {
            LogDebug("Found new lowest unmarked");
            lowest_val = cost_matrix(row, col);
          }
        }
      }
    }

    // subtract from all rows crossed
    for (int row=0; row<cell_state.rows(); row++) {
      if (marked_rows.count(row) != 0) {
        cost_matrix.row(row) -= lowest_val;
      }
    }

    // add to all cols crossed
    for (int col=0; col<cell_state.cols(); col++) {
      if (marked_cols.count(col) != 0) {
        cost_matrix.col(col) += lowest_val;
      }
    }

    LogDebug("One iteration completed");
  }

  return assignment;
}

HungarianAssigner::AssignmentSet HungarianAssigner::assign(const Eigen::Isometry3d& world_T_body, const lm::SemanticDets &dets) {
  AssignmentSet assignment_set;
  for (const auto& classes : object_poses_) {
    int cls = classes.first;
    // Build the cost matrix
    std::vector<Eigen::Vector2d> cam_objs_px;
    std::vector<std::shared_ptr<const Eigen::Isometry3d>> associated_objs;
    for (const auto& world_T_obj : classes.second) {
      try {
        const auto world_T_cam = lc::GtPose(world_T_body).compose(params_.body_T_cam);
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(world_T_cam, *params_.cam_intrinsics);
        cam_objs_px.push_back(camera.project(world_T_obj->translation()));
        associated_objs.push_back(world_T_obj);
      } catch (gtsam::CheiralityException e) {
        // Point behind camera
        continue;
      }
    }

    std::vector<size_t> associated_dets;
    size_t det_ind = 0;
    Eigen::ArrayXXd det_locs_px(4,0);
    for (const auto& det : dets) {
      if (det.class_id == cls) {
        det_locs_px.conservativeResize(det_locs_px.rows(), det_locs_px.cols()+1);
        det_locs_px.block<2,1>(0, det_locs_px.cols()-1) = det.image_point;
        det_locs_px.block<2,1>(2, det_locs_px.cols()-1) = det.bounding_box;
        associated_dets.push_back(det_ind);
      }
      det_ind++;
    }

    Eigen::ArrayXXd cost_matrix(det_locs_px.cols(), cam_objs_px.size());
    int ind = 0;
    for (const auto& obj_loc_px : cam_objs_px) {
      Eigen::ArrayXXd diff = det_locs_px.block(0,0,2,det_locs_px.cols()).colwise() - obj_loc_px.array();
      if (params_.scale_matching_distance_with_bbox) {
        diff /= det_locs_px.block(2,0,2,det_locs_px.cols())/2;
      }
      cost_matrix.col(ind++) = (diff.row(0).pow(2) + diff.row(1).pow(2)).sqrt();
      for (int row=0; row<cost_matrix.rows(); row++) {
        if (cost_matrix(row, ind) > params_.matching_distance_thresh) {
          cost_matrix(row, ind) = params_.matching_distance_thresh;
        }
      }
    }

    bool transposed = false;
    if (cost_matrix.rows() > cost_matrix.cols()) {
      transposed = true;
      cost_matrix.transposeInPlace();
    }

    // Now solve the optimization problem with the built matrix
    const auto raw_assign = solve(cost_matrix);
    for (const auto& a : raw_assign) {
      AssignmentInd assignment_ind = a;
      if (cost_matrix(a.first, a.second) >= params_.matching_distance_thresh - 0.0001) {
        continue;
      }
      if (transposed) {
        assignment_ind.first = a.second;
        assignment_ind.second = a.first;
      }
      if (assignment_ind.first < associated_dets.size() && assignment_ind.second < associated_objs.size()) {
        assignment_set.push_back(Assignment(associated_dets[assignment_ind.first], associated_objs[assignment_ind.second]));
      } else {
        LogError("Ignoring out of bounds match. This is bad, should never get here");
      }
    }
  }
  return assignment_set;
}
} // namespace graph_localizer
