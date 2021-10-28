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

#include <marker_tracking/arconfigio.h>

#include <config_reader/config_reader.h>

#include <glog/logging.h>
#include <iostream>
#include <list>

namespace marker_tracking {

static const int BL = 0;
static const int BR = 1;
static const int TR = 2;
static const int TL = 3;

void AddCorners(ARTagMap *ar_corner_world_location, int id,
                Eigen::Matrix<float, 4, 3> &corners) {
  // Check that we have something that is close to 90 degrees
  Eigen::Vector3f ua = corners.row(TR) - corners.row(TL),
                  ub = corners.row(BL) - corners.row(TL);
  float angle = acos(ua.dot(ub) / (ua.norm() * ub.norm()));
  static const float SMALL_ANGLE = 1.0 * M_PI / 180.0;
  if (fabs(fabs(angle) - M_PI / 2) > SMALL_ANGLE) {
    LOG(INFO) << "Corner measurements:\n" << corners;
    LOG(INFO) << "top edge length:"
              << (corners.row(TR) - corners.row(TL)).norm();
    LOG(INFO) << "left edge length:"
              << (corners.row(TL) - corners.row(BL)).norm();
    LOG(ERROR) << "AR " << id
               << "'s measurements are not square. What should be a right "
                  "angle, measures "
               << angle * 180 / M_PI << " deg";
  }
  // Make fourth measurement
  corners.row(BR) = corners.row(TR) + ub.transpose();
  ar_corner_world_location->insert(std::make_pair(id, corners));
}

Eigen::Vector3f ParseToVector(config_reader::ConfigReader::Table *marker_specs,
                              const char *name) {
  float corner_pos[3];
  config_reader::ConfigReader::Table marker_corner(marker_specs, name);
  marker_corner.GetReal(1, &corner_pos[0]);
  marker_corner.GetReal(2, &corner_pos[1]);
  marker_corner.GetReal(3, &corner_pos[2]);
  return Eigen::Vector3f(corner_pos[0], corner_pos[1], corner_pos[2]);
}

void LoadARTagsConfig(config_reader::ConfigReader *config,
                      ARTagMap *ar_corner_world_location) {
  if (!config->ReadFiles()) {
    LOG(FATAL) << "Failed to read config file!" << std::endl;
  }
  config_reader::ConfigReader::Table markers_table(config, "markers_world");
  for (int i = 0; i < markers_table.GetSize(); i++) {
    config_reader::ConfigReader::Table marker_specs(&markers_table, (i + 1));
    int marker_id;
    marker_specs.GetInt("id", &marker_id);
    Eigen::Matrix<float, 4, 3> corners;  // BL, BR, TR, TL
    corners.setZero();
    corners.row(TL) = ParseToVector(&marker_specs, "top_left");
    corners.row(TR) = ParseToVector(&marker_specs, "top_right");
    corners.row(BL) = ParseToVector(&marker_specs, "bottom_left");
    AddCorners(ar_corner_world_location, marker_id, corners);
  }
}

}  // end namespace marker_tracking
