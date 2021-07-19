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

#ifndef GRAPH_BAG_SPARSE_MAPPING_POSE_ADDER_H_
#define GRAPH_BAG_SPARSE_MAPPING_POSE_ADDER_H_

#include <gtsam/geometry/Pose3.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <string>

namespace graph_bag {
// Reads through a bag file and adds sparse mapping poses using ml/features/pose messages
// and body_T_nav_cam extrinsics
class SparseMappingPoseAdder {
 public:
  SparseMappingPoseAdder(const std::string& input_bag_name, const std::string& output_bag_name,
                         const gtsam::Pose3& nav_cam_T_body);
  void AddPoses();

 private:
  rosbag::Bag input_bag_;
  rosbag::Bag output_bag_;
  gtsam::Pose3 nav_cam_T_body_;
};
}  // end namespace graph_bag

#endif  // GRAPH_BAG_SPARSE_MAPPING_POSE_ADDER_H_
