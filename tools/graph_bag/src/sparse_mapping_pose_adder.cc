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

#include <ff_util/ff_names.h>
#include <graph_bag/sparse_mapping_pose_adder.h>
#include <graph_bag/utilities.h>
#include <graph_localizer/utilities.h>
#include <localization_common/utilities.h>

#include <ff_msgs/VisualLandmarks.h>

namespace graph_bag {
namespace lc = localization_common;
namespace lm = localization_measurements;
SparseMappingPoseAdder::SparseMappingPoseAdder(const std::string& input_bag_name, const std::string& output_bag_name,
                                               const gtsam::Pose3& nav_cam_T_body)
    : input_bag_(input_bag_name, rosbag::bagmode::Read),
      output_bag_(output_bag_name, rosbag::bagmode::Write),
      nav_cam_T_body_(nav_cam_T_body) {}

void SparseMappingPoseAdder::AddPoses() {
  rosbag::View view(input_bag_);
  for (const rosbag::MessageInstance msg : view) {
    if (string_ends_with(msg.getTopic(), TOPIC_LOCALIZATION_ML_FEATURES)) {
      const ff_msgs::VisualLandmarksConstPtr vl_features = msg.instantiate<ff_msgs::VisualLandmarks>();
      if (vl_features->landmarks.size() >= 5) {
        const auto world_T_nav_cam = lc::GtPose(*vl_features);
        const auto world_T_body = world_T_nav_cam * nav_cam_T_body_;
        // TODO(rsoussan): put this in loc common
        const auto sparse_mapping_pose_msg =
          graph_localizer::PoseMsg(world_T_body, lc::TimeFromHeader(vl_features->header));
        const ros::Time timestamp = lc::RosTimeFromHeader(vl_features->header);
        output_bag_.write("/" + std::string(TOPIC_SPARSE_MAPPING_POSE), timestamp, sparse_mapping_pose_msg);
      }
    }
    output_bag_.write(msg.getTopic(), msg.getTime(), msg);
  }
}
}  // namespace graph_bag
