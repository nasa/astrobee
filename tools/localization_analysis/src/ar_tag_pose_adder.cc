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

#include <ff_common/ff_names.h>
#include <localization_analysis/ar_tag_pose_adder.h>
#include <localization_analysis/utilities.h>
#include <localization_common/utilities.h>

#include <ff_msgs/VisualLandmarks.h>

namespace localization_analysis {
namespace lc = localization_common;
namespace lm = localization_measurements;
ArTagPoseAdder::ArTagPoseAdder(const std::string& input_bag_name, const std::string& output_bag_name,
                                               const gtsam::Pose3& dock_cam_T_body)
    : input_bag_(input_bag_name, rosbag::bagmode::Read),
      output_bag_(output_bag_name, rosbag::bagmode::Write),
      dock_cam_T_body_(dock_cam_T_body) {}

void ArTagPoseAdder::AddPoses() {
  rosbag::View view(input_bag_);
  for (const rosbag::MessageInstance msg : view) {
    if (string_ends_with(msg.getTopic(), TOPIC_LOCALIZATION_AR_FEATURES)) {
      const ff_msgs::VisualLandmarksConstPtr vl_features = msg.instantiate<ff_msgs::VisualLandmarks>();
      if (vl_features->landmarks.size() >= 5) {
        const auto world_T_dock_cam = lc::PoseFromMsg(vl_features->pose);
        const auto world_T_body = world_T_dock_cam * dock_cam_T_body_;
        // TODO(rsoussan): put this in loc common
        const auto ar_tag_pose_msg =
          PoseMsg(world_T_body, lc::TimeFromHeader(vl_features->header));
        const ros::Time timestamp = lc::RosTimeFromHeader(vl_features->header);
        output_bag_.write("/" + std::string(TOPIC_AR_TAG_POSE), timestamp, ar_tag_pose_msg);
      }
    }
    output_bag_.write(msg.getTopic(), msg.getTime(), msg);
  }
}
}  // namespace localization_analysis
