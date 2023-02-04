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

#include <ff_common/init.h>
#include <ff_common/utils.h>
#include <ff_common/ff_ros.h>

#include <gflags/gflags.h>

#if ROS1
#include <ff_msgs/VisualLandmarks.h>
#include <sensor_msgs/PointCloud2.h>

#else
#include <ff_msgs/msg/visual_landmarks.hpp>
namespace ff_msgs {
typedef msg::VisualLandmarks VisualLandmarks;
typedef msg::VisualLandmarks::SharedPtr VisualLandmarksPtr;
}  // namespace ff_msgs
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace sensor_msgs {
typedef msg::PointCloud2 PointCloud2;
}
namespace std_msgs {
typedef msg::Header Header;
}

#endif

DEFINE_string(input_topic, "/loc/ml/features",
              "The input features topic to convert.");
DEFINE_string(output_topic, "/loc/rviz",
              "The output topic suitable for rviz.");

Publisher<sensor_msgs::PointCloud2> landmarks_pub = NULL;
NodeHandle node;

void landmarks_callback(ff_msgs::VisualLandmarksPtr const l) {
  sensor_msgs::PointCloud2 out;
  out.header = std_msgs::Header();
  out.header.stamp = node->get_clock()->now();
  out.header.frame_id = "world";
  out.height = 1;
  out.width = l->landmarks.size();

  out.fields.resize(3);
  out.fields[0].name = "x";
  out.fields[0].offset = 0;
  out.fields[0].datatype = 7;
  out.fields[0].count = 1;
  out.fields[1].name = "y";
  out.fields[1].offset = 4;
  out.fields[1].datatype = 7;
  out.fields[1].count = 1;
  out.fields[2].name = "z";
  out.fields[2].offset = 8;
  out.fields[2].datatype = 7;
  out.fields[2].count = 1;

  out.is_bigendian = false;
  out.point_step = 12;
  out.row_step = out.point_step * out.width;
  out.is_dense = true;
  out.data.resize(out.row_step);

  for (unsigned int i = 0; i < l->landmarks.size(); i++) {
    memcpy(&out.data[out.point_step * i + 0], &l->landmarks[i].x, 4);
    memcpy(&out.data[out.point_step * i + 4], &l->landmarks[i].y, 4);
    memcpy(&out.data[out.point_step * i + 8], &l->landmarks[i].z, 4);
  }

  landmarks_pub->publish(out);
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  rclcpp::init(argc, argv);   \
  node = rclcpp::Node::make_shared("landmark_msg_cnv");

  auto landmarks_sub = node->create_subscription<ff_msgs::VisualLandmarks>(FLAGS_input_topic, 5, landmarks_callback);

  landmarks_pub = node->create_publisher<sensor_msgs::PointCloud2>(FLAGS_output_topic, 5);

  rclcpp::spin(node);

  return 0;
}
