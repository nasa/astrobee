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

#ifndef FF_UTIL_CONVERSION_H_
#define FF_UTIL_CONVERSION_H_

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace ff_util {

class Conversion {
 public:
  static void Convert(const geometry_msgs::msg::Point &in, tf2::Vector3 &out) {
    out = tf2::Vector3(in.x, in.y, in.z);
  }
  static void Convert(const geometry_msgs::msg::Vector3 &in, tf2::Vector3 &out) {
    out = tf2::Vector3(in.x, in.y, in.z);
  }
  static void Convert(const geometry_msgs::msg::Quaternion &in, tf2::Quaternion &out) {
    out = tf2::Quaternion(in.x, in.y, in.z, in.w);
  }
  static void Convert(const tf2::Vector3 &in, geometry_msgs::msg::Point &out) {
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
  }
  static void Convert(const tf2::Vector3 &in, geometry_msgs::msg::Vector3 &out) {
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
  }
  static void Convert(const tf2::Quaternion &in, geometry_msgs::msg::Quaternion &out) {
    out.w = in.w();
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
  }
};

}  // namespace ff_util

#endif  // FF_UTIL_CONVERSION_H_
