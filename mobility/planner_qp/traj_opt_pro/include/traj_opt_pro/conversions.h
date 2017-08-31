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

#ifndef TRAJ_OPT_PRO_CONVERSIONS_H_
#define TRAJ_OPT_PRO_CONVERSIONS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <traj_opt_basic/types.h>

namespace traj_opt {
class Conversions {
 public:
  //  Conversions() {}
  static Vec4 PointToVec(const geometry_msgs::Point &p) {
    Vec4 v;
    v << p.x, p.y, p.z, 0.0;
    return v;
  }
  static geometry_msgs::Point VecToPoint(const Vec4 &v) {
    geometry_msgs::Point p;
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
    return p;
  }
  static Vec4 Vector3ToVec(const geometry_msgs::Vector3 &p) {
    Vec4 v;
    v << p.x, p.y, p.z, 0.0;
    return v;
  }
  static geometry_msgs::Vector3 VecToVector3(const Vec4 &v) {
    geometry_msgs::Vector3 p;
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
    return p;
  }
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_PRO_CONVERSIONS_H_
