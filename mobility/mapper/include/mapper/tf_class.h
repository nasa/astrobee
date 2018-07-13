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

#ifndef MAPPER_TF_CLASS_H_
#define MAPPER_TF_CLASS_H_

#include <tf/transform_listener.h>
#include <string>

namespace tf_listener {

class TfClass{
 public:
  tf::TransformListener listener_;
  tf::StampedTransform transform_;

  // Constructor
  TfClass() {}

  // Print everything within transform (for debugging)
  void PrintTransform() {
    tf::Quaternion q = transform_.getRotation();
    tf::Vector3 v = transform_.getOrigin();
    double yaw, pitch, roll;
    transform_.getBasis().getRPY(roll, pitch, yaw);
    std::cout << "- Translation: [" << v.getX() << ", "
                    << v.getY() << ", "
                    << v.getZ() << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion ["
          << q.getX() << ", "
          << q.getY() << ", "
          << q.getZ() << ", "
          << q.getW() << "]" << std::endl
          << "            in RPY (radian) ["
          <<  roll << ", "
          << pitch << ", "
          << yaw << "]" << std::endl
          << "            in RPY (degree) ["
          <<  roll*180.0/M_PI << ", "
          << pitch*180.0/M_PI << ", "
          << yaw*180.0/M_PI << "]" << std::endl;
  }

  // Get transform from original to target frame
  bool GetTransform(const std::string &original_frame,
            const std::string &target_frame) {
    try {
      // First we wait until transform is published, then we look it up
      if (listener_.waitForTransform(target_frame, original_frame,
                     ros::Time(0), ros::Duration(5))) {
        listener_.lookupTransform(target_frame, original_frame,
                     ros::Time(0), transform_);
        } else {
          ROS_DEBUG("Transform from %s to %s not being published!",
               original_frame.c_str(),
               target_frame.c_str());
          return false;
        }
      }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
      }

    return true;
  }
};

}  // namespace tf_listener

#endif  // MAPPER_TF_CLASS_H_
