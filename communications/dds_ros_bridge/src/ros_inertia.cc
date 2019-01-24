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

#include "dds_ros_bridge/ros_inertia.h"

ff::RosInertiaToRapid::RosInertiaToRapid(const std::string& subscribe_topic,
                                         const std::string& pub_topic,
                                         const ros::NodeHandle &nh,
                                         const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  data_supplier_.reset(new ff::RosInertiaToRapid::DataSupplier(
      rapid::ext::astrobee::INERTIAL_PROPERTIES_TOPIC + pub_topic, "",
      "AstrobeeInertiaProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosInertiaToRapid::InertiaCallback,
                       this);

  rapid::RapidHelper::initHeader(data_supplier_->event().hdr);
}

void ff::RosInertiaToRapid::InertiaCallback(
                        geometry_msgs::InertiaStampedConstPtr const& inertia) {
  rapid::ext::astrobee::InertialProperties &data_msg = data_supplier_->event();

  data_msg.hdr.timeStamp = util::RosTime2RapidTime(ros::Time::now());

  // Frame id holds the name of the inertia profile
  std::strncpy(data_msg.name, inertia->header.frame_id.data(), 32);
  data_msg.name[31] = '\0';

  // Mass
  data_msg.mass = inertia->inertia.m;

  // Center of Mass
  data_msg.centerOfMass[0] = inertia->inertia.com.x;
  data_msg.centerOfMass[1] = inertia->inertia.com.y;
  data_msg.centerOfMass[2] = inertia->inertia.com.x;

  // Inertia Tensor
  //     | ixx ixy ixz |
  // I = | ixy iyy iyz |
  //     | ixz iyz izz |
  data_msg.matrix[0] = inertia->inertia.ixx;
  data_msg.matrix[1] = inertia->inertia.ixy;
  data_msg.matrix[2] = inertia->inertia.ixz;
  data_msg.matrix[3] = inertia->inertia.ixy;
  data_msg.matrix[4] = inertia->inertia.iyy;
  data_msg.matrix[5] = inertia->inertia.iyz;
  data_msg.matrix[6] = inertia->inertia.ixz;
  data_msg.matrix[7] = inertia->inertia.iyz;
  data_msg.matrix[8] = inertia->inertia.izz;

  data_supplier_->sendEvent();
}
