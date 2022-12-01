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

/**
 * This header makes ROS1 to ROS2 migration macros
 * It should be included in any file that uses ROS.
 **/

#ifndef FF_COMMON_ROS_H_
#define FF_COMMON_ROS_H_

#ifdef ROS1
#include <ros/ros.h>
#define ROS_CREATE_NODE(name)  \
  ros::init(argc, argv, name); \
  ros::NodeHandle nh;          \
  ros::NodeHandle private_nh("~");

using Publisher = ros::Publisher*;

#define ROS_CREATE_PUBLISHER(msg, topic, queue)             private_nh.advertise<msg>(topic, queue)
#define ROS_CREATE_SUBSCRIBER(msg, topic, queue, callback)  private_nh.subscribe(topic, queue, callback)

#define ROS_TIME_NOW()  ros::Time::now()
#define ROS_SPIN()      ros::spin()
#define ROS_SPIN_ONCE() ros::spinOnce()
#define ROS_OK()        ros::ok()
#define ROS_SHUTDOWN()  ros::shutdown()

#else
#include "rclcpp/rclcpp.hpp"
namespace ros = rclcpp;

#define ROS_CREATE_NODE(name) \
  rclcpp::init(argc, argv);   \
  auto node = rclcpp::Node::make_shared(name, "/" name);

template<class MessageType>
using Publisher = std::shared_ptr<ros::Publisher<MessageType>>;

#define ROS_CREATE_PUBLISHER(msg, topic, queue)             node->create_publisher<msg>(topic, queue)
#define ROS_CREATE_SUBSCRIBER(msg, topic, queue, callback)  node->create_subscription<msg>(topic, queue, callback)

#define ROS_TIME_NOW()  rclcpp::Clock().now()
#define ROS_SPIN()      rclcpp::spin(node)
#define ROS_SPIN_ONCE() rclcpp::spin_some(node)
#define ROS_OK()        rclcpp::ok()
#define ROS_SHUTDOWN()  rclcpp::shutdown()

#endif
#endif  // FF_COMMON_ROS_H_
