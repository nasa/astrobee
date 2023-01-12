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

#ifndef FF_COMMON_FF_ROS_H_
#define FF_COMMON_FF_ROS_H_

#if ROS1
#include <ros/ros.h>

using NodeHandle = ros::NodeHandle*;

// #define ROS_NODE_VAR &nh_private_

#define ROS_CREATE_NODE(name)  \
  ros::init(argc, argv, name); \
  ros::NodeHandle nh;          \
  ros::NodeHandle nh_private_("~");

template<class MessageType>
using Publisher = ros::Publisher*;

#define ROS_CREATE_PUBLISHER(pub, msg, topic, queue)                \
  ros::Publisher __publ = nh_private_.advertise<msg>(topic, queue); \
  pub = &__publ
#define ROS_CREATE_SUBSCRIBER(msg, topic, queue, callback)  nh_private_.subscribe(topic, queue, callback)


template<class MessageType>
using Service = ros::ServiceServer*;
#define FF_CREATE_SERVICE(serv, msg, topic, callback)                       \
  ros::ServiceServer __serv = nh_private_.advertiseService(topic, callback, this); \
  serv = &__serv

template<class MessageType>
using ServiceClient = ros::ServiceClient*;
#define ROS_CREATE_SERVICE_(serv_client, msg, topic)
ros::ServiceClient __serv_client = nh_private_.serviceClient<msg>(topic);
serv_client = &__serv_client

using Duration = ros::Duration*;

#define FF_DEBUG(...)   ROS_DEBUG_NAMED(ros::this_node::getName(), __VA_ARGS__)
#define FF_INFO(...)    ROS_INFO_NAMED(ros::this_node::getName(), __VA_ARGS__)
#define FF_WARN(...)    ROS_WARN_NAMED(ros::this_node::getName(), __VA_ARGS__)
#define FF_ERROR(...)   ROS_ERROR_NAMED(ros::this_node::getName(), __VA_ARGS__)
#define FF_FATAL(...)   ROS_FATAL_NAMED(ros::this_node::getName(), __VA_ARGS__)

#define FF_DEBUG_STREAM(...)   ROS_DEBUG_STREAM_NAMED(ros::this_node::getName(), __VA_ARGS__)
#define FF_INFO_STREAM(...)    ROS_INFO_STREAM_NAMED(ros::this_node::getName(), __VA_ARGS__)
#define FF_WARN_STREAM(...)    ROS_WARN_STREAM_NAMED(ros::this_node::getName(), __VA_ARGS__)
#define FF_ERROR_STREAM(...)   ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), __VA_ARGS__)
#define FF_FATAL_STREAM(...)   ROS_FATAL_STREAM_NAMED(ros::this_node::getName(), __VA_ARGS__)


#define FF_TIME_NOW()  ros::Time::now()
#define FF_SPIN()      ros::spin()
#define FF_SPIN_ONCE() ros::spinOnce()
#define FF_OK()        ros::ok()
#define FF_SHUTDOWN()  ros::shutdown()

#else
#include "rclcpp/rclcpp.hpp"
namespace ros = rclcpp;

using NodeHandle = std::shared_ptr<rclcpp::Node>;

#define ROS_NODE_VAR node_

#define ROS_CREATE_NODE(name) \
  rclcpp::init(argc, argv);   \
  auto node_ = rclcpp::Node::make_shared(name, "/" name);

template<class MessageType>
using Publisher = std::shared_ptr<rclcpp::Publisher<MessageType>>;

#define ROS_CREATE_PUBLISHER(pub, msg, topic, queue)        pub = node_->create_publisher<msg>(topic, queue)
#define ROS_CREATE_SUBSCRIBER(msg, topic, queue, callback)  node_->create_subscription<msg>(topic, queue, callback)

template<class MessageType>
using Service = std::shared_ptr<rclcpp::Service<MessageType>>;
#define FF_CREATE_SERVICE(serv, msg, topic, callback) \
  serv = node_->create_service<msg>(topic, std::bind(callback, this, std::placeholders::_1, std::placeholders::_2))

template<class MessageType>
using ServiceClient = std::shared_ptr<rclcpp::Client<MessageType>>;
#define ROS_CREATE_SERVICE_CLIENT(serv_client, msg, topic) \
  serv_client = node_->create_client<msg>(topic)

using Duration = std::shared_ptr<rclcpp::Duration>;

#define FF_DEBUG(...)   RCLCPP_DEBUG(LOGGER, __VA_ARGS__)
#define FF_INFO(...)    RCLCPP_INFO(LOGGER, __VA_ARGS__)
#define FF_WARN(...)    RCLCPP_WARN(LOGGER, __VA_ARGS__)
#define FF_ERROR(...)   RCLCPP_ERROR(LOGGER, __VA_ARGS__)
#define FF_FATAL(...)   RCLCPP_FATAL(LOGGER, __VA_ARGS__)

#define FF_DEBUG_STREAM(...)   RCLCPP_DEBUG_STREAM(LOGGER, __VA_ARGS__)
#define FF_INFO_STREAM(...)    RCLCPP_INFO_STREAM(LOGGER, __VA_ARGS__)
#define FF_WARN_STREAM(...)    RCLCPP_WARN_STREAM(LOGGER, __VA_ARGS__)
#define FF_ERROR_STREAM(...)   RCLCPP_ERROR_STREAM(LOGGER, __VA_ARGS__)
#define FF_FATAL_STREAM(...)   RCLCPP_FATAL_STREAM(LOGGER, __VA_ARGS__)

#define toSec() seconds()

#define FF_TIME_NOW()  clock_->now()
#define FF_SPIN()      rclcpp::spin(node_)
#define FF_SPIN_ONCE() rclcpp::spin_some(node_)
#define FF_OK()        rclcpp::ok()
#define FF_SHUTDOWN()  rclcpp::shutdown()

#endif
#endif  // FF_COMMON_FF_ROS_H_
