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

// This is the ROS2 astrobee branch. Check out the other branch for ros1.
// We keep the definitions for both versions to reducing merging hassle.
#define ROS2

#if ROS1
#include <ros/ros.h>

using NodeHandle = ros::NodeHandle*;

template<class MessageType>
using Publisher = ros::Publisher*;

#define FF_CREATE_PUBLISHER(pub, node, msg, topic, queue)                \
  ros::Publisher __publ = node.advertise<msg>(topic, queue); \
  pub = &__publ
#define FF_CREATE_SUBSCRIBER(node, msg, topic, queue, callback)  node.subscribe(topic, queue, callback)


template<class MessageType>
using Service = ros::ServiceServer*;
#define FF_CREATE_SERVICE(serv, msg, topic, callback)                       \
  ros::ServiceServer __serv = nh_private_.advertiseService(topic, callback, this); \
  serv = &__serv

template<class MessageType>
using ServiceClient = ros::ServiceClient*;
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

#define FF_SPIN()      ros::spin()
#define FF_SPIN_ONCE() ros::spinOnce()
#define FF_OK()        ros::ok()
#define FF_SHUTDOWN()  ros::shutdown()

#else
#include "ff_common/ff_names.h"
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace ros = rclcpp;

using NodeHandle = std::shared_ptr<rclcpp::Node>;

template<class MessageType>
using Publisher = std::shared_ptr<rclcpp::Publisher<MessageType>>;

template<class MessageType>
using Subscriber = std::shared_ptr<rclcpp::Subscription<MessageType>>;

inline rclcpp::QoS QoSType(std::string const& topic, size_t history_depth) {
  if (LatchedTopic(topic)) {
    rclcpp::QoS latched_qos(1);
    latched_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    return latched_qos;
  }
  return rclcpp::QoS(history_depth);
}

#define FF_CREATE_PUBLISHER(node, msg, topic, queue_size) \
  node->create_publisher<msg>(topic, QoSType(topic, queue_size))
#define FF_CREATE_SUBSCRIBER(node, msg, topic, queue_size, callback) \
  node->create_subscription<msg>(topic, QoSType(topic, queue_size), callback)

template<class MessageType>
using Service = std::shared_ptr<rclcpp::Service<MessageType>>;
#define FF_CREATE_SERVICE(node, msg, topic, callback) \
  node->create_service<msg>(topic, callback)

template<class MessageType>
using ServiceClient = std::shared_ptr<rclcpp::Client<MessageType>>;
using Duration = std::shared_ptr<rclcpp::Duration>;

#define FF_DEFINE_LOGGER(name) static const rclcpp::Logger _FF_LOGGER = rclcpp::get_logger(name);
#define FF_DEBUG(...)   RCLCPP_DEBUG(_FF_LOGGER, __VA_ARGS__)
#define FF_INFO(...)    RCLCPP_INFO(_FF_LOGGER, __VA_ARGS__)
#define FF_WARN(...)    RCLCPP_WARN(_FF_LOGGER, __VA_ARGS__)
#define FF_ERROR(...)   RCLCPP_ERROR(_FF_LOGGER, __VA_ARGS__)
#define FF_FATAL(...)   RCLCPP_FATAL(_FF_LOGGER, __VA_ARGS__)

#define FF_DEBUG_STREAM(...)   RCLCPP_DEBUG_STREAM(_FF_LOGGER, __VA_ARGS__)
#define FF_INFO_STREAM(...)    RCLCPP_INFO_STREAM(_FF_LOGGER, __VA_ARGS__)
#define FF_WARN_STREAM(...)    RCLCPP_WARN_STREAM(_FF_LOGGER, __VA_ARGS__)
#define FF_ERROR_STREAM(...)   RCLCPP_ERROR_STREAM(_FF_LOGGER, __VA_ARGS__)
#define FF_FATAL_STREAM(...)   RCLCPP_FATAL_STREAM(_FF_LOGGER, __VA_ARGS__)

#define toSec() seconds()

#define FF_SPIN()      rclcpp::spin(node_)
#define FF_SPIN_ONCE() rclcpp::spin_some(node_)
#define FF_OK()        rclcpp::ok()
#define FF_SHUTDOWN()  rclcpp::shutdown()

#endif
#endif  // FF_COMMON_FF_ROS_H_
