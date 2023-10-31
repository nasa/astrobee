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

#ifndef COMMS_BRIDGE_ROS_ROS_BRIDGE_PUBLISHER_H_
#define COMMS_BRIDGE_ROS_ROS_BRIDGE_PUBLISHER_H_

/* This is a specialization of ROSBridgePublisher using a ROS conduit
   that is another topic on the same ROS master
   ie, it's a ROS-ROS bridge over ROS
*/

#include <ff_msgs/RelayAdvertisement.h>
#include <ff_msgs/RelayContent.h>
#include <ff_msgs/RelayReset.h>

#include <comms_bridge/bridge_publisher.h>

#include <string>

#define DEFAULT_ROSROSBRIDGE_PUB_META_TOPIC_PREFIX "/polymorph_relay"
// default time to delay between advertising and publishing on that topic [sec]
#define DEFAULT_ROSROSBRIDGE_PUB_ADVERTISE_DELAY 3.0

class ROSROSBridgePublisher : public ROSBridgePublisher {
 public:
  ROSROSBridgePublisher(const std::string& meta_topic_prefix = DEFAULT_ROSROSBRIDGE_PUB_META_TOPIC_PREFIX,
                        double ad2pub_delay = DEFAULT_ROSROSBRIDGE_PUB_ADVERTISE_DELAY);
  virtual ~ROSROSBridgePublisher();

 protected:
  void setupMetaChannels();
  void setupReverseMetaChannels();

  void requestTopicInfo(std::string const& output_topic);

  void handleContentMessage(const ros_bridge::RelayContent::ConstPtr& msg);
  void handleAdMessage(const ros_bridge::RelayAdvertisement::ConstPtr& msg);

  std::string m_meta_topic_prefix;
  ros::Subscriber m_advert_sub;
  ros::Subscriber m_content_sub;
  ros::Publisher m_reset_pub;

  // prohibit shallow copy or assignment
  ROSROSBridgePublisher(const ROSROSBridgePublisher&) : ROSBridgePublisher(m_ad2pub_delay) {}
  void operator=(const ROSROSBridgePublisher&) {}
};

#endif  // COMMS_BRIDGE_ROS_ROS_BRIDGE_PUBLISHER_H_
