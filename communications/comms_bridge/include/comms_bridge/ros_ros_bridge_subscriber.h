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

#ifndef COMMS_BRIDGE_ROS_ROS_BRIDGE_SUBSCRIBER_H_
#define COMMS_BRIDGE_ROS_ROS_BRIDGE_SUBSCRIBER_H_

/* This is a specialization of ROSBridgeSubscriber using a ROS conduit
   that is another topic on the same ROS master
   ie, it's a ROS-ROS bridge over ROS
*/

#include <ff_msgs/RelayAdvertisement.h>
#include <ff_msgs/RelayContent.h>
#include <ff_msgs/RelayReset.h>

#include <comms_bridge/bridge_subscriber.h>

#include <string>
#include <map>

#define DEFAULT_ROSROSBRIDGE_PUB_META_TOPIC_PREFIX "/polymorph_relay"

class ROSROSBridgeSubscriber : public BridgeSubscriber {
 public:
  explicit ROSROSBridgeSubscriber(const std::string& meta_topic_prefix = DEFAULT_ROSROSBRIDGE_PUB_META_TOPIC_PREFIX);
  virtual ~ROSROSBridgeSubscriber();

  // Called with the mutex held
  virtual void subscribeTopic(std::string const& in_topic, const RelayTopicInfo& info);

  // Called with the mutex held
  virtual void advertiseTopic(const RelayTopicInfo& info);

  // Called with the mutex held
  virtual void relayMessage(const RelayTopicInfo& topic_info, ContentInfo const& content_info);

 protected:
  void setupMetaChannels();
  void setupReverseMetaChannels();
  void handleResetMessage(const ff_msgs::RelayReset::ConstPtr& msg);

  std::string m_meta_topic_prefix;
  unsigned int m_n_advertised;
  ros::Publisher m_advertiser_pub;
  ros::Publisher m_relayer_pub;
  ros::Subscriber m_reset_sub;

  std::map<std::string, bool> requested_resets;  // topics requiring re-advert

  // prohibit shallow copy or assignment
  ROSROSBridgeSubscriber(const ROSROSBridgeSubscriber&) {}
  void operator=(const ROSROSBridgeSubscriber&) {}
};

#endif  // COMMS_BRIDGE_ROS_ROS_BRIDGE_SUBSCRIBER_H_
