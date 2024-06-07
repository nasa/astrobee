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

#ifndef COMMS_BRIDGE_GENERIC_ROS_SUB_RAPID_PUB_H_
#define COMMS_BRIDGE_GENERIC_ROS_SUB_RAPID_PUB_H_

/* This is a specialization of ROSBridgeSubscriber using a DDS conduit
*/

#include <comms_bridge/bridge_subscriber.h>
#include <comms_bridge/generic_rapid_pub.h>

#include <ff_msgs/GenericCommsAdvertisementInfo.h>
#include <ff_msgs/GenericCommsContent.h>
#include <ff_msgs/GenericCommsReset.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "dds_msgs/GenericCommsRequestSupport.h"

namespace ff {

class TopicEntry {
 public:
  TopicEntry() : connecting_robot_(""),
                 out_topic_(""),
                 rate_seconds_(0),
                 last_time_pub_(0),
                 seq_num_(0),
                 type_md5_sum_(""),
                 data_size_(0),
                 data_(NULL) {}
  TopicEntry(std::string robot_in, std::string topic_in, double rate_in) :
      connecting_robot_(robot_in),
      out_topic_(topic_in),
      rate_seconds_(rate_in),
      last_time_pub_(0),
      seq_num_(0),
      type_md5_sum_(""),
      data_size_(0),
      data_(NULL) {}

  void SetDataToSend(const int seq_num,
                     std::string const& md5_sum,
                     const size_t data_size,
                     uint8_t const* data) {
    seq_num_ = seq_num;
    type_md5_sum_ = md5_sum;
    data_size_ = data_size;
    data_ = data;
  }

  std::string connecting_robot_;
  std::string out_topic_;
  double rate_seconds_;

  // Info only needed for rate messages. A little wasted space but quicker
  // program execution
  double last_time_pub_;
  int seq_num_;
  std::string type_md5_sum_;
  size_t data_size_;
  uint8_t const* data_;
};

class GenericROSSubRapidPub : public BridgeSubscriber {
 public:
  explicit GenericROSSubRapidPub(ros::NodeHandle const* nh);
  ~GenericROSSubRapidPub();

  void AddTopics(std::map<std::string,
       std::vector<std::shared_ptr<TopicEntry>>> const& link_entries);

  float FloatMod(float x, float y);

  void InitializeDDS(std::vector<std::string> const& connections);

  // Called with the mutex held
  virtual void subscribeTopic(std::string const& in_topic,
                              const RelayTopicInfo& info);

  // Called with the mutex held
  virtual void advertiseTopic(const RelayTopicInfo& info);

  // Called with the mutex held
  virtual void relayMessage(const RelayTopicInfo& topic_info,
                            ContentInfo const& content_info);

  void ConvertRequest(rapid::ext::astrobee::GenericCommsRequest const* data,
                      std::string const& connecting_robot);

  void CheckPubMsg(const ros::TimerEvent& event);

 private:
  bool dds_initialized_, pub_rate_msgs_;

  ros::Timer rate_msg_pub_timer_;

  std::map<std::string, std::vector<std::shared_ptr<TopicEntry>>> topic_mapping_;
  std::map<std::string, GenericRapidPubPtr> robot_connections_;
  std::vector<std::shared_ptr<TopicEntry>> rate_topic_entries_;
};

}  // end namespace ff

#endif  // COMMS_BRIDGE_GENERIC_ROS_SUB_RAPID_PUB_H_
