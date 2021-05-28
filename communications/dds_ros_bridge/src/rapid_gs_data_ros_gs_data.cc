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

#include "dds_ros_bridge/rapid_gs_data_ros_gs_data.h"

namespace {

#define GENERATE_GUEST_SCIENCE_CASE(NAME) \
  case rapid::ext::astrobee::GUEST_SCIENCE_##NAME: \
    return ff_msgs::GuestScienceData::NAME

#define GENERATE_GUEST_SCIENCE_DEFAULT(NAME) \
  default: \
    ROS_FATAL("DDS: Unknown guest science data type: %d", type); \
    return ff_msgs::GuestScienceData::NAME

uint8_t ConvertType(rapid::ext::astrobee::GuestScienceDataType type) {
  switch (type) {
    GENERATE_GUEST_SCIENCE_CASE(STRING);
    GENERATE_GUEST_SCIENCE_CASE(JSON);
    GENERATE_GUEST_SCIENCE_CASE(BINARY);
    GENERATE_GUEST_SCIENCE_DEFAULT(STRING);
  }
}

} // end namespace

namespace ff {

RapidGuestScienceDataToRos::RapidGuestScienceDataToRos(
                                        const std::string& subscribe_topic,
                                        const std::string& subscribe_partition,
                                        const std::string& pub_topic,
                                        const ros::NodeHandle &nh,
                                        const unsigned int queue_size)
  : RapidSubRosPub(subscribe_topic,
                   pub_topic,
                   nh,
                   "AstrobeeGuestScienceDataProfile",
                   queue_size) {
  // advertise ros topic
  pub_ = nh_.advertise<ff_msgs::GuestScienceData>(pub_topic, queue_size);
  subscriber_partition_ = subscribe_partition;

  // connect to ddsEventLoop
  // @todo confirm topic suffix has '-'
  try {
    dds_event_loop_.connect<rapid::ext::astrobee::GuestScienceData>(this,
                                  subscribe_topic,                    // topic
                                  subscribe_partition,                // name
                                  "AstrobeeGuestScienceDataProfile",  // profile
                                  "");                                // library
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidGuestScienceDataToRos exception: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidGuestScienceDataToRos exception unknown");
    throw;
  }

  // start thread
  StartThread();
}

void RapidGuestScienceDataToRos::operator() (
                          rapid::ext::astrobee::GuestScienceData const* data) {
  ff_msgs::GuestScienceData msg;
  util::RapidHeader2Ros(data->hdr, &msg.header);

  msg.apk_name = data->apkName;
  msg.data_type = ConvertType(data->type);
  msg.topic = data->topic;

  unsigned char* buf = data->data.get_contiguous_buffer();
  msg.data.reserve(data->data.length());
  msg.data.resize(data->data.length());
  std::memmove(msg.data.data(), buf, data->data.length());

  pub_.publish(msg);
}

}  // end namespace ff
