/* Copyright (c) 2017, United State Government, as represented by the
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

#include "ground_dds_ros_bridge/rapid_guest_science_data.h"

namespace ff {

RapidGuestScienceDataToRos::RapidGuestScienceDataToRos(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RapidSubRosPub(subscribe_topic,
                   pub_topic,
                   nh,
                   "RapidGuestScienceDataToRos",
                   queue_size) {
  // advertise ros topic, make it latched
  pub_ = nh_.advertise<ff_msgs::GuestScienceData>(publish_topic_, queue_size);

  // connect to ddsEventLoop
  try {
    dds_event_loop_.connect<rapid::ext::astrobee::GuestScienceData>(this,
    rapid::ext::astrobee::GUEST_SCIENCE_DATA_TOPIC + subscribe_topic,  // topic
    "",                                                                // name
    "AstrobeeGuestScienceDataProfile",                                 // profile
    "");                                                               // library
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
                  rapid::ext::astrobee::GuestScienceData const* rapid_gs_data) {
  ff_msgs::GuestScienceData data;
  util::RapidHeader2Ros(rapid_gs_data->hdr, &data.header);

  data.apk_name = rapid_gs_data->apkName;

  if (rapid_gs_data->type == rapid::ext::astrobee::GUEST_SCIENCE_STRING) {
    data.data_type = ff_msgs::GuestScienceData::STRING;
  } else if (rapid_gs_data->type == rapid::ext::astrobee::GUEST_SCIENCE_JSON) {
    data.data_type = ff_msgs::GuestScienceData::JSON;
  } else if (rapid_gs_data->type ==
                                  rapid::ext::astrobee::GUEST_SCIENCE_BINARY) {
    data.data_type = ff_msgs::GuestScienceData::BINARY;
  }

  data.topic = rapid_gs_data->topic;

  unsigned char* buf = rapid_gs_data->data.get_contiguous_buffer();
  data.data.reserve(rapid_gs_data->data.length());
  data.data.resize(rapid_gs_data->data.length());
  std::memmove(data.data.data(), buf, rapid_gs_data->data.length());

  pub_.publish(data);
}

}  // end namespace ff
