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
#ifndef LOCALIZATION_ANALYSIS_MESSAGE_BUFFER_H_
#define LOCALIZATION_ANALYSIS_MESSAGE_BUFFER_H_

#include <localization_analysis/message_buffer_params.h>
#include <localization_common/time.h>
#include <localization_common/utilities.h>

#include <map>

namespace localization_analysis {
template <typename MessageType>
class MessageBuffer {
 public:
  explicit MessageBuffer(const MessageBufferParams& params) : params_(params) {}
  // Assumes messages are buffered in time order
  void BufferMessage(const MessageType& msg) {
    const localization_common::Time timestamp = localization_common::TimeFromHeader(msg.header);
    if (last_measurement_time_ && std::abs(*last_measurement_time_ - timestamp) < params_.min_msg_spacing) {
      LOG(WARNING) << "BufferMessage: Dropping message that arrived too close to previous message.";
      return;
    }
    msg_buffer_.emplace(timestamp, msg);
    last_measurement_time_ = timestamp;
  }

  boost::optional<MessageType> GetMessage(const localization_common::Time current_time) {
    if (msg_buffer_.empty()) return boost::none;
    if (current_time - msg_buffer_.cbegin()->first < params_.msg_delay) {
      VLOG(2) << "GetMessage: Current time too close to message time, no message available.";
      return boost::none;
    }
    const auto msg = msg_buffer_.cbegin()->second;
    msg_buffer_.erase(msg_buffer_.begin());
    return msg;
  }

 private:
  MessageBufferParams params_;
  std::map<localization_common::Time, MessageType> msg_buffer_;
  boost::optional<localization_common::Time> last_measurement_time_;
};
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_MESSAGE_BUFFER_H_
