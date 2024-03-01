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

#include "comms_bridge/generic_rapid_sub.h"

#include <string>

namespace ff {

GenericRapidSub::GenericRapidSub(const std::string& entity_name, const std::string& subscribe_topic,
                                 const std::string& subscriber_partition)
    : dds_event_loop_(entity_name),
      subscribe_topic_(subscribe_topic),
      subscriber_partition_(subscriber_partition),
      alive_(true) {}

GenericRapidSub::~GenericRapidSub() {
  alive_ = false;  // Notify thread to exit
  thread_.join();
}

void GenericRapidSub::StartThread() {
  // Start joinable thread
  thread_ = std::thread(&GenericRapidSub::ThreadExec, this);
}

void GenericRapidSub::ThreadExec() {
  while (alive_) {
    // process events at 10hz
    dds_event_loop_.processEvents(kn::milliseconds(100));
  }
}

}  // end namespace ff
