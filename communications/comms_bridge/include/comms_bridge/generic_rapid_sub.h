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

#ifndef COMMS_BRIDGE_GENERIC_RAPID_SUB_H_
#define COMMS_BRIDGE_GENERIC_RAPID_SUB_H_

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "ros/ros.h"

#include "knDds/DdsEventLoop.h"

#include "knShare/Time.h"

namespace ff {

/**
 * @brief base class for rapid subscriber to ros publisher
 * @details base class for rapid subscriber to ros publisher.
 *          A kn::DdsEventLoop is run within its own thread of execution.
 *          Child classes must connect requested message and callback
 *          to m_ddsEventLoop and call startThread()
 */
class GenericRapidSub {
 protected:
  GenericRapidSub(const std::string& entity_name,
                  const std::string& subscribe_topic,
                  const std::string& subscriber_partition);

  ~GenericRapidSub();

  /**
   * Will start thread exection by calling threadExec()
   */
  virtual void StartThread();

  std::string subscribe_topic_;
  std::string subscriber_partition_;

  std::atomic<bool> alive_;
  std::thread thread_;
  kn::DdsEventLoop dds_event_loop_;

 private:
  /**
  * Function to execute within seperate thread
  *   process DdsEventLoop at 10Hz
  */
  virtual void ThreadExec();
};

typedef std::shared_ptr<GenericRapidSub> GenericRapidSubPtr;

}  // end namespace ff

#endif  // COMMS_BRIDGE_GENERIC_RAPID_SUB_H_
