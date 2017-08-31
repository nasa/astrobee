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

#ifndef DDS_ROS_BRIDGE_RAPID_SUB_ROS_PUB_H_
#define DDS_ROS_BRIDGE_RAPID_SUB_ROS_PUB_H_

#include <memory>
#include <string>
#include <thread>

#include "ros/ros.h"
#include "knDds/DdsEventLoop.h"

namespace ff {

/**
 * @brief base class for rapid subscriber to ros publisher
 * @details base class for rapid subscriber to ros publisher.
 *          A kn::DdsEventLoop is run within its own thread of execution.
 *          Child classes must connect requeseted messege and callback
 *          to m_ddsEventLoop and call startThread()
 */
class RapidSubRosPub {
 protected:
  RapidSubRosPub(const std::string& subscribeTopic, const std::string& pubTopic,
                 const ros::NodeHandle &nh, const std::string& entityName,
                 const unsigned int queueSize);
  ~RapidSubRosPub();

  /**
   * Will start thread execution by calling threadExec()
   */
  virtual void StartThread();

  ros::NodeHandle m_nh_;
  ros::Publisher m_pub_;
  std::string m_subscribeTopic_;
  std::string m_publishTopic_;
  unsigned int m_queueSize_;

  std::thread m_thread_;
  kn::DdsEventLoop m_ddsEventLoop_;

 private:
  /**
  * Function to execute within seperate thread
  *   process DdsEventLoop at 10Hz
  */
  virtual void ThreadExec();
};

typedef std::shared_ptr<RapidSubRosPub> RapidSubRosPubPtr;

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_RAPID_SUB_ROS_PUB_H_
