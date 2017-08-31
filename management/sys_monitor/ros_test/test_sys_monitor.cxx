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

#include <sys_monitor/sys_monitor.h>

#include <gtest/gtest.h>
#include <glog/logging.h>

#include <unistd.h>

TEST(SysMonitor, SysMonitor) {
  int argc = 0;
  ros::init(argc, NULL, "SysMonitor");
  sys_monitor::SysMonitor monitor;

  LOG(INFO) << ros::Time::now();
  ros::spinOnce();
  usleep(1000);
  ros::spinOnce();
  LOG(INFO) << ros::Time::now();
  EXPECT_TRUE(true);

  // This needs to end eventually, so no calling ros::spin();
}

TEST(SysMonitor, SysMonitor2) {
  int argc = 0;
  ros::init(argc, NULL, "SysMonitor2");
  sys_monitor::SysMonitor monitor;

  LOG(INFO) << ros::Time::now();
  ros::spinOnce();
  usleep(1000);
  ros::spinOnce();
  LOG(INFO) << ros::Time::now();
  EXPECT_FALSE(false);

  // This needs to end eventually, so no calling ros::spin();
}
