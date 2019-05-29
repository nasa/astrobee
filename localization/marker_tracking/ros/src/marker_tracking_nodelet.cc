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

#include <marker_tracking_node/marker_tracking_nodelet.h>

#include <common/init.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace marker_tracking_node {

MarkerTrackingNodelet::MarkerTrackingNodelet(void)
    : ff_util::FreeFlyerNodelet(NODE_AR_TAGS) {}

void MarkerTrackingNodelet::Initialize(ros::NodeHandle* nh) {
  common::InitFreeFlyerApplication(getMyArgv(), false);

  inst_.reset(new MarkerTracker(nh, GetPrivateHandle(), getName()));
}

};  // namespace marker_tracking_node

PLUGINLIB_EXPORT_CLASS(marker_tracking_node::MarkerTrackingNodelet,
  nodelet::Nodelet)
