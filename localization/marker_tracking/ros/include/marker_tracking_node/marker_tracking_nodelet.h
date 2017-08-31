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

#ifndef MARKER_TRACKING_NODE_MARKER_TRACKING_NODELET_H_
#define MARKER_TRACKING_NODE_MARKER_TRACKING_NODELET_H_

#include <marker_tracking_node/marker_tracker.h>

#include <ff_util/ff_nodelet.h>

namespace marker_tracking_node {

class MarkerTrackingNodelet : public ff_util::FreeFlyerNodelet {
 public:
  MarkerTrackingNodelet();
  virtual void Initialize(ros::NodeHandle* nh);
 private:
  boost::shared_ptr<MarkerTracker> inst_;
};

};  // namespace marker_tracking_node

#endif  // MARKER_TRACKING_NODE_MARKER_TRACKING_NODELET_H_

