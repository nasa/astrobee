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

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// ROS messages
// #include <octomap_msgs/Octomap.h>

// FSW includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_util/config_server.h>
#include <ff_util/conversion.h>

// FSW messages
#include <ff_msgs/ControlAction.h>
#include <ff_msgs/ControlCommand.h>

/**
 * \ingroup mobility
 */
namespace sentinel {

class SentinelNodelet : public ff_util::FreeFlyerNodelet {
 public:
  enum State {
    WAITING_FOR_INIT,
    WAITING_FOR_SEGMENT,
    WATCHING
  };

  // Constructor
  SentinelNodelet() : ff_util::FreeFlyerNodelet(NODE_SENTINEL, true), state_(WAITING_FOR_INIT) {}

  // Destructor
  ~SentinelNodelet() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    // Read the node parameters
    cfg_.Initialize(GetPrivateHandle(), "mobility/sentinel.config");
    cfg_.Listen(boost::bind(&SentinelNodelet::ReconfigureCallback, this, _1));
    // Setup a timer to forward diagnostics
    timer_d_ = nh->createTimer(ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
      &SentinelNodelet::DiagnosticsCallback, this, false, true);
    // Listen to the point cloud messages, EKF and shaper trajectories
    sub_s_ = nh->subscribe(TOPIC_GNC_CTL_SEGMENT, 1, &SentinelNodelet::SegmentCallback, this);
    // sub_m_ = nh->subscribe(TOPIC_MOBILITY_COLLISIONS, 1, &SentinelNodelet::MapCallback, this);
    // Publish collisions on this topic
    pub_c_ = nh->advertise < geometry_msgs::PointStamped > (TOPIC_MOBILITY_COLLISIONS, 5, true);
    // Switch to waiting for a segment
    state_ = WAITING_FOR_SEGMENT;
      // Notify initialization complete
    NODELET_DEBUG_STREAM("Initialization complete");
  }

  // Send diagnostics
  void DiagnosticsCallback(const ros::TimerEvent &event) {
    SendDiagnostics(cfg_.Dump());
  }

  // Callback for a reconfigure
  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    return cfg_.Reconfigure(config);
  }

  // Called when a new segment arrives for processing
  void SegmentCallback(const ff_msgs::ControlGoal::ConstPtr& msg) {
    if (state_ == WAITING_FOR_INIT) return;
    if (msg->mode == ff_msgs::ControlCommand::MODE_NOMINAL) {
      segment_ = msg->segment;
      state_ = WATCHING;
    } else {
      segment_.clear();
      state_ = WAITING_FOR_SEGMENT;
    }
  }

  // Called when new information is injected into the map
  // void MapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {}

  // ROS
 private:
  State state_;                 // State of the sentinel node
  ff_util::ConfigServer cfg_;   // Dynamic reconfiguration
  ros::Subscriber sub_s_;       // Segment subscription
  ros::Subscriber sub_m_;       // Map update subscription
  ros::Publisher pub_c_;        // Collision publisher
  ros::Timer timer_d_;          // Diangostic timer
  ff_util::Segment segment_;    // The segment in progress
};

PLUGINLIB_DECLARE_CLASS(sentinel, SentinelNodelet,
                        sentinel::SentinelNodelet, nodelet::Nodelet);

}  // namespace sentinel
