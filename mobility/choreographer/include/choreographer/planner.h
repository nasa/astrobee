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

#ifndef CHOREOGRAPHER_PLANNER_H_
#define CHOREOGRAPHER_PLANNER_H_

// Standard includes
#include <ros/ros.h>
#include <pcl/point_types.h>

// FSW libraries
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_flight.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <ff_util/conversion.h>
#include <jsonloader/keepout.h>
#include <mapper/point_cloud.h>

// FSW messages
#include <ff_msgs/PlanAction.h>
#include <ff_msgs/ControlState.h>
#include <ff_msgs/Zone.h>
#include <ff_msgs/RegisterPlanner.h>
#include <ff_msgs/GetZones.h>
#include <ff_msgs/GetMap.h>

#include <memory>
#include <string>
#include <vector>

/**
 * \ingroup mobility
 */
namespace planner {

// Convenience declarations
using RESPONSE = ff_msgs::PlanResult;

// Abstract class for implementing planners
class PlannerImplementation : public ff_util::FreeFlyerNodelet {
 private:
  enum State {
    INITIALIZING,
    WAITING,
    PLANNING
  };

 public:
  // The constructor takes the planner name and a brief description of it
  explicit PlannerImplementation(std::string const& name, std::string const& description) :
    ff_util::FreeFlyerNodelet(std::string(PREFIX_MOBILITY_PLANNER_PRIVATE) + name, true),
    state_(INITIALIZING) {
    // Configure the registration details
    registration_.request.name = name;
    registration_.request.description = description;
    registration_.request.unregister = false;
  }

  // Destructor deregisters with choreographer
  virtual ~PlannerImplementation() {
    // registration_.request.unregister = true;
    // client_r_.Call(registration_);
  }

 protected:
  // Derived classes must implement these functions
  virtual bool InitializePlanner(ros::NodeHandle *nh) = 0;
  virtual void PlanCallback(ff_msgs::PlanGoal const& goal) = 0;

  // Send the planner result
  void PlanResult(ff_msgs::PlanResult const& result) {
    switch (state_) {
    case PLANNING:
      NODELET_DEBUG_STREAM("Plan result received");
      NODELET_DEBUG_STREAM(result);
      Complete(result.response, result);
      break;
    default:
      NODELET_WARN_STREAM("Plan result received in non-planning state");
      break;
    }
  }

  // Send some planner feedback
  void PlanFeedback(ff_msgs::PlanFeedback const& feedback) {
    switch (state_) {
    case PLANNING:
      server_p_.SendFeedback(feedback);
      break;
    default:
      NODELET_WARN_STREAM("Plan feedback received in non-planning state");
      break;
    }
  }

  // Get the keepin and keepout zones
  bool GetZones(std::vector < ff_msgs::Zone > & zones) {
    ff_msgs::GetZones srv;
    if (client_z_.Call(srv)) {
      zones = srv.response.zones;
      return true;
    }
    return false;
  }
  bool GetFreeMap(pcl::PointCloud<pcl::PointXYZ> *points, float *resolution) {
    ff_msgs::GetMap srv;
    if (client_f_.Call(srv)) {
      pcl::fromROSMsg(srv.response.points, *points);
      *resolution = srv.response.resolution;
      return true;
    }
    return false;
  }
  bool GetObstacleMap(pcl::PointCloud<pcl::PointXYZ> *points, float *resolution) {
    ff_msgs::GetMap srv;
    if (client_o_.Call(srv)) {
      pcl::fromROSMsg(srv.response.points, *points);
      *resolution = srv.response.resolution;
      return true;
    }
    return false;
  }

 private:
  void Initialize(ros::NodeHandle *nh) {
    cfg_fm_.AddFile("flight.config");
    cfg_fm_.ReadFiles();
    // Listen for plan requests
    server_p_.SetGoalCallback(std::bind(&PlannerImplementation::GoalCallback, this, std::placeholders::_1));
    server_p_.SetPreemptCallback(std::bind(&PlannerImplementation::PreemptCallback, this));
    server_p_.SetCancelCallback(std::bind(&PlannerImplementation::CancelCallback, this));
    std::string topic = std::string(PREFIX_MOBILITY_PLANNER)
                      + registration_.request.name
                      + std::string(SUFFIX_MOBILITY_PLANNER);
    server_p_.Create(nh, topic);
    // Initialize the get zone call
    client_z_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_z_.SetTimeoutCallback(std::bind(&PlannerImplementation::GetZoneTimeoutCallback, this));
    client_z_.Create(nh, SERVICE_MOBILITY_GET_ZONES);
    // Initialize the register planner call
    client_r_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_r_.SetTimeoutCallback(std::bind(&PlannerImplementation::RegisterTimeoutCallback, this));
    client_r_.Create(nh, SERVICE_MOBILITY_PLANNER_REGISTER);
    // Initialize the free map call
    client_f_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_f_.SetTimeoutCallback(std::bind(&PlannerImplementation::GetFreeMapTimeoutCallback, this));
    client_f_.Create(nh, SERVICE_MOBILITY_GET_FREE_MAP);
    // Initialize the obstacle map call
    client_o_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_o_.SetTimeoutCallback(std::bind(&PlannerImplementation::GetObstacleMapTimeoutCallback, this));
    client_o_.Create(nh, SERVICE_MOBILITY_GET_OBSTACLE_MAP);
    // Initialize the planner itself
    if (!InitializePlanner(nh))
      InitFault("Planner could not be initialized");
  }

  // Deal with a fault in a responsible manner - note that this may also be
  // called if action and service servers timeout on conenction.
  void InitFault(std::string const& msg ) {
    NODELET_ERROR_STREAM(msg);
    AssertFault(ff_util::INITIALIZATION_FAILED, msg);
    return;
  }

  // Finish this action
  void Complete(int32_t response_code, ff_msgs::PlanResult result = ff_msgs::PlanResult()) {
    switch (state_) {
    case PLANNING:
      result.response = response_code;
      if (result.response > 0)
        server_p_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
      else if (result.response < 0)
        server_p_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      else
        server_p_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
      break;
    default:
      NODELET_WARN_STREAM("Plan result received in non-planning state");
      break;
    }
    // We are now back to waiting
    state_ = WAITING;
  }

  // Ensure all clients are connected
  void ConnectedCallback(void) {
    NODELET_DEBUG_STREAM("ConnectedCallback()");
    if (!client_z_.IsConnected()) return;  // Zone
    if (!client_r_.IsConnected()) return;  // Register
    if (!client_o_.IsConnected()) return;  // Register
    if (!client_f_.IsConnected()) return;  // Register
    if (state_ != INITIALIZING) return;    // Don't initialize twice
    // Register this planner
    NODELET_DEBUG_STREAM("Registering planner");
    client_r_.Call(registration_);
    // Move to waiting state
    state_ = WAITING;
  }

  // Timeout on a trajectory generation request
  void RegisterTimeoutCallback(void) {
    return InitFault("Timeout connecting to register service");
  }

  // Timeout on a trajectory generation request
  void GetZoneTimeoutCallback(void) {
    return InitFault("Timeout connecting to the get zone service");
  }

  // Timeout on a free map request
  void GetFreeMapTimeoutCallback(void) {
    return InitFault("Timeout connecting to the get free map service");
  }

  // Timeout on a obstacle map request
  void GetObstacleMapTimeoutCallback(void) {
    return InitFault("Timeout connecting to the get obstacle map service");
  }

  // Check that the value is less than the given bound
  bool CheckBound(double candidate, double upper_bound) {
    return (candidate <= upper_bound);
  }

  // Get the value
  double GetValue(double candidate, double upper_bound) {
    if (candidate > 0.0 && candidate <= upper_bound)
      return candidate;
    return upper_bound;
  }

  // Called when a new planning goal arrives
  void GoalCallback(ff_msgs::PlanGoalConstPtr const& old_goal) {
    NODELET_DEBUG_STREAM("A new plan request was just received");
    NODELET_DEBUG_STREAM(*old_goal);
    switch (state_) {
    default:
    case INITIALIZING:
      return Complete(RESPONSE::PROBLEM_CONNECTING_TO_SERVICES);
    case PLANNING:
      Complete(RESPONSE::PREEMPTED);
    case WAITING:
      break;
    }
    // We are now planning
    state_ = PLANNING;
    // Call the implementation with the plan goal
    return PlanCallback(*old_goal);
  }

  // Cancel the current operation
  void PreemptCallback(void) {
    return Complete(RESPONSE::PREEMPTED);
  }

  // Cancel the current operation
  void CancelCallback(void) {
    return Complete(RESPONSE::CANCELLED);
  }

 private:
  State state_;                                          // Planner state
  ff_util::FreeFlyerActionServer<ff_msgs::PlanAction> server_p_;
  ff_util::FreeFlyerServiceClient<ff_msgs::GetZones> client_z_;
  ff_util::FreeFlyerServiceClient<ff_msgs::RegisterPlanner> client_r_;
  ff_util::FreeFlyerServiceClient<ff_msgs::GetMap> client_f_, client_o_;
  ff_msgs::RegisterPlanner registration_;                // Registration info
  config_reader::ConfigReader cfg_fm_;                   // Configuration
};

}  // namespace planner

#endif  // CHOREOGRAPHER_PLANNER_H_
