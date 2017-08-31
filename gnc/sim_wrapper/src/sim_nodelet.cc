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

// Standard includes
#include <ros/ros.h>

// FSW utils
#include <ff_util/ff_nodelet.h>

// For plugin loading
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Interface
#include <sim_wrapper/sim.h>

// C++
#include <memory>

/**
 * \ingroup gnc
 */
namespace sim_wrapper {

class SimWrapperNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // Constructor
  SimWrapperNodelet() : ff_util::FreeFlyerNodelet(NODE_SIM_WRAPPER, false) {}

  // Destructor
  virtual ~SimWrapperNodelet() {}

 protected:
  // Called on initialization
  virtual void Initialize(ros::NodeHandle *nh) {
    sim_ = std::shared_ptr<Sim>(new sim_wrapper::Sim(nh));
    timer_ = nh->createTimer(ros::Rate(62.5),
      &SimWrapperNodelet::TimerCallback, this, false, true);
  }

  // Called when the simulaiton needs to be stepped forward
  void TimerCallback(ros::TimerEvent const& event) {
    sim_->Step();
  }

 protected:
  std::shared_ptr<Sim> sim_;    // simulator interface
  ros::Timer timer_;            // rate loop timer
};

PLUGINLIB_DECLARE_CLASS(sim_wrapper, SimWrapperNodelet,
                        sim_wrapper::SimWrapperNodelet, nodelet::Nodelet);

}  // namespace sim_wrapper
