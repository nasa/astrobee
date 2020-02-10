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

#ifndef EXECUTIVE_OP_STATE_H_
#define EXECUTIVE_OP_STATE_H_

#include <ros/ros.h>

#include <executive/executive.h>

#include <ff_msgs/ArmAction.h>
#include <ff_msgs/CommandArg.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/ControlState.h>
#include <ff_msgs/DockAction.h>
#include <ff_msgs/MotionAction.h>
#include <ff_util/ff_action.h>

#include <string>

using ff_msgs::CommandConstants;

namespace executive {
class Executive;
class OpStateRepo;
/**
 * OpState class must be derived from for each operating state
 * and added to the OpStateRepo, allowing explicit state specific behavior
 */
class OpState {
 public:
  virtual ~OpState() {}
  virtual OpState* StartupState(std::string const& cmd_id = "");
  virtual OpState* HandleCmd(ff_msgs::CommandStampedPtr const& cmd);
  OpState* HandleCmd(ff_msgs::CommandStampedPtr const& cmd,
                     bool& completed,
                     bool& successful);

  virtual OpState* HandleResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              std::string const& result_response,
                              std::string const& cmd_id,
                              Action const& action);

  virtual OpState* HandleWaitCallback();

  virtual OpState* HandleGuestScienceAck(
                                        ff_msgs::AckStampedConstPtr const& ack);

  virtual void AckCmd(std::string const& cmd_id,
                    uint8_t completed_status = ff_msgs::AckCompletedStatus::OK,
                    std::string const& message = "",
                    uint8_t status = ff_msgs::AckStatus::COMPLETED);

  std::string GenerateActionFailedMsg(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              Action const& action,
                              std::string const& action_result = "");

  std::string GetActionString(Action const& action);

  virtual bool PausePlan(ff_msgs::CommandStampedPtr const& cmd);

  OpState* TransitionToState(unsigned char id);

  std::string const& name() const {return name_;}
  unsigned char const& id() const {return id_;}

 protected:
  OpState(std::string const& name, unsigned char id);
  void SetExec(Executive *const exec);
  void SetPlanStatus(bool successful, std::string err_msg = "");

  std::string const name_;
  unsigned char const id_;
  Executive* exec_;

 private:
  friend class OpStateRepo;
  OpState (const OpState&) = delete;
  OpState& operator= (const OpState&) = delete;
};
typedef std::unique_ptr<OpState> OpStatePtr;
}  // namespace executive
#endif  // EXECUTIVE_OP_STATE_H_
