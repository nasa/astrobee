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
#include <ff_msgs/ExecuteAction.h>
#include <ff_msgs/MoveAction.h>
#include <ff_msgs/StopAction.h>
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
  virtual OpState* StartupState(std::string const& cmd_id = "",
                                std::string const& cmd_origin = "");
  virtual OpState* HandleCmd(ff_msgs::CommandStampedPtr const& cmd);
  OpState* HandleCmd(ff_msgs::CommandStampedPtr const& cmd,
                     bool& completed,
                     bool& successful,
                     std::string& err_msg,
                     uint8_t& status,
                     bool plan = false);

  // Functions for system actions
  // Arm
  virtual OpState* HandleArmResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::ArmResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin);

  // Dock
  virtual OpState* HandleDockActive();
  virtual OpState* HandleDockFeedback(
                                ff_msgs::DockFeedbackConstPtr const& feedback);
  virtual OpState* HandleDockResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::DockResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin);

  // Execute
  virtual OpState* HandleExecuteActive();
  virtual OpState* HandleExecuteResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::ExecuteResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin);

  // Idle
  virtual OpState* HandleIdleResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::IdleResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin);

  // Perch
  virtual OpState* HandlePerchActive();
  virtual OpState* HandlePerchFeedback();
  virtual OpState* HandlePerchResult(std::string const& cmd_id,
                                     std::string const& cmd_origin);


  // Move
  virtual OpState* HandleMoveActive();
  virtual OpState* HandleMoveFeedback(
                                ff_msgs::MoveFeedbackConstPtr const& feedback);
  virtual OpState* HandleMoveResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::MoveResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin);

  // Stop
  virtual OpState* HandleStopActive();
  virtual OpState* HandleStopResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::StopResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin);

  // Switch
  virtual OpState* HandleSwitchResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::SwitchResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin);

  // Undock
  virtual OpState* HandleUndockFeedback(
                              ff_msgs::UndockFeedbackConstPtr const& feedback);
  virtual OpState* HandleUndockResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::UndockResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin);

  // Unperching
  virtual OpState* HandleUnperchActive();
  virtual OpState* HandleUnperchFeedback();
  virtual OpState* HandleUnperchResult(std::string const& cmd_id,
                                       std::string const& cmd_origin);

  virtual OpState* HandleWaitCallback();

  // TODO(Katie) Remove if you end up changing the start, custom, and stop
  // commands to actions.
  virtual OpState* HandleGuestScienceAck(ff_msgs::AckStampedConstPtr const&
                                                                          ack);

  void AckMobilityStateIssue(std::string cmd_id,
                             std::string cmd_origin,
                             std::string cmd_name,
                             std::string current_mobility_state,
                             std::string accepted_mobility_state = "");
  bool CheckNotMoving(std::string cmd_id,
                      std::string cmd_origin,
                      std::string cmd_name);

  std::string GenerateActionFailedMsg(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              std::string const& goal_name,
                              std::string const& action_result = "");

  std::string const& name() const {return name_;}
  unsigned char const& id() const {return id_;}

 protected:
  OpState(std::string const& name, unsigned char id);
  void SetExec(Executive *const exec);

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
