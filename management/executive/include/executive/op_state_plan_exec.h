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

#ifndef EXECUTIVE_OP_STATE_PLAN_EXEC_H_
#define EXECUTIVE_OP_STATE_PLAN_EXEC_H_

#include <string>
#include "executive/op_state.h"
#include "executive/utils/sequencer/sequencer.h"
#include "ff_msgs/AckCompletedStatus.h"

namespace executive {
class OpStatePlanExec : public OpState {
 public:
  ~OpStatePlanExec() {}

  OpState* StartupState(std::string const& cmd_id,
                        std::string const& cmd_origin);
  OpState* HandleCmd(ff_msgs::CommandStampedPtr const& cmd);

  OpState* HandleArmResult(ff_util::FreeFlyerActionState::Enum const& state,
                           ff_msgs::ArmResultConstPtr const& result,
                           std::string const& cmd_id,
                           std::string const& cmd_origin);

  // Docking action stuff
  OpState* HandleDockActive(Action const& action);
  OpState* HandleDockFeedback(ff_msgs::DockFeedbackConstPtr const& feedback);
  OpState* HandleDockResult(ff_util::FreeFlyerActionState::Enum const& state,
                            ff_msgs::DockResultConstPtr const& result,
                            std::string const& cmd_id,
                            std::string const& cmd_origin,
                            Action const& action);

  // Teleop action stuff
  OpState* HandleMotionActive(Action const& action);
  OpState* HandleMotionResult(ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::MotionResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin,
                              Action const& action);

  OpState* HandleWaitCallback();

  // TODO(Katie) Remove if you end up changing the start, custom, and stop
  // commands to actions.
  OpState* HandleGuestScienceAck(ff_msgs::AckStampedConstPtr const& ack);

 protected:
  explicit OpStatePlanExec(std::string const& name, unsigned char id) :
    OpState(name, id), waiting_(false), run_plan_cmd_id_("") {}

 private:
  // allow creation only by repo
  friend class OpStateRepo;

  OpState* HandleCommandComplete(bool successful, std::string const& err_msg,
                                 uint8_t status);
  OpState* HandleActionComplete(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              std::string const& name = "",
                              std::string const& result = "");
  OpState* AckStartPlanItem();
  OpState* StartNextPlanItem();

  bool waiting_;

  std::string run_plan_cmd_id_;
  std::string run_plan_cmd_origin_;

  // TODO(Katie) Remove before flight
  bool first_segment_;
};
}  // namespace executive
#endif  // EXECUTIVE_OP_STATE_PLAN_EXEC_H_
