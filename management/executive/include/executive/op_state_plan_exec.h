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

  OpState* StartupState(std::string const& cmd_id);
  OpState* HandleCmd(ff_msgs::CommandStampedPtr const& cmd);

  OpState* HandleResult(ff_util::FreeFlyerActionState::Enum const& state,
                        std::string const& result_response,
                        std::string const& cmd_id,
                        Action const& action);

  OpState* HandleWaitCallback();

  OpState* HandleGuestScienceAck(ff_msgs::AckStampedConstPtr const& ack);

  void AckCmd(std::string const& cmd_id,
              uint8_t completed_status = ff_msgs::AckCompletedStatus::OK,
              std::string const& message = "",
              uint8_t status = ff_msgs::AckStatus::COMPLETED);

  void AckPlanCmdFailed(uint8_t completed_status, std::string const& message);

  bool PausePlan(ff_msgs::CommandStampedPtr const& cmd);

 protected:
  explicit OpStatePlanExec(std::string const& name, unsigned char id) :
    OpState(name, id), waiting_(false) {}

 private:
  // allow creation only by repo
  friend class OpStateRepo;

  OpState* HandleActionComplete(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              Action const& action,
                              std::string const& result);
  OpState* AckStartPlanItem();
  OpState* StartNextPlanItem();

  bool waiting_;

  // TODO(Katie) Remove before flight
  bool first_segment_;
};
}  // namespace executive
#endif  // EXECUTIVE_OP_STATE_PLAN_EXEC_H_
