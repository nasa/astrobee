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

#ifndef EXECUTIVE_EXECUTIVE_H_
#define EXECUTIVE_EXECUTIVE_H_

#include <config_reader/config_reader.h>
#include <executive/executive_action_client.h>
#include <executive/utils/sequencer/plan_io.h>
#include <executive/utils/sequencer/sequencer.h>
#include <ff_hw_msgs/SetEnabled.h>
#include <ff_hw_msgs/SetFlashlight.h>
#include <ff_msgs/AckCompletedStatus.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/AckStatus.h>
#include <ff_msgs/AgentStateStamped.h>
#include <ff_msgs/ArmAction.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CompressedFile.h>
#include <ff_msgs/CompressedFileAck.h>
#include <ff_msgs/ConfigureCamera.h>
#include <ff_msgs/ControlCommand.h>
#include <ff_msgs/DockAction.h>
#include <ff_msgs/EnableCamera.h>
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/PlanStatusStamped.h>
#include <ff_msgs/SetZones.h>
#include <ff_msgs/SwitchAction.h>
#include <ff_msgs/Zone.h>
#include <ff_util/config_client.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>

#include <json/json.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

using ff_msgs::CommandConstants;

namespace executive {
class OpState;

/**
 * Executive class is the mediator, responsible for
 * receiving broadcasted messages and forwarding to
 * specific nodes.
 *
 * Decisions are handled through current executing state
 *
 * ref: state & mediator design pattern
 * 
 */
class Executive : public ff_util::FreeFlyerNodelet {
 public:
  Executive();
  ~Executive();

  // callbacks, handled by states
  void CmdCallback(ff_msgs::CommandStampedPtr const& cmd);
  void DockStateCallback(ff_msgs::DockStatePtr const& state);
  void GuestScienceAckCallback(ff_msgs::AckStampedConstPtr const& ack);
  void PlanCallback(ff_msgs::CompressedFileConstPtr const& plan);
  void ZonesCallback(ff_msgs::CompressedFileConstPtr const& zones);

  // Action based commands
  bool FillArmGoal(ff_msgs::CommandStampedPtr const& cmd,
                   std::string& err_msg,
                   bool plan = false);
  bool FillDockGoal(ff_msgs::CommandStampedPtr const& cmd,
                    std::string& err_msg,
                    bool plan = false);
  bool FillMotionGoal(Action action,
                      ff_msgs::CommandStampedPtr const& cmd = nullptr);

  bool StartAction(Action action,
                   std::string const& cmd_id,
                   std::string const& cmd_origin,
                   std::string& err_msg,
                   bool plan = false);
  bool IsActionRunning(Action action);
  bool AreActionsRunning();
  void CancelAction(Action action);

  bool RemoveAction(Action action);

  void ArmResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                         ff_msgs::ArmResultConstPtr const& result);

  void DockActiveCallback();
  void DockFeedbackCallback(ff_msgs::DockFeedbackConstPtr const& feedback);
  void DockResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                          ff_msgs::DockResultConstPtr const& result);

  void MotionActiveCallback();
  void MotionFeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback);
  void MotionResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                            ff_msgs::MotionResultConstPtr const& result);

  void SwitchResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                            ff_msgs::SwitchResultConstPtr const& result);

  void PublishCmdAck(std::string const& cmd_id,
                     std::string const& cmd_origin,
                     uint8_t completed_status = ff_msgs::AckCompletedStatus::OK,
                     std::string const& message = "",
                     uint8_t status = ff_msgs::AckStatus::COMPLETED);

  void PublishPlan();
  void PublishPlanStatus(uint8_t status);

  ff_msgs::MobilityState GetMobilityState();

  void SetMobilityState(uint8_t state, uint32_t sub_state = 0);
  bool SetPlan();
  void SetPlanExecState(uint8_t state);
  void SetProximity(float proximity);
  std::string SetZones();

  ros::Time MsToSec(std::string timestamp);

  sequencer::ItemType GetCurrentPlanItemType();
  ff_msgs::CommandStampedPtr GetPlanCommand();
  bool AckCurrentPlanItem();
  uint8_t GetPlanExecState();

  bool SetOperatingLimits(std::vector<ff_msgs::CommandArg> const& conditions,
                          std::string& err_msg);

  bool ConfigureMobility(std::string const& cmd_id,
                         std::string const& cmd_origin,
                         std::string& err_msg,
                         bool plan = false);
  bool ConfigureMobility(bool move_to_start,
                         bool enable_holonomic,
                         std::string& err_msg);

  bool ResetEkf(std::string const& cmd_id,
                std::string const& cmd_origin);

  void StartWaitTimer(float duration);
  void StopWaitTimer();
  void WaitCallback(ros::TimerEvent const& te);

  bool StopAllMotion(bool &stop_started,
                     std::string const& cmd_id,
                     std::string const& cmd_origin,
                     bool plan = false);

  bool EnableAutoReturn(ff_msgs::CommandStampedPtr const& cmd);

  bool Dock(ff_msgs::CommandStampedPtr const& cmd,
            std::string& err_msg,
            uint8_t& completed_status,
            bool plan = false);

  bool Undock(ff_msgs::CommandStampedPtr const& cmd,
              std::string& err_msg,
              bool plan = false);

  bool SetCheckObstacles(ff_msgs::CommandStampedPtr const& cmd);

  bool SetCheckZones(ff_msgs::CommandStampedPtr const& cmd);

  bool SetHolonomicMode(ff_msgs::CommandStampedPtr const& cmd);

  void StopArm(std::string const& cmd_id, std::string const& cmd_origin);

  void StowArm(std::string const& cmd_id, std::string const& cmd_origin);

  void SkipPlanStep(std::string const& cmd_id, std::string const& cmd_origin);

  bool DownloadData(ff_msgs::CommandStampedPtr const& cmd, std::string& err_msg,
                    uint8_t& completed_status, bool plan = false);

  void StopDownload(ff_msgs::CommandStampedPtr const& cmd);

  bool ClearData(ff_msgs::CommandStampedPtr const& cmd, std::string& err_msg,
                 uint8_t& completed_status, bool plan = false);

  bool PowerOnItem(ff_msgs::CommandStampedPtr const& cmd, std::string& err_msg,
                   uint8_t& completed_status, bool plan = false);
  bool PowerOffItem(ff_msgs::CommandStampedPtr const& cmd, std::string& err_msg,
                    uint8_t& completed_status, bool plan = false);

  bool SetFlashlightBrightness(ff_msgs::CommandStampedPtr const& cmd,
                               std::string& err_msg,
                               uint8_t& completed_status,
                               bool plan = false);

  bool SetCamera(ff_msgs::CommandStampedPtr const& cmd, std::string& err_msg,
                 uint8_t& completed_status, bool plan = false);

  bool SetCameraRecording(ff_msgs::CommandStampedPtr const& cmd,
                             std::string& err_msg,
                             uint8_t& completed_status,
                             bool plan = false);

  bool SetCameraStreaming(ff_msgs::CommandStampedPtr const& cmd,
                          std::string& err_msg,
                          uint8_t& completed_status,
                          bool plan = false);

  bool SendGuestScienceCommand(ff_msgs::CommandStampedPtr const& cmd,
                               std::string& err_msg,
                               uint8_t& completed_status,
                               bool plan = false);

  void DetermineStartupMobilityState();

  void Shutdown(std::string const& cmd_id, std::string const& cmd_origin);

  void SetOpState(OpState* state);

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  void ReadParams();
  void PublishAgentState();
  OpState* state_;

  ExecutiveActionClient<ff_msgs::ArmAction> arm_ac_;
  ExecutiveActionClient<ff_msgs::DockAction> dock_ac_;
  ExecutiveActionClient<ff_msgs::MotionAction> motion_ac_;
  ExecutiveActionClient<ff_msgs::SwitchAction> switch_ac_;

  config_reader::ConfigReader config_params_;

  ff_msgs::AgentStateStamped agent_state_;

  ff_msgs::AckStamped ack_;

  ff_msgs::CompressedFileAck cf_ack_;
  ff_msgs::CompressedFileConstPtr plan_, zones_;

  ff_msgs::ArmGoal arm_goal_;
  ff_msgs::DockGoal dock_goal_;
  ff_msgs::MotionGoal motion_goal_;
  ff_msgs::SwitchGoal switch_goal_;

  ros::NodeHandle nh_;

  ros::Publisher agent_state_pub_, cmd_ack_pub_, plan_pub_, plan_status_pub_;
  ros::Publisher cf_ack_pub_, gs_cmd_pub_;

  ros::ServiceClient zones_client_, laser_enable_client_, reset_ekf_client_;
  ros::ServiceClient front_flashlight_client_, back_flashlight_client_;
  ros::ServiceClient dock_cam_config_client_, dock_cam_enable_client_;
  ros::ServiceClient nav_cam_config_client_, nav_cam_enable_client_;

  ros::Subscriber cmd_sub_, dock_state_sub_, gs_ack_sub_, plan_sub_, zones_sub_;

  ros::Timer reload_params_timer_, wait_timer_;

  sequencer::Sequencer sequencer_;

  std::shared_ptr<ff_util::ConfigClient> choreographer_cfg_;
  std::shared_ptr<ff_util::ConfigClient> mapper_cfg_;

  std::vector<Action> running_actions_;

  // Action timeouts
  double action_active_timeout_;
  double arm_feedback_timeout_, motion_feedback_timeout_;
  double dock_result_timeout_, perch_result_timeout_, switch_result_timeout_;

  int pub_queue_size_;
  int sub_queue_size_;

  // TODO(Katie) Move to Agent state stamped
  bool allow_blind_flying_;
};
typedef std::unique_ptr<Executive> ExecutivePtr;
}  // namespace executive
#endif  // EXECUTIVE_EXECUTIVE_H_
