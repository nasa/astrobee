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
#include <ff_hw_msgs/ClearTerminate.h>
#include <ff_hw_msgs/ConfigurePayloadPower.h>
#include <ff_hw_msgs/ConfigureSystemLeds.h>
#include <ff_hw_msgs/SetEnabled.h>
#include <ff_hw_msgs/SetFlashlight.h>
#include <ff_msgs/AckCompletedStatus.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/AckStatus.h>
#include <ff_msgs/AgentStateStamped.h>
#include <ff_msgs/ArmAction.h>
#include <ff_msgs/CameraStatesStamped.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CompressedFile.h>
#include <ff_msgs/CompressedFileAck.h>
#include <ff_msgs/ConfigureCamera.h>
#include <ff_msgs/ControlCommand.h>
#include <ff_msgs/DockAction.h>
#include <ff_msgs/EnableCamera.h>
#include <ff_msgs/EnableRecording.h>
#include <ff_msgs/FaultState.h>
#include <ff_msgs/GuestScienceApk.h>
#include <ff_msgs/GuestScienceConfig.h>
#include <ff_msgs/GuestScienceState.h>
#include <ff_msgs/LocalizationAction.h>
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/PerchAction.h>
#include <ff_msgs/PlanStatusStamped.h>
#include <ff_msgs/SetDataToDisk.h>
#include <ff_msgs/SetInertia.h>
#include <ff_msgs/SetRate.h>
#include <ff_msgs/SetZones.h>
#include <ff_msgs/Zone.h>
#include <ff_util/config_client.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_service.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>

#include <json/json.h>
#include <json/value.h>

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

  // Message and timeout callbacks
  void CameraStatesCallback(ff_msgs::CameraStatesStampedConstPtr const& state);
  void CmdCallback(ff_msgs::CommandStampedPtr const& cmd);
  void DataToDiskCallback(ff_msgs::CompressedFileConstPtr const& data);
  void DockStateCallback(ff_msgs::DockStateConstPtr const& state);
  void FaultStateCallback(ff_msgs::FaultStateConstPtr const& state);
  void GuestScienceAckCallback(ff_msgs::AckStampedConstPtr const& ack);
  void GuestScienceConfigCallback(ff_msgs::GuestScienceConfigConstPtr const&
                                                                        config);
  void GuestScienceStateCallback(ff_msgs::GuestScienceStateConstPtr const&
                                                                        state);
  void GuestScienceCustomCmdTimeoutCallback(ros::TimerEvent const& te);
  void GuestScienceStartStopCmdTimeoutCallback(ros::TimerEvent const& te);
  void InertiaCallback(geometry_msgs::InertiaStampedConstPtr const& inertia);
  void LedConnectedCallback();
  void MotionStateCallback(ff_msgs::MotionStatePtr const& state);
  void PerchStateCallback(ff_msgs::PerchStateConstPtr const& state);
  void PlanCallback(ff_msgs::CompressedFileConstPtr const& plan);
  void SysMonitorHeartbeatCallback(ff_msgs::HeartbeatConstPtr const& heartbeat);
  void SysMonitorTimeoutCallback(ros::TimerEvent const& te);
  void WaitCallback(ros::TimerEvent const& te);
  void ZonesCallback(ff_msgs::CompressedFileConstPtr const& zones);

  // Action based commands
  bool AreActionsRunning();
  void CancelAction(Action action, std::string cmd);
  bool FillArmGoal(ff_msgs::CommandStampedPtr const& cmd);
  bool FillDockGoal(ff_msgs::CommandStampedPtr const& cmd);
  bool FillMotionGoal(Action action,
                      ff_msgs::CommandStampedPtr const& cmd = nullptr);
  bool IsActionRunning(Action action);
  bool StartAction(Action action, std::string const& cmd_id);
  bool RemoveAction(Action action);

  // Action callbacks
  void ArmResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                         ff_msgs::ArmResultConstPtr const& result);

  void DockResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                          ff_msgs::DockResultConstPtr const& result);

  void LocalizationResultCallback(
                            ff_util::FreeFlyerActionState::Enum const& state,
                            ff_msgs::LocalizationResultConstPtr const& result);

  void MotionFeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback);
  void MotionResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                           ff_msgs::MotionResultConstPtr const& result);

  void PerchResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                           ff_msgs::PerchResultConstPtr const& result);

  // Publishers
  void PublishCmdAck(std::string const& cmd_id,
                     uint8_t completed_status = ff_msgs::AckCompletedStatus::OK,
                     std::string const& message = "",
                     uint8_t status = ff_msgs::AckStatus::COMPLETED);
  void PublishPlan();
  void PublishPlanStatus(uint8_t status);

  // Getters
  ff_msgs::MobilityState GetMobilityState();
  uint8_t GetPlanExecState();
  std::string GetRunPlanCmdId();

  // Setters
  void SetMobilityState();
  void SetMobilityState(uint8_t state, uint32_t sub_state = 0);
  void SetOpState(OpState* state);
  void SetPlanExecState(uint8_t state);
  void SetRunPlanCmdId(std::string cmd_id);

  // Helper functions
  void AckMobilityStateIssue(ff_msgs::CommandStampedPtr const& cmd,
                             std::string const& current_mobility_state,
                             std::string const& accepted_mobility_state = "");
  bool ArmControl(ff_msgs::CommandStampedPtr const& cmd);
  bool CheckNotMoving(ff_msgs::CommandStampedPtr const& cmd);
  bool CheckServiceExists(ros::ServiceClient& serviceIn,
                          std::string const& serviceName,
                          std::string const& cmd_in);
  bool CheckStoppedOrDrifting(std::string const& cmd_id,
                              std::string const& cmd_name);
  bool ConfigureLed(ff_hw_msgs::ConfigureSystemLeds& led_srv);
  bool ConfigureMobility(std::string const& cmd_id);
  bool ConfigureMobility(bool move_to_start,
                         std::string& err_msg);
  ros::Time MsToSec(std::string timestamp);
  bool PowerItem(ff_msgs::CommandStampedPtr const& cmd, bool on);
  bool ResetEkf(std::string const& cmd_id);
  void StartWaitTimer(float duration);
  void StopWaitTimer();

  // Plan related functions
  bool AckCurrentPlanItem();
  sequencer::ItemType GetCurrentPlanItemType();
  ff_msgs::CommandStampedPtr GetPlanCommand();
  bool GetSetPlanInertia(std::string const& cmd_id);
  void GetSetPlanOperatingLimits();

  // Commands
  bool ArmPanAndTilt(ff_msgs::CommandStampedPtr const& cmd);
  bool AutoReturn(ff_msgs::CommandStampedPtr const& cmd);
  bool ClearData(ff_msgs::CommandStampedPtr const& cmd);
  bool CustomGuestScience(ff_msgs::CommandStampedPtr const& cmd);
  bool Dock(ff_msgs::CommandStampedPtr const& cmd);
  bool DownloadData(ff_msgs::CommandStampedPtr const& cmd);
  bool Fault(ff_msgs::CommandStampedPtr const& cmd);
  bool GripperControl(ff_msgs::CommandStampedPtr const& cmd);
  bool IdlePropulsion(ff_msgs::CommandStampedPtr const& cmd);
  bool InitializeBias(ff_msgs::CommandStampedPtr const& cmd);
  bool NoOp(ff_msgs::CommandStampedPtr const& cmd);
  bool PausePlan(ff_msgs::CommandStampedPtr const& cmd);
  bool Perch(ff_msgs::CommandStampedPtr const& cmd);
  bool PowerItemOff(ff_msgs::CommandStampedPtr const& cmd);
  bool PowerItemOn(ff_msgs::CommandStampedPtr const& cmd);
  bool Prepare(ff_msgs::CommandStampedPtr const& cmd);
  bool ReacquirePosition(ff_msgs::CommandStampedPtr const& cmd);
  bool ResetEkf(ff_msgs::CommandStampedPtr const& cmd);
  bool RunPlan(ff_msgs::CommandStampedPtr const& cmd);
  bool SetCamera(ff_msgs::CommandStampedPtr const& cmd);
  bool SetCameraRecording(ff_msgs::CommandStampedPtr const& cmd);
  bool SetCameraStreaming(ff_msgs::CommandStampedPtr const& cmd);
  bool SetCheckObstacles(ff_msgs::CommandStampedPtr const& cmd);
  bool SetCheckZones(ff_msgs::CommandStampedPtr const& cmd);
  bool SetDataToDisk(ff_msgs::CommandStampedPtr const& cmd);
  bool SetEnableAutoReturn(ff_msgs::CommandStampedPtr const& cmd);
  bool SetEnableImmediate(ff_msgs::CommandStampedPtr const& cmd);
  bool SetFlashlightBrightness(ff_msgs::CommandStampedPtr const& cmd);
  bool SetHolonomicMode(ff_msgs::CommandStampedPtr const& cmd);
  bool SetInertia(ff_msgs::CommandStampedPtr const& cmd);
  bool SetOperatingLimits(ff_msgs::CommandStampedPtr const& cmd);
  bool SetPlan(ff_msgs::CommandStampedPtr const& cmd);
  bool SetPlanner(ff_msgs::CommandStampedPtr const& cmd);
  bool SetTelemetryRate(ff_msgs::CommandStampedPtr const& cmd);
  bool SetTimeSync(ff_msgs::CommandStampedPtr const& cmd);
  bool SetZones(ff_msgs::CommandStampedPtr const& cmd);
  bool Shutdown(ff_msgs::CommandStampedPtr const& cmd);
  bool SkipPlanStep(ff_msgs::CommandStampedPtr const& cmd);
  bool StartGuestScience(ff_msgs::CommandStampedPtr const& cmd);
  bool StartRecording(ff_msgs::CommandStampedPtr const& cmd);
  bool StopAllMotion(ff_msgs::CommandStampedPtr const& cmd);
  bool StopArm(ff_msgs::CommandStampedPtr const& cmd);
  bool StopDownload(ff_msgs::CommandStampedPtr const& cmd);
  bool StopRecording(ff_msgs::CommandStampedPtr const& cmd);
  bool StopGuestScience(ff_msgs::CommandStampedPtr const& cmd);
  bool StowArm(ff_msgs::CommandStampedPtr const& cmd);
  bool SwitchLocalization(ff_msgs::CommandStampedPtr const& cmd);
  bool Undock(ff_msgs::CommandStampedPtr const& cmd);
  bool Unperch(ff_msgs::CommandStampedPtr const& cmd);
  bool Unterminate(ff_msgs::CommandStampedPtr const& cmd);
  bool Wait(ff_msgs::CommandStampedPtr const& cmd);
  bool WipeHlp(ff_msgs::CommandStampedPtr const& cmd);

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();
  bool ReadCommand(config_reader::ConfigReader::Table *response,
                   ff_msgs::CommandStampedPtr cmd);
  void PublishAgentState();
  OpState* state_;

  ExecutiveActionClient<ff_msgs::ArmAction> arm_ac_;
  ExecutiveActionClient<ff_msgs::DockAction> dock_ac_;
  ExecutiveActionClient<ff_msgs::LocalizationAction> localization_ac_;
  ExecutiveActionClient<ff_msgs::MotionAction> motion_ac_;
  ExecutiveActionClient<ff_msgs::PerchAction> perch_ac_;

  config_reader::ConfigReader config_params_;

  ff_msgs::AgentStateStamped agent_state_;

  ff_msgs::AckStamped ack_;

  ff_msgs::CommandStampedPtr sys_monitor_init_fault_response_;
  ff_msgs::CommandStampedPtr sys_monitor_heartbeat_fault_response_;

  ff_msgs::CompressedFileAck cf_ack_;
  ff_msgs::CompressedFileConstPtr plan_, zones_, data_to_disk_;

  ff_msgs::CameraStatesStamped camera_states_;
  ff_msgs::DockStateConstPtr dock_state_;
  ff_msgs::FaultStateConstPtr fault_state_;
  ff_msgs::GuestScienceConfigConstPtr guest_science_config_;
  ff_msgs::MotionStatePtr motion_state_;
  ff_msgs::PerchStateConstPtr perch_state_;

  ff_msgs::ArmGoal arm_goal_;
  ff_msgs::DockGoal dock_goal_;
  ff_msgs::LocalizationGoal localization_goal_;
  ff_msgs::MotionGoal motion_goal_;
  ff_msgs::PerchGoal perch_goal_;

  ff_util::FreeFlyerServiceClient<ff_hw_msgs::ConfigureSystemLeds> led_client_;

  geometry_msgs::InertiaStampedConstPtr current_inertia_;

  ros::NodeHandle nh_;

  ros::Publisher agent_state_pub_, cmd_ack_pub_, plan_pub_, plan_status_pub_;
  ros::Publisher cf_ack_pub_, gs_cmd_pub_;

  ros::ServiceClient zones_client_, laser_enable_client_;
  ros::ServiceClient front_flashlight_client_, back_flashlight_client_;
  ros::ServiceClient dock_cam_config_client_, dock_cam_enable_client_;
  ros::ServiceClient haz_cam_config_client_, haz_cam_enable_client_;
  ros::ServiceClient nav_cam_config_client_, nav_cam_enable_client_;
  ros::ServiceClient perch_cam_config_client_, perch_cam_enable_client_;
  ros::ServiceClient sci_cam_config_client_, sci_cam_enable_client_;
  ros::ServiceClient payload_power_client_, pmc_enable_client_;
  ros::ServiceClient set_inertia_client_, set_rate_client_;
  ros::ServiceClient set_data_client_, enable_recording_client_;
  ros::ServiceClient eps_terminate_client_;

  ros::Subscriber cmd_sub_, dock_state_sub_, fault_state_sub_, gs_ack_sub_;
  ros::Subscriber heartbeat_sub_, motion_sub_, plan_sub_, zones_sub_, data_sub_;
  ros::Subscriber gs_config_sub_, gs_state_sub_, camera_state_sub_;
  ros::Subscriber perch_state_sub_, inertia_sub_;

  ros::Timer gs_start_stop_command_timer_, gs_custom_command_timer_;
  ros::Timer reload_params_timer_, wait_timer_, sys_monitor_heartbeat_timer_;
  ros::Timer sys_monitor_startup_timer_;

  sequencer::Sequencer sequencer_;

  std::shared_ptr<ff_util::ConfigClient> choreographer_cfg_;
  std::shared_ptr<ff_util::ConfigClient> mapper_cfg_;

  std::string primary_apk_running_, run_plan_cmd_id_;
  std::string gs_start_stop_cmd_id_, gs_custom_cmd_id_;

  std::vector<Action> running_actions_;

  // Action timeouts
  double action_active_timeout_, gs_command_timeout_;
  double arm_feedback_timeout_, motion_feedback_timeout_;
  double dock_result_timeout_, perch_result_timeout_;
  double localization_result_timeout_, led_connected_timeout_;
  double sys_monitor_heartbeat_timeout_, sys_monitor_startup_time_secs_;

  int pub_queue_size_;
  int sub_queue_size_;

  // TODO(Katie) Move to Agent state stamped
  bool allow_blind_flying_;
  bool live_led_on_;
  bool sys_monitor_heartbeat_fault_blocking_;
  bool sys_monitor_init_fault_blocking_;
  bool sys_monitor_heartbeat_fault_occurring_;
  bool sys_monitor_init_fault_occurring_;
};
typedef std::unique_ptr<Executive> ExecutivePtr;
}  // namespace executive
#endif  // EXECUTIVE_EXECUTIVE_H_
