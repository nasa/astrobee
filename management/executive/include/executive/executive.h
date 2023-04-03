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

#include <ff_hw_msgs/srv/clear_terminate.hpp>
#include <ff_hw_msgs/srv/configure_payload_power.hpp>
#include <ff_hw_msgs/srv/configure_system_leds.hpp>
#include <ff_hw_msgs/srv/set_enabled.hpp>
#include <ff_hw_msgs/srv/set_flashlight.hpp>

#include <ff_msgs/action/arm.hpp>
#include <ff_msgs/action/dock.hpp>
#include <ff_msgs/action/localization.hpp>
#include <ff_msgs/action/motion.hpp>
#include <ff_msgs/action/perch.hpp>
#include <ff_msgs/msg/ack_completed_status.hpp>
#include <ff_msgs/msg/ack_stamped.hpp>
#include <ff_msgs/msg/ack_status.hpp>
#include <ff_msgs/msg/agent_state_stamped.hpp>
#include <ff_msgs/msg/camera_states_stamped.hpp>
#include <ff_msgs/msg/command_constants.hpp>
#include <ff_msgs/msg/command_stamped.hpp>
#include <ff_msgs/msg/compressed_file.hpp>
#include <ff_msgs/msg/compressed_file_ack.hpp>
#include <ff_msgs/msg/control_command.hpp>
#include <ff_msgs/msg/fault_state.hpp>
#include <ff_msgs/msg/guest_science_apk.hpp>
#include <ff_msgs/msg/guest_science_config.hpp>
#include <ff_msgs/msg/guest_science_state.hpp>
#include <ff_msgs/msg/plan_status_stamped.hpp>
#include <ff_msgs/msg/zone.hpp>
#include <ff_msgs/srv/configure_camera.hpp>
#include <ff_msgs/srv/enable_camera.hpp>
#include <ff_msgs/srv/enable_recording.hpp>
#include <ff_msgs/srv/response_only.hpp>
#include <ff_msgs/srv/set_data_to_disk.hpp>
#include <ff_msgs/srv/set_float.hpp>
#include <ff_msgs/srv/set_inertia.hpp>
#include <ff_msgs/srv/set_rate.hpp>
#include <ff_msgs/srv/set_zones.hpp>
#include <ff_msgs/srv/unload_load_nodelet.hpp>

#include <ff_common/ff_names.h>
#include <ff_common/ff_ros.h>
#include <ff_util/config_client.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_component.h>
#include <ff_util/ff_service.h>

#include <std_srvs/srv/empty.hpp>

#include <json/json.h>
#include <json/value.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

using ff_msgs::msg::CommandConstants;
using ff_util::FreeFlyerServiceClient;

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
class Executive : public ff_util::FreeFlyerComponent {
 public:
  explicit Executive(rclcpp::NodeOptions const& options);
  ~Executive();

  // Message and timeout callbacks
  void CameraStatesCallback(
                      ff_msgs::msg::CameraStatesStamped::SharedPtr const state);
  void CmdCallback(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  void DataToDiskCallback(ff_msgs::msg::CompressedFile::SharedPtr const data);
  void DockStateCallback(ff_msgs::msg::DockState::SharedPtr const state);
  void FaultStateCallback(ff_msgs::msg::FaultState::SharedPtr const state);
  void GuestScienceAckCallback(ff_msgs::msg::AckStamped::SharedPtr const ack);
  void GuestScienceConfigCallback(
                      ff_msgs::msg::GuestScienceConfig::SharedPtr const config);
  void GuestScienceStateCallback(
                        ff_msgs::msg::GuestScienceState::SharedPtr const state);
  void GuestScienceCustomCmdTimeoutCallback();
  void GuestScienceStartStopRestartCmdTimeoutCallback();
  void InertiaCallback(
                  geometry_msgs::msg::InertiaStamped::SharedPtr const inertia);
  void LedConnectedCallback();
  void MotionStateCallback(ff_msgs::msg::MotionState::SharedPtr const state);
  void PerchStateCallback(ff_msgs::msg::PerchState::SharedPtr const state);
  void PlanCallback(ff_msgs::msg::CompressedFile::SharedPtr const plan);
  void SysMonitorHeartbeatCallback(
                            ff_msgs::msg::Heartbeat::SharedPtr const heartbeat);
  void SysMonitorTimeoutCallback();
  void WaitCallback();
  void ZonesCallback(ff_msgs::msg::CompressedFile::SharedPtr const zones);

  // Action based commands
  bool AreActionsRunning();
  void CancelAction(Action action, std::string cmd);
  bool FillArmGoal(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool FillDockGoal(ff_msgs::msg::CommandStamped::SharedPtr const cmd,
                    bool return_to_dock);
  bool FillMotionGoal(Action action,
                  ff_msgs::msg::CommandStamped::SharedPtr const cmd = nullptr);
  bool IsActionRunning(Action action);
  bool StartAction(Action action, std::string const& cmd_id);
  bool RemoveAction(Action action);

  // Action callbacks
  void ArmResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                    std::shared_ptr<const ff_msgs::action::Arm::Result> result);

  void DockResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
            std::shared_ptr<const ff_msgs::action::Dock::Result> const result);

  void LocalizationResultCallback(
          ff_util::FreeFlyerActionState::Enum const& state,
          std::shared_ptr<const ff_msgs::action::Localization::Result> result);

  void MotionFeedbackCallback(
            std::shared_ptr<const ff_msgs::action::Motion::Feedback> feedback);
  void MotionResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                std::shared_ptr<const ff_msgs::action::Motion::Result> result);

  void PerchResultCallback(ff_util::FreeFlyerActionState::Enum const& state,
                  std::shared_ptr<const ff_msgs::action::Perch::Result> result);

  // Publishers
  void PublishCmdAck(std::string const& cmd_id,
              uint8_t completed_status = ff_msgs::msg::AckCompletedStatus::OK,
              std::string const& message = "",
              uint8_t status = ff_msgs::msg::AckStatus::COMPLETED);
  void PublishPlan();
  void PublishPlanStatus(uint8_t status);

  // Getters
  ff_msgs::msg::MobilityState GetMobilityState();
  uint8_t GetPlanExecState();
  std::string GetRunPlanCmdId();

  // Setters
  void SetMobilityState();
  void SetMobilityState(uint8_t state, uint32_t sub_state = 0);
  void SetOpState(OpState* state);
  void SetPlanExecState(uint8_t state);
  void SetRunPlanCmdId(std::string cmd_id);

  // Helper functions
  void AckMobilityStateIssue(ff_msgs::msg::CommandStamped::SharedPtr const cmd,
                             std::string const& current_mobility_state,
                             std::string const& accepted_mobility_state = "");
  bool ArmControl(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool CheckServiceExists(bool serviceExists,
                          std::string const& serviceName,
                          std::string const& cmd_in);
  bool CheckStoppedOrDrifting(std::string const& cmd_id,
                              std::string const& cmd_name);
  bool ConfigureLed(
      ff_util::FreeFlyerService<ff_hw_msgs::srv::ConfigureSystemLeds>& led_srv);
  bool ConfigureMobility(bool move_to_start, std::string& err_msg);
  bool FailCommandIfMoving(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool LoadUnloadNodelet(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  rclcpp::Time MsToSec(std::string timestamp);
  bool PowerItem(ff_msgs::msg::CommandStamped::SharedPtr const cmd, bool on);
  bool ProcessGuestScienceCommand(
                            ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool ResetEkf(std::string const& cmd_id);
  void StartWaitTimer(double duration);
  void StopWaitTimer();

  // Output functions
  void Debug(std::string output);
  void Error(std::string output);
  void Info(std::string output);
  void Warn(std::string output);

  // Plan related functions
  bool AckCurrentPlanItem();
  sequencer::ItemType GetCurrentPlanItemType();
  ff_msgs::msg::CommandStamped::SharedPtr GetPlanCommand();
  bool GetSetPlanInertia(std::string const& cmd_id);
  void GetSetPlanOperatingLimits();

  // Commands
  bool ArmPanAndTilt(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool AutoReturn(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool CustomGuestScience(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool DeployArm(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool Dock(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool EnableAstrobeeIntercomms(
                            ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool Fault(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool GripperControl(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool IdlePropulsion(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool InitializeBias(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool LoadNodelet(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool NoOp(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool PausePlan(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool Perch(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool PowerItemOff(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool PowerItemOn(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool Prepare(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool ReacquirePosition(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool ResetEkf(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool RestartGuestScience(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool RunPlan(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetCamera(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetCameraRecording(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetCameraStreaming(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetCheckObstacles(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetCheckZones(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetDataToDisk(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetEnableAutoReturn(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetEnableImmediate(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetEnableReplan(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetFlashlightBrightness(
                            ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetHolonomicMode(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetInertia(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetOperatingLimits(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetPlan(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetPlanner(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetTelemetryRate(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SetZones(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SkipPlanStep(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool StartGuestScience(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool StartRecording(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool StopAllMotion(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool StopArm(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool StopGuestScience(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool StopRecording(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool StowArm(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool SwitchLocalization(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool Undock(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool UnloadNodelet(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool Unperch(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool Unterminate(ff_msgs::msg::CommandStamped::SharedPtr const cmd);
  bool Wait(ff_msgs::msg::CommandStamped::SharedPtr const cmd);

 protected:
  virtual void Initialize(NodeHandle &nh);
  bool ReadParams();
  bool ReadMapperParams();
  bool ReadCommand(config_reader::ConfigReader::Table *response,
                   ff_msgs::msg::CommandStamped::SharedPtr cmd);
  void PublishAgentState();
  OpState* state_;

  ExecutiveActionClient<ff_msgs::action::Arm> arm_ac_;
  ExecutiveActionClient<ff_msgs::action::Dock> dock_ac_;
  ExecutiveActionClient<ff_msgs::action::Localization> localization_ac_;
  ExecutiveActionClient<ff_msgs::action::Motion> motion_ac_;
  ExecutiveActionClient<ff_msgs::action::Perch> perch_ac_;

  config_reader::ConfigReader config_params_, mapper_config_params_;

  ff_msgs::msg::AgentStateStamped agent_state_;

  ff_msgs::msg::AckStamped ack_;

  ff_msgs::msg::CommandStamped::SharedPtr sys_monitor_init_fault_response_;
  ff_msgs::msg::CommandStamped::SharedPtr sys_monitor_heartbeat_fault_response_;

  ff_msgs::msg::CompressedFileAck::SharedPtr cf_ack_;
  ff_msgs::msg::CompressedFile::SharedPtr plan_, zones_, data_to_disk_;

  ff_msgs::msg::CameraStatesStamped::SharedPtr camera_states_;
  ff_msgs::msg::DockState::SharedPtr dock_state_;
  ff_msgs::msg::FaultState::SharedPtr fault_state_;
  ff_msgs::msg::GuestScienceConfig::SharedPtr guest_science_config_;
  ff_msgs::msg::MotionState::SharedPtr motion_state_;
  ff_msgs::msg::PerchState::SharedPtr perch_state_;

  ff_msgs::action::Arm::Goal arm_goal_;
  ff_msgs::action::Dock::Goal dock_goal_;
  ff_msgs::action::Localization::Goal localization_goal_;
  ff_msgs::action::Motion::Goal motion_goal_;
  ff_msgs::action::Perch::Goal perch_goal_;

  geometry_msgs::msg::InertiaStamped::SharedPtr current_inertia_;

  NodeHandle nh_;

  Publisher<ff_msgs::msg::AgentStateStamped> agent_state_pub_;
  Publisher<ff_msgs::msg::AckStamped> cmd_ack_pub_;
  Publisher<ff_msgs::msg::CompressedFile> plan_pub_;
  Publisher<ff_msgs::msg::PlanStatusStamped> plan_status_pub_;
  Publisher<ff_msgs::msg::CompressedFileAck> cf_ack_pub_;
  Publisher<ff_msgs::msg::CommandStamped> gs_cmd_pub_;

  FreeFlyerServiceClient<ff_msgs::srv::SetZones> zones_client_;
  FreeFlyerServiceClient<ff_hw_msgs::srv::SetEnabled> laser_enable_client_;
  FreeFlyerServiceClient<ff_hw_msgs::srv::SetFlashlight>
                                                      front_flashlight_client_;
  FreeFlyerServiceClient<ff_hw_msgs::srv::SetFlashlight>
                                                        back_flashlight_client_;
  FreeFlyerServiceClient<ff_msgs::srv::ConfigureCamera> dock_cam_config_client_;
  FreeFlyerServiceClient<ff_msgs::srv::EnableCamera> dock_cam_enable_client_;
  FreeFlyerServiceClient<ff_msgs::srv::ConfigureCamera> haz_cam_config_client_;
  FreeFlyerServiceClient<ff_msgs::srv::EnableCamera> haz_cam_enable_client_;
  FreeFlyerServiceClient<ff_msgs::srv::ConfigureCamera> nav_cam_config_client_;
  FreeFlyerServiceClient<ff_msgs::srv::EnableCamera> nav_cam_enable_client_;
  FreeFlyerServiceClient<ff_msgs::srv::ConfigureCamera> perch_cam_config_client_;
  FreeFlyerServiceClient<ff_msgs::srv::EnableCamera> perch_cam_enable_client_;
  FreeFlyerServiceClient<ff_msgs::srv::ConfigureCamera> sci_cam_config_client_;
  FreeFlyerServiceClient<ff_msgs::srv::EnableCamera> sci_cam_enable_client_;
  FreeFlyerServiceClient<ff_hw_msgs::srv::ConfigurePayloadPower>
                                                          payload_power_client_;
  FreeFlyerServiceClient<ff_hw_msgs::srv::SetEnabled> pmc_enable_client_;
  FreeFlyerServiceClient<ff_msgs::srv::SetInertia> set_inertia_client_;
  FreeFlyerServiceClient<ff_msgs::srv::SetRate> set_rate_client_;
  FreeFlyerServiceClient<ff_msgs::srv::SetDataToDisk> set_data_client_;
  FreeFlyerServiceClient<ff_msgs::srv::EnableRecording>
                                                      enable_recording_client_;
  FreeFlyerServiceClient<ff_hw_msgs::srv::ClearTerminate> eps_terminate_client_;
  FreeFlyerServiceClient<ff_msgs::srv::ResponseOnly>
                                    enable_astrobee_intercommunication_client_;
  FreeFlyerServiceClient<ff_msgs::srv::UnloadLoadNodelet> unload_load_nodelet_client_;
  FreeFlyerServiceClient<ff_msgs::srv::SetFloat> set_collision_distance_client_;
  FreeFlyerServiceClient<ff_hw_msgs::srv::ConfigureSystemLeds> led_client_;


  Subscriber<ff_msgs::msg::CameraStatesStamped> camera_state_sub_;
  Subscriber<ff_msgs::msg::CommandStamped> cmd_sub_;
  Subscriber<ff_msgs::msg::CompressedFile> data_sub_;
  Subscriber<ff_msgs::msg::DockState> dock_state_sub_;
  Subscriber<ff_msgs::msg::FaultState> fault_state_sub_;
  Subscriber<ff_msgs::msg::AckStamped> gs_ack_sub_;
  Subscriber<ff_msgs::msg::GuestScienceConfig> gs_config_sub_;
  Subscriber<ff_msgs::msg::GuestScienceState> gs_state_sub_;
  Subscriber<ff_msgs::msg::Heartbeat> heartbeat_sub_;
  Subscriber<geometry_msgs::msg::InertiaStamped> inertia_sub_;
  Subscriber<ff_msgs::msg::MotionState> motion_sub_;
  Subscriber<ff_msgs::msg::PerchState> perch_state_sub_;
  Subscriber<ff_msgs::msg::CompressedFile> plan_sub_;
  Subscriber<ff_msgs::msg::CompressedFile> zones_sub_;

  ff_util::FreeFlyerTimer gs_start_stop_restart_command_timer_;
  ff_util::FreeFlyerTimer gs_custom_command_timer_;
  ff_util::FreeFlyerTimer reload_params_timer_;
  ff_util::FreeFlyerTimer wait_timer_;
  ff_util::FreeFlyerTimer sys_monitor_heartbeat_timer_;
  ff_util::FreeFlyerTimer sys_monitor_startup_timer_;

  sequencer::Sequencer sequencer_;

  std::shared_ptr<ff_util::ConfigClient> choreographer_cfg_;
  std::shared_ptr<ff_util::ConfigClient> mapper_cfg_;

  std::string primary_apk_running_, run_plan_cmd_id_;
  std::string gs_start_stop_restart_cmd_id_, gs_custom_cmd_id_;

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
