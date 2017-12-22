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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Common includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/config_server.h>
#include <ff_util/conversion.h>

// Messages we use
#include <ff_msgs/EkfState.h>
#include <ff_msgs/SetEkfInput.h>
#include <ff_msgs/SwitchAction.h>

// Pipelines
#include <localization_manager/localization_pipeline_ml.h>
#include <localization_manager/localization_pipeline_ar.h>
#include <localization_manager/localization_pipeline_hr.h>
#include <localization_manager/localization_pipeline_pl.h>
#include <localization_manager/localization_pipeline_gt.h>
#include <localization_manager/localization_pipeline_no.h>

// STL includes
#include <functional>
#include <string>
#include <map>

/**
 * \ingroup localization
 */
namespace localization_manager {

// Convenience declarations
typedef std::map<std::string, std::shared_ptr<Pipeline>> PipelineMap;

// The manager nodelet for switching between localization modes
class LocalizationManagerNodelet : public ff_util::FreeFlyerNodelet {
 public:
  enum State : uint8_t {
    STATE_INITIALIZING           = 0,    // System not yet ready
    STATE_WAITING_FOR_SWITCH     = 1,    // Waiting for switch call
    STATE_WAITING_FOR_STABLE     = 2,    // Waiting for pipeline to report stable
    STATE_WAITING_FOR_CONFIDENCE = 3     // Eaiting for EKF confience to rise
  };
  enum Event : uint8_t {
    EVENT_CURRENT_STABLE         = 0,    // Current pipeline is stable
    EVENT_CURRENT_UNSTABLE       = 1,    // Current pipeline is unstable
    EVENT_GOAL_STABLE            = 2,    // Goal pipeline is stable
    EVENT_GOAL_UNSTABLE          = 3,    // Goal pipeline is unstable
    EVENT_EKF_STABLE             = 4,    // EKF stable
    EVENT_EKF_UNSTABLE           = 5,    // EKF unstable
    EVENT_CANCEL                 = 6,    // Cancel the operation in process
    EVENT_PREEMPT                = 7,    // Preemption request
    EVENT_NEW                    = 8     // New request
  };

  // Constructor
  LocalizationManagerNodelet() :
    ff_util::FreeFlyerNodelet(NODE_LOCALIZATION_MANAGER, true),
    state_(STATE_INITIALIZING) {}

  // Destructor
  virtual ~LocalizationManagerNodelet() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    // Cannot create until we have a callback function
    PipelineCallbackType cb = std::bind(&LocalizationManagerNodelet::PipelineCallback, this,
      std::placeholders::_1, std::placeholders::_2);

    // Set up the individual localization pipelines
    if (!AddPipeline(nh, GetPrivateHandle(), cb, ff_msgs::SetEkfInput::Request::MODE_NONE))
      NODELET_ERROR("Could not create the none localization pipeline");
    if (!AddPipeline(nh, GetPrivateHandle(), cb, ff_msgs::SetEkfInput::Request::MODE_MAP_LANDMARKS))
      NODELET_ERROR("Could not create the map_landmarks localization pipeline");
    if (!AddPipeline(nh, GetPrivateHandle(), cb, ff_msgs::SetEkfInput::Request::MODE_AR_TAGS))
      NODELET_ERROR("Could not create the ar_tags localization pipeline");
    if (!AddPipeline(nh, GetPrivateHandle(), cb, ff_msgs::SetEkfInput::Request::MODE_HANDRAIL))
      NODELET_ERROR("Could not create the handrail localization pipeline");
    if (!AddPipeline(nh, GetPrivateHandle(), cb, ff_msgs::SetEkfInput::Request::MODE_PERCH))
      NODELET_ERROR("Could not create the perch localization pipeline");
    if (!AddPipeline(nh, GetPrivateHandle(), cb, ff_msgs::SetEkfInput::Request::MODE_TRUTH))
      NODELET_ERROR("Could not create the ground truth localization pipeline");

    // Read the node parameters
    cfg_.Initialize(GetPrivateHandle(), "localization/localization_manager.config");
    cfg_.Listen(boost::bind(&LocalizationManagerNodelet::ReconfigureCallback, this, _1));
    cfg_.Lim<std::string>("pipeline", EnumeratePipelines());
    cfg_.Lim<std::string>("fallback", EnumeratePipelines());

    // Set EKF input service
    service_i_.SetConnectedTimeout(cfg_.Get<double>("timeout_service_set_input"));
    service_i_.SetConnectedCallback(std::bind(&LocalizationManagerNodelet::ConnectedCallback, this));
    service_i_.SetTimeoutCallback(std::bind(&LocalizationManagerNodelet::TimeoutCallback, this));
    service_i_.Create(nh, SERVICE_GNC_EKF_SET_INPUT);

    // Enable optical flow service
    service_o_.SetConnectedTimeout(cfg_.Get<double>("timeout_service_enable_of"));
    service_o_.SetConnectedCallback(std::bind(&LocalizationManagerNodelet::ConnectedCallback, this));
    service_o_.SetTimeoutCallback(std::bind(&LocalizationManagerNodelet::TimeoutCallback, this));
    service_o_.Create(nh, SERVICE_LOCALIZATION_OF_ENABLE);

    // Timers to check for EKF stability / instability
    timer_s_ = nh->createTimer(ros::Duration(cfg_.Get<double>("timeout_stability")),
      &LocalizationManagerNodelet::TimerSCallback, this, true, false);
    timer_u_ = nh->createTimer(ros::Duration(cfg_.Get<double>("timeout_stability")),
      &LocalizationManagerNodelet::TimerUCallback, this, true, false);

    // Backup the NodeHandle for use in the ConnectedCallback
    nh_ = nh;
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    NODELET_DEBUG_STREAM("ConnectedCallback()");
    if (!service_i_.IsConnected()) return;     // Check service is connected
    if (!service_o_.IsConnected()) return;     // Check service is connected
    if (state_ != STATE_INITIALIZING) return;  // Prevent multiple calls

    // Set the fallback pipeline
    std::string fallback = "no";
    if (!cfg_.Get<std::string>("fallback", fallback))
      NODELET_ERROR("No valid fallback pipeline specfified in the localization manager");
    fall_ = pipelines_.find(fallback);
    if (fall_ == pipelines_.end())
      NODELET_ERROR("Fallback pipeline specified in config does not exist");

    // Set the current pipeline
    std::string pipeline = "no";
    if (!cfg_.Get<std::string>("pipeline", pipeline))
      NODELET_ERROR("No valid default pipeline specfified in the localization manager");
    curr_ = pipelines_.find(pipeline);
    if (curr_ == pipelines_.end())
      NODELET_ERROR("Default pipeline specified in config does not exist");

    // Set the goal pipeline
    goal_ = curr_;

    // Turn off all pipelines but the one we want
    for (PipelineMap::iterator it = pipelines_.begin(); it != pipelines_.end(); it++)
      if (!it->second->Enable(curr_ == it))
        NODELET_ERROR_STREAM("Could not toggle localization pipeline: " << it->first);

    // Forcibly EKF to use the current localization pipeline
    ff_msgs::SetEkfInput msg;
    msg.request.mode = curr_->second->GetMode();
    if (!service_i_.Call(msg))
      NODELET_ERROR_STREAM("Could not set the EKF to use the fallback localization pipeline");

    // Subscribe to EKF updates from GNC
    sub_ = nh_->subscribe(TOPIC_GNC_EKF, 1, &LocalizationManagerNodelet::EkfCallback, this);

    // Start the EKF stability timers - when a good EKF confidence calls back it simply defers
    // the deadline of the unstable timer. Therefore, if "timer_s_" calls back, the EKF has
    // been stable for duration "timeout_stability". Conversely, if "timer_u_" calls back.
    // the EKF has been unstable for "timeout_stability".
    Pipeline::StartTimer(timer_s_, cfg_.Get<double>("timeout_stability"));
    Pipeline::StartTimer(timer_u_, cfg_.Get<double>("timeout_stability"));

    // Set the mode to initialized
    ChangeState(STATE_WAITING_FOR_SWITCH);

    // Create the switch action
    action_.SetGoalCallback(std::bind(&LocalizationManagerNodelet::GoalCallback, this, std::placeholders::_1));
    action_.SetPreemptCallback(std::bind(&LocalizationManagerNodelet::PreemptCallback, this));
    action_.SetCancelCallback(std::bind(&LocalizationManagerNodelet::CancelCallback, this));
    action_.Create(nh_, ACTION_LOCALIZATION_MANAGER_SWITCH);
  }

  // Callback for a reconfigure (switch localization mode manually)
  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    if (state_ != STATE_WAITING_FOR_SWITCH) {
      NODELET_DEBUG("Cannot reconfigure in non-WAITING_FOR_SWITCH state");
      return false;
    }
    return cfg_.Reconfigure(config);
  }

  // Timeout on a trajectory generation request
  void TimeoutCallback(void) {
    NODELET_ERROR("ServiceTimeoutCallback()");
  }

  ////////////////////////////////////////////////////////////////////////////////////////

  // Convenience
  bool Error(std::string const& msg) {
    NODELET_ERROR_STREAM(msg);
    return false;
  }

  // Turn on the goal pipeline
  bool EnableNew() {
    // Sanity check
    if (goal_ == pipelines_.end())
      return Error("The goal pipeline pointer is invalid");
    // If the pipeline requires optic
    if (goal_->second->NeedsOpticalFlow()) {
      ff_msgs::SetBool msg;
      msg.request.enable = true;
      if (!service_o_.Call(msg))
        return Error("Could not enable optical flow for the goal pipeline");
    }
    // Enable the new pipeline
    if (!goal_->second->Enable(true))
      return Error("Could not enable the goal pipeline");
    // Success
    return true;
  }

  // Set the EKF to use the goal localizatin pipeline
  bool SetEKFInput() {
    // Sanity check
    if (goal_ == pipelines_.end())
      return Error("The goal pipeline pointer is invalid");
    // Immediately tell the EKF to use the goal localization pipeline
    ff_msgs::SetEkfInput msg;
    msg.request.mode = goal_->second->GetMode();
    if (!service_i_.Call(msg))
      return Error("Could not set the EKF to use the goal localization pipeline");
    // Success
    return true;
  }

  // Adopt the goal pipeline as the current pipeline
  bool DisableOld() {
    // Sanity check
    if (curr_ == pipelines_.end())
      return Error("The goal pipeline pointer is invalid");
    if (goal_ == pipelines_.end())
      return Error("The goal pipeline pointer is invalid");
    // If the goal doesn't require optical flow, turn it off
    if (!goal_->second->NeedsOpticalFlow()) {
      ff_msgs::SetBool msg;
      msg.request.enable = false;
      if (!service_o_.Call(msg))
        return Error("could not disable optical flow for the goal pipeline");
    }
    // Disable the oldpipeline
    if (!curr_->second->Enable(false))
      return Error("Could not enable the goal pipeline");
    // Set the current equal to the goal
    curr_ = goal_;
    // Success
    return true;
  }

  // Cancel a switch completely
  bool Revert() {
    // Sanity check
    if (curr_ == pipelines_.end()) return false;
    if (goal_ == pipelines_.end()) return false;
    // Immediately tell the EKF to use the current localization pipeline
    ff_msgs::SetEkfInput msg;
    msg.request.mode = curr_->second->GetMode();
    if (!service_i_.Call(msg))
      return Error("Could not set the EKF to use the fallback localization pipeline");
    // If the current pipeline does not require optical flow, disable it
    if (!curr_->second->NeedsOpticalFlow()) {
      ff_msgs::SetBool msg;
      msg.request.enable = false;
      if (!service_o_.Call(msg))
        return Error("Could not disable optical flow");
    }
    // Disable the goal pipeline
    if (!goal_->second->Enable(false))
      return Error("Could not disable the new pipeline");
    // Set the goal pipeline to the current pipeline
    goal_ = curr_;
    // Success
    return true;
  }

  // Fall back to a "safe" pipeline -- turn on the fallback pipeline and set the EKF to
  // imediately receive its features as the input.
  bool Fallback() {
    // If we don't allwo fallback, we'll have to deal with the bad current pipeling
    if (!cfg_.Get<bool>("enable_fallback"))
      return Error("Fallback is disabled");
    // Don't try and fallback to a pipeline we are currently using...
    if (fall_ == curr_) {
       NODELET_WARN("We are currently on the fallback pipeline. Have you set the bias?.");
       return true;
    }
    // Turn on optical flow if the fallback localization system requires it
    if (!fall_->second->NeedsOpticalFlow()) {
      ff_msgs::SetBool msg;
      msg.request.enable = true;
      if (!service_o_.Call(msg))
        return Error("Could not enable optical flow for the fallback localization pipeline");
    }
    // Enable the fallback pipeline
    if (!fall_->second->Enable(true))
      return Error("Could not enable the fallback localization pipeline");
    // Immediately switch to the EKF
    ff_msgs::SetEkfInput msg;
    msg.request.mode = fall_->second->GetMode();
    if (!service_i_.Call(msg))
      return Error("Could not set the EKF to use the fallback localization pipeline");
    // Disable both the current and goal pipelines
    if (curr_ != pipelines_.end())
      curr_->second->Enable(false);
    if (goal_ != pipelines_.end())
      goal_->second->Enable(false);
    // Switch both the current and goal pipelines to fallback
    curr_ = fall_;
    goal_ = fall_;
    // Success
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////

  // Control state change with nice debug output
  void ChangeState(State state) {
    switch (state) {
    case STATE_INITIALIZING:
      NODELET_DEBUG_STREAM("Moving to state INITIALIZING");                     break;
    case STATE_WAITING_FOR_SWITCH:
      NODELET_DEBUG_STREAM("Moving to state WAITING_FOR_SWITCH");               break;
    case STATE_WAITING_FOR_STABLE:
      NODELET_DEBUG_STREAM("Moving to state WAITING_FOR_STABLE");               break;
    case STATE_WAITING_FOR_CONFIDENCE:
      NODELET_DEBUG_STREAM("Moving to state WAITING_FOR_CONFIDENCE");           break;
    }
    state_ = state;
  }

  // Complete a switch action
  void Complete(int32_t response) {
    // Package up the response
    ff_msgs::SwitchResult switch_result;
    switch_result.response = response;
    if (response > 0)
      action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, switch_result);
    else if (response < 0)
      action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, switch_result);
    else
      action_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, switch_result);
    // Always return to a waiting state
    return ChangeState(STATE_WAITING_FOR_SWITCH);
  }

  // The state
  void StateMachine(Event event, std::string const& pipeline = "") {
    switch (event) {
    ///////////////////////////////////////////////////////////////////
    // WE RECEIVE NOTIFICATION THAT THE CURRENT PIPELINE GOES STABLE //
    ///////////////////////////////////////////////////////////////////

    case EVENT_CURRENT_STABLE: {
      switch (state_) {
      case STATE_WAITING_FOR_SWITCH:
      case STATE_WAITING_FOR_STABLE:
      case STATE_WAITING_FOR_CONFIDENCE:
        NODELET_DEBUG_STREAM("Pipline is stable");
        return;
      // We should never get here
      default:
        break;
      }
    }
    break;

    /////////////////////////////////////////////////////////////////////
    // WE RECEIVE NOTIFICATION THAT THE CURRENT PIPELINE GOES UNSTABLE //
    /////////////////////////////////////////////////////////////////////

    case EVENT_CURRENT_UNSTABLE: {
      switch (state_) {
      case STATE_WAITING_FOR_SWITCH:
      case STATE_WAITING_FOR_STABLE:
        if (!curr_->second->NeedsEKF()) {
          NODELET_DEBUG_STREAM("Ignoring unstable EKF");
        } else {
          NODELET_WARN_STREAM("Pipeline is unstable. Falling back to safe pipeline.");
          if (!Fallback())
            NODELET_ERROR_STREAM("Could not fall back to safe pipeline. Are you already on it?");
        }
        return;
      case STATE_WAITING_FOR_CONFIDENCE:
      // We should never get here
      default:
        break;
      }
    }
    break;

    ////////////////////////////////////////////////////////////////
    // WE RECEIVE NOTIFICATION THAT THE GOAL PIPELINE GOES STABLE //
    ////////////////////////////////////////////////////////////////

    case EVENT_GOAL_STABLE: {
      switch (state_) {
      // If we are waiting for pipeline or EKF stability, then we need to abort the switcj
      case STATE_WAITING_FOR_STABLE:
        NODELET_DEBUG_STREAM("Goal pipeline has gone stable. Switching EKF.");
        if (!SetEKFInput())
          return Complete(ff_msgs::SwitchResult::COULD_NOT_SWITCH_EKF);
        // Change the state
        ChangeState(STATE_WAITING_FOR_CONFIDENCE);
        // Special case: immediatelty fake EKF confidence
        if (!goal_->second->NeedsEKF())
          StateMachine(EVENT_EKF_STABLE);
        return;
      // Ignore stability updates during confidence check
      case STATE_WAITING_FOR_SWITCH:
      case STATE_WAITING_FOR_CONFIDENCE:
        return;
      // We should never get here
      default:
        break;
      }
    }
    break;

    //////////////////////////////////////////////////////////////////
    // WE RECEIVE NOTIFICATION THAT THE GOAL PIPELINE GOES UNSTABLE //
    //////////////////////////////////////////////////////////////////

    case EVENT_GOAL_UNSTABLE: {
      switch (state_) {
      // If we are waiting for pipeline or EKF stability, then we need to abort the switcj
      case STATE_WAITING_FOR_STABLE:
      case STATE_WAITING_FOR_CONFIDENCE:
        NODELET_WARN_STREAM("Goal pipeline has gone unstable. Aborting switch.");
        if (!Revert())
          return Complete(ff_msgs::SwitchResult::COULD_NOT_CANCEL);
        return Complete(ff_msgs::SwitchResult::PIPELINE_NOT_STABLE);
      // We should never get here
      default:
        break;
      }
    }
    break;

    //////////////////////////////////////////////////////
    // WE RECEIVE NOTIFICATION THAT THE EKF GOES STABLE //
    //////////////////////////////////////////////////////

    case EVENT_EKF_STABLE: {
      switch (state_) {
      // Nominal case: we expect stability pre and during switch
      case STATE_WAITING_FOR_SWITCH:
      case STATE_WAITING_FOR_STABLE:
        NODELET_DEBUG_STREAM("EKF is stable.");
        return;
      // If we are waiting for EKF confidence before completing switck
      case STATE_WAITING_FOR_CONFIDENCE:
        NODELET_DEBUG_STREAM("EKF is stable. Completing the switch");
        if (!DisableOld())
          return Complete(ff_msgs::SwitchResult::COULD_NOT_DISABLE_OLD_PIPELINE);
        // Nominal case
        return Complete(ff_msgs::SwitchResult::SUCCESS);
      // We should never get here
      default:
        break;
      }
    }
    break;

    ////////////////////////////////////////////////////////
    // WE RECEIVE NOTIFICATION THAT THE EKF GOES UNSTABLE //
    ////////////////////////////////////////////////////////

    case EVENT_EKF_UNSTABLE: {
      switch (state_) {
      // If in idle mode and EKF goes unstable, try to automatically revert to fallback pipeline
      case STATE_WAITING_FOR_SWITCH:
        NODELET_WARN_STREAM("EKF is unstable. Falling back to safe pipeline.");
        if (!Fallback())
          NODELET_ERROR_STREAM("Could not fall back to safe pipeline. Are you already on it?");
        return;
      // If current pipeline goes unstable prior ro switching, force the switch to keep stability.
      case STATE_WAITING_FOR_STABLE:
        NODELET_WARN_STREAM("Premature switch to goal pipeline to avoid unstable current pipleine");
        if (!SetEKFInput())
          return Complete(ff_msgs::SwitchResult::COULD_NOT_SWITCH_EKF);
        // Nominal behaviour
        return ChangeState(STATE_WAITING_FOR_STABLE);
      // If we are waiting for thew EKF to stabilize and it doesn't, return an error
      case STATE_WAITING_FOR_CONFIDENCE:
        NODELET_WARN_STREAM("EKF not stable in new pipeline. Reverting to previous pipeline.");
        if (!Revert())
          return Complete(ff_msgs::SwitchResult::COULD_NOT_CANCEL);
        return Complete(ff_msgs::SwitchResult::EKF_NOT_STABLE);
      // We should never get here
      default:
        break;
      }
    }
    break;

    //////////////////////////////////
    // WE RECEIVE A NEW SWITCH GOAL //
    //////////////////////////////////

    case EVENT_NEW: {
      switch (state_) {
      case STATE_WAITING_FOR_SWITCH: {
        PipelineMap::iterator test = pipelines_.find(pipeline);
        if (test == pipelines_.end())
          return Complete(ff_msgs::SwitchResult::PIPELINE_NOT_FOUND);
        if (curr_ == test)
          return Complete(ff_msgs::SwitchResult::PIPELINE_ALREADY_ACTIVE);
        // Try and initiate the switch
        if (test->second->NeedsOpticalFlow()) {
          ff_msgs::SetBool msg;
          msg.request.enable = true;
          if (!service_o_.Call(msg))
            return Complete(ff_msgs::SwitchResult::COULD_NOT_ENABLE_OPTICAL_FLOW);
        }
        // Enable the new pipeline
        if (!test->second->Enable(true))
          return Complete(ff_msgs::SwitchResult::COULD_NOT_ENABLE_NEW_PIPELINE);
        // Mark the goal as the test
        goal_ = test;
        // Change the state to waiting
        ChangeState(STATE_WAITING_FOR_STABLE);
        // Special case: If NONE is selected, fake stability
        if (!goal_->second->NeedsEKF())
          StateMachine(EVENT_GOAL_STABLE);
        // Always return at this point
        return;
      }
      // Disallowed state transitions
      case STATE_WAITING_FOR_STABLE:
      case STATE_WAITING_FOR_CONFIDENCE:
        return Complete(ff_msgs::SwitchResult::BAD_STATE_TRANSITION);
        break;
      case STATE_INITIALIZING:
      default:
        break;
      }
    }
    break;

    ////////////////
    // PREEMPTION //
    ////////////////

    case EVENT_PREEMPT: {
      switch (state_) {
      case STATE_WAITING_FOR_STABLE:
      case STATE_WAITING_FOR_CONFIDENCE:
        return Complete(ff_msgs::SwitchResult::PREEMPTED);
      case STATE_WAITING_FOR_SWITCH:
      case STATE_INITIALIZING:
      default:
        break;
      }
    }
    break;

    //////////////////
    // CANCELLATION //
    //////////////////

    case EVENT_CANCEL: {
      switch (state_) {
      case STATE_WAITING_FOR_STABLE:
      case STATE_WAITING_FOR_CONFIDENCE:
        return Complete(ff_msgs::SwitchResult::CANCELLED);
      case STATE_WAITING_FOR_SWITCH:
      case STATE_INITIALIZING:
      default:
        break;
      }
    }
    break;

    /////////////////////////
    // INVALID TRANSITIONS //
    /////////////////////////

    default:
      break;
    }

    // Catch all for bad state transitions
    NODELET_ERROR_STREAM("Unexpected event " << event << " in state " << state_);
  }

  ////////////////////////////////////////////////////////////////////////////////////////

  // If this ever gets called then we've been unstable for duration:timeout_stability
  void TimerSCallback(ros::TimerEvent const& event) {
    StateMachine(EVENT_EKF_STABLE);
    Pipeline::StartTimer(timer_s_, cfg_.Get<double>("timeout_stability"));
  }

  // If this ever gets called then we've been unstable for duration:timeout_stability
  void TimerUCallback(ros::TimerEvent const& event) {
    StateMachine(EVENT_EKF_UNSTABLE);
    Pipeline::StartTimer(timer_s_, cfg_.Get<double>("timeout_stability"));
    Pipeline::StartTimer(timer_u_, cfg_.Get<double>("timeout_stability"));
  }

  // Every time we get an EKF callback with valid data we defer the nstable callback
  void EkfCallback(ff_msgs::EkfState::ConstPtr const& msg) {
    if (msg->confidence <= cfg_.Get<int>("maximum_confidence"))
      Pipeline::StartTimer(timer_u_, cfg_.Get<double>("timeout_stability"));
  }

  // Called on a pipeline state change
  void PipelineCallback(std::string const& name, bool stable) {
    // Deal with events from the pipeline we are currently in
    if (curr_ != pipelines_.end() && name == curr_->first)
      return StateMachine(stable ? EVENT_CURRENT_STABLE : EVENT_CURRENT_UNSTABLE, name);
    // Deal with events from the pipeline to which we should switch
    if (goal_ != pipelines_.end() && name == goal_->first)
      return StateMachine(stable ? EVENT_GOAL_STABLE : EVENT_GOAL_UNSTABLE, name);
  }

  // Called when the localization mode must be switched
  void GoalCallback(ff_msgs::SwitchGoalConstPtr const& goal) {
    return StateMachine(EVENT_NEW, goal->pipeline);
  }

  // Called when a switch request is preempted by another switch request
  void PreemptCallback() {
    StateMachine(EVENT_CANCEL);
  }

  // Called when a switch request is preempted by another switch request
  void CancelCallback() {
    StateMachine(EVENT_PREEMPT);
  }

 private:
  // Enumerate all pipelines
  std::map<std::string, std::string> EnumeratePipelines() {
    std::map<std::string, std::string> enumeration;
    for (PipelineMap::iterator it = pipelines_.begin(); it != pipelines_.end(); it++)
      enumeration[it->second->GetName()] = it->second->GetDesc();
    return enumeration;
  }

  // Create a new pipeline
  bool AddPipeline(ros::NodeHandle *nh, ros::NodeHandle *nhp, PipelineCallbackType cb, uint8_t mode) {
    // Create the object
    std::shared_ptr<Pipeline> ptr = nullptr;
    switch (mode) {
    case ff_msgs::SetEkfInput::Request::MODE_NONE:
      ptr = std::shared_ptr<Pipeline>(new NOPipeline(nh, nhp, mode, cb));
      break;
    case ff_msgs::SetEkfInput::Request::MODE_MAP_LANDMARKS:
      ptr = std::shared_ptr<Pipeline>(new MLPipeline(nh, nhp, mode, cb));
      break;
    case ff_msgs::SetEkfInput::Request::MODE_AR_TAGS:
      ptr = std::shared_ptr<Pipeline>(new ARPipeline(nh, nhp, mode, cb));
      break;
    case ff_msgs::SetEkfInput::Request::MODE_HANDRAIL:
      ptr = std::shared_ptr<Pipeline>(new HRPipeline(nh, nhp, mode, cb));
      break;
    case ff_msgs::SetEkfInput::Request::MODE_PERCH:
      ptr = std::shared_ptr<Pipeline>(new PLPipeline(nh, nhp, mode, cb));
      break;
    case ff_msgs::SetEkfInput::Request::MODE_TRUTH:
      ptr = std::shared_ptr<Pipeline>(new GTPipeline(nh, nhp, mode, cb));
      break;
    default:
      break;
    }
    // If the pointer is null then there was a problem allocating the object
    if (ptr == nullptr)
      return false;
    // We now add this to the pipeline registry
    pipelines_[ptr->GetName()] = ptr;
    // Success
    return true;
  }

 private:
  State state_;                                                       // State
  ff_util::ConfigServer cfg_;                                         // Configuration server
  ff_util::FreeFlyerServiceClient<ff_msgs::SetEkfInput> service_i_;   // EKF set input service
  ff_util::FreeFlyerServiceClient<ff_msgs::SetBool> service_o_;       // Optical flow service
  ff_util::FreeFlyerActionServer<ff_msgs::SwitchAction> action_;      // Action server
  PipelineMap pipelines_;                                             // Available pipelines
  PipelineMap::iterator curr_;                                        // Current pipeline
  PipelineMap::iterator goal_;                                        // Goal pipeline
  PipelineMap::iterator fall_;                                        // Fallback pipeline
  ros::NodeHandle *nh_;                                               // Cached NodeHandle
  ros::Subscriber sub_;                                               // EKF subscriber
  ros::Timer timer_s_;                                                // Stability timer
  ros::Timer timer_u_;                                                // Timeout timer
};

PLUGINLIB_EXPORT_CLASS(localization_manager::LocalizationManagerNodelet, nodelet::Nodelet);

}  // namespace localization_manager
