#include "ctl_acados_mpc/ctl_mpc.h"

namespace ctl_mpc {

using RESPONSE = ff_msgs::ControlResult;

Ctl_MPC::Ctl_MPC(ros::NodeHandle* nh, std::string const& name) : 
    fsm_(WAITING, std::bind(&Ctl_MPC::UpdateCallback, this, std::placeholders::_1, std::placeholders::_2)),
    name_(name), control_enabled_(true), use_ground_truth_(false), inertia_received_(false), setpoint_received_(false), flight_enabled_(false) 
{
    // Configure the state machine
    fsm_.Add(WAITING, GOAL_NOMINAL, [this](FSM::Event const& event) -> FSM::State 
    {
        // If the timestamp is in the past, then we need to check its not too
        // stale. If it is stale, it might be indicative of timesync issues
        // between the MLP and LLP so we should reject the command
        mutex_segment_.lock();
        ros::Duration delta = segment_.front().when - ros::Time::now();
        if (delta.toSec() < - MAX_LATENCY) {
            mutex_segment_.unlock();
            return Result(RESPONSE::TIMESYNC_ISSUE);
        }
        // For deferred executions
        timer_.stop();
        timer_.setPeriod(delta, true);
        timer_.start();
        mutex_segment_.unlock();
        // Set to stopping mode until we start the segment
        SetCommand(ff_msgs::ControlCommand::MODE_STOP);
        // Update state
        return NOMINAL;
    });
    fsm_.Add(NOMINAL, GOAL_COMPLETE, [this](FSM::Event const& event) -> FSM::State 
    {
        if (!SetCommand(ff_msgs::ControlCommand::MODE_STOP))
            return Result(RESPONSE::CONTROL_FAILED);
        return STOPPING;
    });
    fsm_.Add(NOMINAL, GOAL_CANCEL, [this](FSM::Event const& event) -> FSM::State 
    {
        // In case of goal cancel, we want to stay at our current position with 0 velocity
        static ff_msgs::ControlCommand msg;
        msg.current.pose.position.x = acados_in.x0[0];
        msg.current.pose.position.y = acados_in.x0[1];
        msg.current.pose.position.z = acados_in.x0[2];
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(acados_out.x1[6], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(acados_out.x1[7], Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(acados_out.x1[8], Eigen::Vector3f::UnitZ());
        msg.current.pose.orientation.x = q.x();
        msg.current.pose.orientation.y = q.y();
        msg.current.pose.orientation.z = q.z();
        msg.current.pose.orientation.w = q.w();
        msg.current.twist.linear.x = 0;
        msg.current.twist.linear.y = 0;
        msg.current.twist.linear.z = 0;
        msg.current.twist.angular.x = 0;
        msg.current.twist.angular.y = 0;
        msg.current.twist.angular.z = 0;
        msg.next = msg.current;

        if (!SetCommand(ff_msgs::ControlCommand::MODE_STOP, msg))
            return Result(RESPONSE::CONTROL_FAILED);
        return Result(RESPONSE::CANCELLED);
    });
    fsm_.Add(WAITING, GOAL_STOP, [this](FSM::Event const& event) -> FSM::State 
    {
        if (!SetCommand(ff_msgs::ControlCommand::MODE_STOP))
            return Result(RESPONSE::CONTROL_FAILED);
        return STOPPING;
    });
    fsm_.Add(STOPPING, GOAL_COMPLETE, [this](FSM::Event const& event) -> FSM::State 
    {
        return Result(RESPONSE::SUCCESS);
    });

    // Set the operating mode to STOP by default, with the reference at initial position
    // so that when the speed ramps up we don't drift off position because of small forces and torques
    static ff_msgs::ControlCommand msg;
    msg.mode = ff_msgs::ControlCommand::MODE_STOP;
    msg.current.pose.position.x = acados_in.x0[0];
    msg.current.pose.position.y = acados_in.x0[1];
    msg.current.pose.position.z = acados_in.x0[2];
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(acados_out.x1[6], Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(acados_out.x1[7], Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(acados_out.x1[8], Eigen::Vector3f::UnitZ());
    msg.current.pose.orientation.x = q.x();
    msg.current.pose.orientation.y = q.y();
    msg.current.pose.orientation.z = q.z();
    msg.current.pose.orientation.w = q.w();
    msg.current.twist.linear.x = 0;
    msg.current.twist.linear.y = 0;
    msg.current.twist.linear.z = 0;
    msg.current.twist.angular.x = 0;
    msg.current.twist.angular.y = 0;
    msg.current.twist.angular.z = 0;
    msg.next = msg.current;

    SetCommand(ff_msgs::ControlCommand::MODE_STOP, msg);

    InitAcados(); 

    config_.AddFile("gnc.config");
    config_.AddFile("geometry.config");
    ReadParams();
    config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
        config_.CheckFilesUpdated(std::bind(&Ctl_MPC::ReadParams, this));}, false, true);

    pt_ctl_.Initialize("ctl");

    // Set subscribers
    ekf_sub_ = nh->subscribe(
        TOPIC_GNC_EKF, 1, &Ctl_MPC::EkfCallback, this);
    pose_sub_ = nh->subscribe(
        TOPIC_LOCALIZATION_POSE, 1, &Ctl_MPC::PoseCallback, this);
    twist_sub_ = nh->subscribe(
        TOPIC_LOCALIZATION_TWIST, 1, &Ctl_MPC::TwistCallback, this);
    inertia_sub_ = nh->subscribe(
        TOPIC_MOBILITY_INERTIA, 1, &Ctl_MPC::InertiaCallback, this);
    flight_mode_sub_ = nh->subscribe(
        TOPIC_MOBILITY_FLIGHT_MODE, 1, &Ctl_MPC::FlightModeCallback, this);
    command_sub_ = nh->subscribe(
        TOPIC_GNC_CTL_SETPOINT, 1, &Ctl_MPC::SetpointCallback, this);

    // Set publishers
    ctl_pub_ = nh->advertise<ff_msgs::FamCommand>(
        TOPIC_GNC_CTL_COMMAND, 5);
    traj_pub_ = nh->advertise<ff_msgs::ControlState>(
        TOPIC_GNC_CTL_TRAJ, 5);
    segment_pub_ = nh->advertise<ff_msgs::Segment>(
        TOPIC_GNC_CTL_SEGMENT, 5, true);

    // This timer will be used to throttle control to GNC
    timer_ = nh->createTimer(ros::Duration(0),
        &Ctl_MPC::TimerCallback, this, true, false);

    // TODO: DEFINE NAME FOR THIS TOPIC IN ff_util::ff_names (e.g. SERVICE_GNC_CTLMPC_ENABLE)
    enable_srv_ = nh->advertiseService("ctl_mpc/start", &Ctl_MPC::EnableServiceCallback, this);

    // Action client to accept control
    action_.SetGoalCallback(std::bind(&Ctl_MPC::GoalCallback, this, std::placeholders::_1));
    action_.SetPreemptCallback(std::bind(&Ctl_MPC::PreemptCallback, this));
    action_.SetCancelCallback(std::bind(&Ctl_MPC::CancelCallback, this));
    action_.Create(nh, ACTION_GNC_CTL_CONTROL);
}

Ctl_MPC::~Ctl_MPC() {}


// Subscribers, Services, Publishers
bool Ctl_MPC::EnableServiceCallback(std_srvs::SetBoolRequest&req, std_srvs::SetBoolResponse &response) 
{
    bool request = req.data;

    // Check if we are asking for the current state
    if (request == control_enabled_) {
        response.success = false;
        response.message = "Requested current state.";
        return true;
    }

    // Check required action
    response.success = true;
    control_enabled_ = request;
    if (control_enabled_ == false) {
        response.message = "MPC controller disabled.";
        NODELET_DEBUG_STREAM("MPC controller disabled.");
    } else {
        response.message = "MPC controller enabled.";
        NODELET_DEBUG_STREAM("MPC controller enabled.");
    }

    return true;
}

void Ctl_MPC::EkfCallback(const ff_msgs::EkfState::ConstPtr& state) {
    if(use_ground_truth_) return;

    mutex_cmd_msg_.lock();
    Eigen::Quaternionf q = msg_conversions::ros_to_eigen_quat(state->pose.orientation).cast<float>();
    double euler[3];
    QuatToEuler(q, euler);

    acados_in.x0[0] = state->pose.position.x;
    acados_in.x0[1] = state->pose.position.y;
    acados_in.x0[2] = state->pose.position.z;
    acados_in.x0[3] = state->velocity.x;
    acados_in.x0[4] = state->velocity.y;
    acados_in.x0[5] = state->velocity.z;
    acados_in.x0[6] = euler[0];
    acados_in.x0[7] = euler[1];
    acados_in.x0[8] = euler[2];
    acados_in.x0[9] = state->omega.x;
    acados_in.x0[10] = state->omega.y;
    acados_in.x0[11] = state->omega.z;
    mutex_cmd_msg_.unlock();

    // Check if Astrobee is dynamically stopped
    if (fsm_.GetState() == STOPPING) 
    {
        double v_x = state->velocity.x, v_y = state->velocity.y, v_z = state->velocity.z;
        double omega_x = state->omega.x, omega_y = state->omega.y, omega_z = state->omega.z;
        double v_magnitude = sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
        double omega_magnitude = sqrt(omega_x*omega_x + omega_y*omega_y + omega_z*omega_z);
        ROS_DEBUG_STREAM("Validating velocity when stopping:  |v|=  " << (float) v_magnitude
            << "  |omega|= " << (float) omega_magnitude);

        if ((v_magnitude <= sqrt(stopping_vel_thresh_squared_)) &&
            (omega_magnitude <= sqrt(stopping_omega_thresh_squared_))) 
        {
            ROS_INFO_STREAM("Stopping Complete: |v| <=  " << (float) sqrt(stopping_vel_thresh_squared_)
            << "  and |omega| <=  " << (float) sqrt(stopping_omega_thresh_squared_));
            fsm_.Update(GOAL_COMPLETE);
        }
    }

    // Advance control whenever pose is updated
    pt_ctl_.Tick();
    Step(state->header.stamp);
    pt_ctl_.Tock();
}

void Ctl_MPC::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& truth) {
    if(!use_ground_truth_) return;
    
    mutex_cmd_msg_.lock();
    Eigen::Quaternionf q = msg_conversions::ros_to_eigen_quat(truth->pose.orientation).cast<float>();
    double euler[3];
    QuatToEuler(q, euler);

    acados_in.x0[0] = truth->pose.position.x;
    acados_in.x0[1] = truth->pose.position.y;
    acados_in.x0[2] = truth->pose.position.z;
    acados_in.x0[6] = euler[0];
    acados_in.x0[7] = euler[1];
    acados_in.x0[8] = euler[2];
    mutex_cmd_msg_.unlock();

    // Advance control whenever pose is updated
    pt_ctl_.Tick();
    Step(truth->header.stamp);
    pt_ctl_.Tock();
}

void Ctl_MPC::TwistCallback(const geometry_msgs::TwistStamped::ConstPtr& truth) {
    if(!use_ground_truth_) return;

    mutex_cmd_msg_.lock();
    acados_in.x0[3] = truth->twist.linear.x;
    acados_in.x0[4] = truth->twist.linear.y;
    acados_in.x0[5] = truth->twist.linear.z;
    acados_in.x0[9] = truth->twist.angular.x;
    acados_in.x0[10] = truth->twist.angular.y;
    acados_in.x0[11] = truth->twist.angular.z;
    mutex_cmd_msg_.unlock();
}

void Ctl_MPC::SetpointCallback(const ff_msgs::ControlState::ConstPtr& command) {
    // Package up a command using the curren timestamp
    ff_msgs::ControlCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.mode = ff_msgs::ControlCommand::MODE_NOMINAL;
    msg.current = *command;
    msg.current.when = ros::Time::now();
    msg.next = msg.current;
    // Send the command
    if (!SetCommand(msg.mode, msg))
        ROS_WARN("Could not send direct control command");

    setpoint_received_ = true;
}

void Ctl_MPC::InertiaCallback(const geometry_msgs::InertiaStamped::ConstPtr& inertia) {
    std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
    for (int stage = 0; stage <=N; stage++) {
        acados_in.params[stage][24] = inertia->inertia.m;
        acados_in.params[stage][25] = inertia->inertia.ixx;
        acados_in.params[stage][26] = inertia->inertia.iyy;
        acados_in.params[stage][27] = inertia->inertia.izz;
    }
    inertia_received_ = true;
}

void Ctl_MPC::FlightModeCallback(const ff_msgs::FlightMode::ConstPtr& mode) {
    std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
    for (int stage = 0; stage <=N; stage++) {
        acados_in.params[stage][12] = mode->tolerance_pos;
        acados_in.params[stage][13] = mode->tolerance_pos;
        acados_in.params[stage][14] = mode->tolerance_pos;
        acados_in.params[stage][15] = mode->tolerance_vel;
        acados_in.params[stage][16] = mode->tolerance_vel;
        acados_in.params[stage][17] = mode->tolerance_vel;
        acados_in.params[stage][18] = mode->tolerance_att;
        acados_in.params[stage][19] = mode->tolerance_att;
        acados_in.params[stage][20] = mode->tolerance_att;
        acados_in.params[stage][21] = mode->tolerance_omega;
        acados_in.params[stage][22] = mode->tolerance_omega;
        acados_in.params[stage][23] = mode->tolerance_omega;
    }
    flight_enabled_ = (mode->speed > 0 ? mode->control_enabled : false);
}

void Ctl_MPC::PublishControl(ros::Time curr_time) 
{
    static ff_msgs::FamCommand cmd_msg_;
    cmd_msg_.header.stamp = curr_time;
    cmd_msg_.header.frame_id = "body";
    cmd_msg_.wrench.force.x = acados_out.u0[0];
    cmd_msg_.wrench.force.y = acados_out.u0[1];
    cmd_msg_.wrench.force.z = acados_out.u0[2];
    cmd_msg_.wrench.torque.x = acados_out.u0[3];
    cmd_msg_.wrench.torque.y = acados_out.u0[4];
    cmd_msg_.wrench.torque.z = acados_out.u0[5];
    cmd_msg_.position_error.x = acados_in.params[1][0]-acados_out.x1[0];
    cmd_msg_.position_error.y = acados_in.params[1][1]-acados_out.x1[1];
    cmd_msg_.position_error.z = acados_in.params[1][2]-acados_out.x1[2];
    cmd_msg_.attitude_error.x = acados_in.params[1][6]-acados_out.x1[6];
    cmd_msg_.attitude_error.y = acados_in.params[1][7]-acados_out.x1[7];
    cmd_msg_.attitude_error.z = acados_in.params[1][8]-acados_out.x1[8];
    cmd_msg_.status = acados_out.status;
    cmd_msg_.control_mode = mode_;
    ctl_pub_.publish(cmd_msg_);
}

// Action client
void Ctl_MPC::GoalCallback(ff_msgs::ControlGoalConstPtr const& goal) {
    NODELET_INFO_STREAM("GoalCallback: new control goal received");

    // What have we been instructed to do?
    switch (goal->command) {
        // IDLE - returns instantly
        case ff_msgs::ControlGoal::IDLE:
            NODELET_DEBUG_STREAM("Received IDLE command");
            if (!SetCommand(ff_msgs::ControlCommand::MODE_IDLE))
                Result(RESPONSE::CONTROL_FAILED);
            Result(RESPONSE::SUCCESS);
            return;

        // STOP - returns instantly
        case ff_msgs::ControlGoal::STOP:
            NODELET_DEBUG_STREAM("Received STOP command");
            if (!SetCommand(ff_msgs::ControlCommand::MODE_STOP))
                Result(RESPONSE::CONTROL_FAILED);
            fsm_.Update(GOAL_STOP);
            return;

        // NOMINAL - waits until completion
        case ff_msgs::ControlGoal::NOMINAL: {
            NODELET_DEBUG_STREAM("Received NOMINAL command with segment...");
            // NODELET_DEBUG_STREAM(goal->segment);
            if (goal->segment.empty()) {
                Result(RESPONSE::EMPTY_SEGMENT);
                return;
            }
            // Save the segment internally
            mutex_segment_.lock();
            segment_ = goal->segment;       // Copy over the segment to be processed
            setpoint_ = segment_.begin();   // We are processing the first setpoint
            mutex_segment_.unlock();
            // Publish the segment to used by the sentinel (and perhaps others)
            static ff_msgs::Segment msg;
            msg.segment = goal->segment;
            segment_pub_.publish(msg);

            // Start executing and return
            fsm_.Update(GOAL_NOMINAL);
            return;
            }

        // No other control mode makes sense
        default:
        break;
    }

    Result(RESPONSE::INVALID_COMMAND);
}

void Ctl_MPC::PreemptCallback() {
    ROS_INFO_STREAM("PreemptCallback called");
    fsm_.Update(GOAL_CANCEL);
}

void Ctl_MPC::CancelCallback() {
    ROS_INFO_STREAM("CancelCallback called");
    fsm_.Update(GOAL_CANCEL);
}

// Helpers
void Ctl_MPC::InitAcados() {
    capsule = astrobee_acados_create_capsule(); 
    int status = astrobee_acados_create(capsule);
    if(status != 0) {
        NODELET_DEBUG_STREAM("astrobee_acados_create() returned status" << status << ". Exiting.\n");
        exit(1);
    }

    nlp_config = astrobee_acados_get_nlp_config(capsule);
    nlp_dims = astrobee_acados_get_nlp_dims(capsule);
    nlp_in = astrobee_acados_get_nlp_in(capsule);
    nlp_out = astrobee_acados_get_nlp_out(capsule);

    // TODO: Allow the config file to load the Q and R parameters. For now just initialise with value 1
    for (int stage = 0; stage <=N; stage++) {
        // diag(Q)
        acados_in.params[stage][28] = 1;
        acados_in.params[stage][29] = 1;
        acados_in.params[stage][30] = 1;
        acados_in.params[stage][31] = 1;
        acados_in.params[stage][32] = 1;
        acados_in.params[stage][33] = 1;
        acados_in.params[stage][34] = 1;
        acados_in.params[stage][35] = 1;
        acados_in.params[stage][36] = 1;
        acados_in.params[stage][37] = 1;
        acados_in.params[stage][38] = 1;
        acados_in.params[stage][39] = 1;

        //daig(R)
        acados_in.params[stage][40] = 1;
        acados_in.params[stage][41] = 1;
        acados_in.params[stage][42] = 1;
        acados_in.params[stage][43] = 1;
        acados_in.params[stage][44] = 1;
        acados_in.params[stage][45] = 1;
    }
}

void Ctl_MPC::QuatToEuler(const Eigen::Quaternionf& q, double* angles) 
{
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
}

void Ctl_MPC::UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    std::string str = "UNKNOWN";
    switch (event) {
    case GOAL_NOMINAL:            str = "GOAL_NOMINAL";       break;
    case GOAL_COMPLETE:           str = "GOAL_COMPLETE";      break;
    case GOAL_CANCEL:             str = "GOAL_CANCEL";        break;
    case GOAL_STOP:               str = "GOAL_STOP";          break;
    }
    NODELET_DEBUG_STREAM("Received event " << str);
    // Debug state changes
    switch (state) {
    case WAITING:                 str = "WAITING";            break;
    case STOPPING:                str = "STOPPING";           break;
    case NOMINAL:                 str = "NOMINAL";            break;
    }
    NODELET_DEBUG_STREAM("State changed to " << str);
}

FSM::State Ctl_MPC::Result(int32_t response) {
    NODELET_DEBUG_STREAM("Control action completed with code " << response);
    // Stop the platform and
    if (!SetCommand(ff_msgs::ControlCommand::MODE_STOP))
        NODELET_ERROR("Could not stop the platform at end of control action");
    // Send a response
    static ff_msgs::ControlResult result;
    result.response = response;
    mutex_segment_.lock();
    if (!segment_.empty()) {
        result.segment = segment_;
        result.index = std::distance(setpoint_, segment_.begin());
    }

    if (response == RESPONSE::SUCCESS) {
        NODELET_INFO_STREAM("Result: Control succeeded, result sent to client");
        action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    } else if (response == RESPONSE::PREEMPTED) {
        NODELET_INFO_STREAM("Result: Control Preempted, result sent to client");
        action_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
    } else {
        NODELET_INFO_STREAM("Result: Control action aborted, result sent to client");
        action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
    }

    // Publish an empty segment to notify that nominal mode is over
    static ff_msgs::Segment msg;
    segment_pub_.publish(msg);
    // Clear local variables
    timer_.stop();
    segment_.clear();
    setpoint_ = segment_.end();
    mutex_segment_.unlock();
    // Always return to the waiting state
    return WAITING;
}

// When the previous setpoint is sent a timer is created, which fires on the
// edge of the next setpoint. This essentially provides us with a non-blocking
// method for feeding control.
void Ctl_MPC::TimerCallback(const ros::TimerEvent& event) {
    static ff_msgs::ControlCommand msg;
    msg.mode = ff_msgs::ControlCommand::MODE_NOMINAL;
    do {
        mutex_segment_.lock();
        // Get the index of the current setpoint
        size_t idx = std::distance(segment_.begin(), setpoint_);
        // If the setpoint is pointing to the last segment, that's it!
        if (setpoint_ == segment_.end()) {
            mutex_segment_.unlock();
            NODELET_DEBUG_STREAM("Final setpoint " << idx);
            return fsm_.Update(GOAL_COMPLETE);
        }
        // Otherwise, we have at least one more setpoint
        msg.current = *setpoint_;
        // If we only have one setpoint, just copy the first one
        if (++setpoint_ == segment_.end()) {
            mutex_segment_.unlock();
            NODELET_DEBUG_STREAM("Penultimate setpoint " << idx);
            msg.next = msg.current;
        break;
        // We have two valid set points - if the times are equal, then we will
        // treat the last setpoint as the valid one and move on...
        } else {
        msg.next = *setpoint_;
        if (msg.current.when != msg.next.when) {
            mutex_segment_.unlock();
            NODELET_DEBUG_STREAM("Progressing to setpoint " << idx);
            break;
        }
        }
        mutex_segment_.unlock();
        NODELET_DEBUG_STREAM("Skipping setpoint " << idx);
    } while (msg.current.when == msg.next.when);
    // Send the control
    NODELET_DEBUG_STREAM(msg);
    if (!SetCommand(ff_msgs::ControlCommand::MODE_NOMINAL, msg)) {
        Result(RESPONSE::CONTROL_FAILED);
        return;
    }
    // Set off a timer to wait until the next setpoint
    mutex_segment_.lock();
    ros::Duration delta = msg.next.when - ros::Time::now();
    timer_.stop();
    timer_.setPeriod(delta, true);
    timer_.start();
    mutex_segment_.unlock();
    NODELET_DEBUG_STREAM("Sleep: " << delta);
}

bool Ctl_MPC::Step(ros::Time curr_time) 
{
    if (!inertia_received_) {
        NODELET_DEBUG_STREAM_THROTTLE(10, "GNC step waiting for inertia");
        return false;
    }

    if (!flight_enabled_ || !control_enabled_) {
        NODELET_DEBUG_STREAM_THROTTLE(10, "GNC control disabled in flight mode");
        return false;
    }

    if (!setpoint_received_) {
        NODELET_DEBUG_STREAM_THROTTLE(10, "Waiting for initial setpoint to arrive");
        return false;
    }

    std::lock_guard<std::mutex> cmd_lock(mutex_cmd_msg_);
    GetCommand(curr_time);

    ComputeControl();

    PublishControl(curr_time);
    PublishTrajectory(curr_time);
    PublishSetpoint(curr_time);

    // Succesfull step
    return true;
}

void Ctl_MPC::PublishTrajectory(ros::Time curr_time)
{
    static ff_msgs::ControlState current;
    current.when = curr_time;
    current.pose.position.x = acados_out.x1[0];
    current.pose.position.y = acados_out.x1[1];
    current.pose.position.z = acados_out.x1[2];
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(acados_out.x1[6], Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(acados_out.x1[7], Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(acados_out.x1[8], Eigen::Vector3f::UnitZ());
    current.pose.orientation.x = q.x();
    current.pose.orientation.y = q.y();
    current.pose.orientation.z = q.z();
    current.pose.orientation.w = q.w();
    current.twist.linear.x = acados_out.x1[3];
    current.twist.linear.y = acados_out.x1[4];
    current.twist.linear.z = acados_out.x1[5];
    current.twist.angular.x = acados_out.x1[9];
    current.twist.angular.y = acados_out.x1[10];
    current.twist.angular.z = acados_out.x1[11];
    traj_pub_.publish(current);
}

void Ctl_MPC::PublishSetpoint(ros::Time curr_time)
{
    if ((fsm_.GetState() == NOMINAL) || (fsm_.GetState() == STOPPING)) {
        std::lock_guard<std::mutex> lock(mutex_segment_);
        static ff_msgs::ControlFeedback feedback;
        feedback.setpoint.when = curr_time;
        feedback.setpoint.pose.position.x = acados_out.x1[0];
        feedback.setpoint.pose.position.y = acados_out.x1[1];
        feedback.setpoint.pose.position.z = acados_out.x1[2];
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(acados_out.x1[6], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(acados_out.x1[7], Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(acados_out.x1[8], Eigen::Vector3f::UnitZ());
        feedback.setpoint.pose.orientation.x = q.x();
        feedback.setpoint.pose.orientation.y = q.y();
        feedback.setpoint.pose.orientation.z = q.z();
        feedback.setpoint.pose.orientation.w = q.w();
        feedback.setpoint.twist.linear.x = acados_out.x1[3];
        feedback.setpoint.twist.linear.y = acados_out.x1[4];
        feedback.setpoint.twist.linear.z = acados_out.x1[5];
        feedback.setpoint.twist.angular.x = acados_out.x1[9];
        feedback.setpoint.twist.angular.y = acados_out.x1[10];
        feedback.setpoint.twist.angular.z = acados_out.x1[11];
        feedback.index = std::distance(setpoint_, segment_.begin());
        // Sometimese segments arrive with a first setpoint that has a time stamp
        // sometime in the future. In this case we will be in STOPPED mode until
        // the callback timer sets the CMC mode to NOMINAL. In the interim the
        // errors being sent back from the controller make no sense.
        switch (mode_) {
            case ff_msgs::ControlCommand::MODE_NOMINAL:
                feedback.error_position = sqrt(pow(acados_in.params[1][0]-acados_out.x1[0],2)+pow(acados_in.params[1][1]-acados_out.x1[1],2)+pow(acados_in.params[1][2]-acados_out.x1[2],2));
                feedback.error_attitude = sqrt(pow(acados_in.params[1][6]-acados_out.x1[6],2)+pow(acados_in.params[1][7]-acados_out.x1[7],2)+pow(acados_in.params[1][8]-acados_out.x1[8],2));
                feedback.error_velocity = sqrt(pow(acados_in.params[1][3]-acados_out.x1[3],2)+pow(acados_in.params[1][4]-acados_out.x1[4],2)+pow(acados_in.params[1][5]-acados_out.x1[5],2));
                feedback.error_omega    = sqrt(pow(acados_in.params[1][9]-acados_out.x1[9],2)+pow(acados_in.params[1][10]-acados_out.x1[10],2)+pow(acados_in.params[1][11]-acados_out.x1[11],2));
                break;
            default:
                feedback.error_position = 0;
                feedback.error_attitude = 0;
                feedback.error_velocity = 0;
                feedback.error_omega    = 0;
        break;
        }
        action_.SendFeedback(feedback);
    }
}

void Ctl_MPC::ComputeControl() {
    // Set initial state
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

    // Set params for each stage (xref, xlim, m and inertia) and use for warmstart
    for(int stage=0; stage<=N; stage++) {
        astrobee_acados_update_params(capsule, stage, acados_in.params[stage], NP);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, stage, "x", acados_in.params[stage]);
    }
    // warmstart x0 as x0
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, 0, "x", acados_in.x0);

    // Solve
    acados_out.status = astrobee_acados_solve(capsule);
    if (acados_out.status != ACADOS_SUCCESS)
    {
        NODELET_DEBUG_STREAM("SOLVER FAILED: " << acados_out.status);
    }
    else
    {
        NODELET_DEBUG_STREAM("ACADOS SUCCESS, CONTROL INPUT: ");
        d_print_exp_tran_mat( NU, 1, acados_out.u0, NU );
    }

    // Extract control from output. Saved to our acados_out struct
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &acados_out.u0);
    // Extract x1 for feedback
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", &acados_out.x1);
}

bool Ctl_MPC::SetCommand(uint8_t const mode, ff_msgs::ControlCommand const& poseVel) {
    std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
    mode_ = mode;
    if (mode != ff_msgs::ControlCommand::MODE_NOMINAL)
        return true;

    command_ = poseVel;
    
    return true;
}

float Ctl_MPC::GetCommand(ros::Time tim) {
    auto* curr = &command_.next;
    if (curr->when > tim)
        curr = &command_.current;

    Eigen::Quaternionf q = msg_conversions::ros_to_eigen_quat(curr->pose.orientation).cast<float>();
    double euler[3];
    QuatToEuler(q, euler);

    for (int stage = 0; stage <=N; stage++) {
        acados_in.params[stage][0] = curr->pose.position.x;
        acados_in.params[stage][1] = curr->pose.position.y;
        acados_in.params[stage][2] = curr->pose.position.z;
        acados_in.params[stage][3] = curr->twist.linear.x;
        acados_in.params[stage][4] = curr->twist.linear.y;
        acados_in.params[stage][5] = curr->twist.linear.z;
        acados_in.params[stage][6] = euler[0];
        acados_in.params[stage][7] = euler[1];
        acados_in.params[stage][8] = euler[2];
        acados_in.params[stage][9] = curr->twist.angular.x;
        acados_in.params[stage][10] = curr->twist.angular.y;
        acados_in.params[stage][11] = curr->twist.angular.z;
    }
    
    return (tim - curr->when).toSec();
}

// Chainload the readparam call
void Ctl_MPC::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }
  if (!config_.GetBool("tun_debug_ctl_use_truth", &use_ground_truth_))
    ROS_FATAL("tun_debug_ctl_use_truth not specified.");
  // Set linear- and angular velocity threshold for ekf
  // to decide that Astrobee is dynamically stopped.
  if (!config_.GetReal("tun_ctl_stopping_vel_thresh", &stopping_vel_thresh_squared_))
    ROS_FATAL("tun_ctl_stopping_vel_thresh not specified.");
  if (!config_.GetReal("tun_ctl_stopping_omega_thresh", &stopping_omega_thresh_squared_))
    ROS_FATAL("tun_ctl_stopping_omega_thresh not specified.");
}

std::string Ctl_MPC::getName() {return name_;}

}