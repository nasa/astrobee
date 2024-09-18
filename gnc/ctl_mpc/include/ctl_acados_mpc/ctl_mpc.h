#ifndef CTL_MPC_H
#define CTL_MPC_H

// Acados Stuff
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_astrobee.h"

#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// ROS Stuff
#include <ros/ros.h>

#include <geometry_msgs/InertiaStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nodelet/nodelet.h>

// Astrobee Stuff
#include <ff_msgs/ControlCommand.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/ControlAction.h>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/SetFloat.h>
#include <ff_msgs/Segment.h>

#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/perf_timer.h>

#include <msg_conversions/msg_conversions.h>
#include <std_srvs/SetBool.h>

namespace ctl_mpc {

using FSM = ff_util::FSM;

class Ctl_MPC 
{

private:

    // -------------------------------------- //
    //              ACADOS STUFF              //
    // -------------------------------------- //
    #define N      ASTROBEE_N
    #define NX     ASTROBEE_NX
    #define NU     ASTROBEE_NU
    #define NP     ASTROBEE_NP

    astrobee_solver_capsule* capsule;

    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;

    struct solver_output{
        double u0[NU];
        double x1[NX];
        int status;
    };

    struct solver_input{
        double x0[NX];
        double params[N+1][NP];
    };

    solver_input acados_in;
    solver_output acados_out;
    int acados_status;

    // Initialise the acados solver
    void InitAcados();

    // -------------------------------------- //
    //                ASTROBEE                //
    // -------------------------------------- //
    #define MAX_LATENCY 1.0

    FSM fsm_;

    // Declaration of all possible states
    enum : FSM::State {
        WAITING        = 1,
        NOMINAL        = 2,
        STOPPING       = 3
    };

    // Declaration of all possible events
    enum : FSM::Event {
        GOAL_COMPLETE  = (1<<0),
        GOAL_NOMINAL   = (1<<1),
        GOAL_CANCEL    = (1<<2),
        GOAL_STOP      = (1<<3)
    };
    
    // Terminate execution in STOP mode
    FSM::State Result(int32_t response);

    ros::Subscriber twist_sub_, pose_sub_, ekf_sub_, command_sub_, flight_mode_sub_, inertia_sub_;
    ros::Publisher ctl_pub_, segment_pub_, traj_pub_;
    ros::ServiceServer enable_srv_;

    config_reader::ConfigReader config_;
    ros::Timer config_timer_;

    ff_util::PerfTimer pt_ctl_;

    ros::Rate rate_ = ros::Rate(5);
    ros::Timer timer_;

    bool control_enabled_;
    bool use_ground_truth_;
    bool flight_enabled_;
    bool inertia_received_;
    bool setpoint_received_;

    float stopping_vel_thresh_squared_;
    float stopping_omega_thresh_squared_;

    std::string name_;

    uint8_t mode_;

    std::mutex mutex_segment_, mutex_cmd_msg_;

    ff_util::FreeFlyerActionServer<ff_msgs::ControlAction> action_;
    ff_util::Segment segment_;
    ff_util::Segment::iterator setpoint_;
    ff_msgs::ControlCommand command_;

    // ----- ROS Communication ----- //

    // Enable ctl_mpc instead of ctl
    bool EnableServiceCallback(std_srvs::SetBoolRequest&req, std_srvs::SetBoolResponse &response);

    // Called when pose estimate is available
    void EkfCallback(const ff_msgs::EkfState::ConstPtr& state);

    // Called when new ground truth pose is available
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& truth); 

    // Called when new ground truth twist is available
    void TwistCallback(const geometry_msgs::TwistStamped::ConstPtr& truth);

    // Command GNC directly, bypassing the action-based interface
    void SetpointCallback(const ff_msgs::ControlState::ConstPtr& command);

    // Called when there is new inertia info
    void InertiaCallback(const geometry_msgs::InertiaStamped::ConstPtr& inertia);

    // Called when the choreographer updates the flight mode
    void FlightModeCallback(const ff_msgs::FlightMode::ConstPtr& mode);

    // Publishes newly computed control
    void PublishControl(ros::Time curr_time);

    // Publishes the trajectory message
    void PublishTrajectory(ros::Time curr_time);

    // Publishes the current setpoint
    void PublishSetpoint(ros::Time curr_time);

    // Called when a new goal arrives
    void GoalCallback(ff_msgs::ControlGoalConstPtr const& control_goal);

    // Called when a goal is preempted
    void PreemptCallback();

    // Called when a goal is cancelled
    void CancelCallback();
    
    // Used to feed segments
    void TimerCallback(const ros::TimerEvent& event);


    // ----- FUNCTIONALITY ----- //

    // Loops through all steps for control every timestep
    bool Step(ros::Time curr_time);

    // Computes the control using acados
    void ComputeControl();

    // Sets a new command (mode + reference)
    bool SetCommand(uint8_t const mode, 
                    ff_msgs::ControlCommand const& poseVel = ff_msgs::ControlCommand());

    // Gets the command (to have the correct references)
    float GetCommand(ros::Time tim);

    // Helper function to do quat-euler conversion
    void QuatToEuler(const Eigen::Quaternionf& q, double* angles);

    // Make sure data is not older than max latency
    bool CheckDataValidity();

    // Called when internal state changes
    void UpdateCallback(ff_util::FSM::State const& state, ff_util::FSM::Event const& event);

    // Read the control parameters from the LUA config file
    void ReadParams(void);

public:

    Ctl_MPC(ros::NodeHandle* nh, std::string const& name);

    ~Ctl_MPC();

    std::string getName();

};

}

#endif