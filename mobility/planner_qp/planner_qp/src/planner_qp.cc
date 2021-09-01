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

#include <ff_util/config_server.h>
#include <ff_util/ff_nodelet.h>

// For plugin loading
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// For the planner implementation API
#include <choreographer/planner.h>

// Keepout zones for the planner
#include <jsonloader/keepout.h>

// Trajectory solving files
#include <traj_opt_basic/types.h>
#include <traj_opt_pro/nonlinear_trajectory.h>
#include <traj_opt_ros/ros_bridge.h>

#include <decomp_util/ellipse_decomp.h>
#include <jps3d/planner/jps_3d_util.h>

#include <mapper/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <functional>

#define DEBUG false
#define OUTPUT_DEBUG NODELET_DEBUG_STREAM
// #define OUTPUT_DEBUG ROS_ERROR_STREAM
/**
 * \ingroup planner
 */

namespace planner_qp {

using RESPONSE = ff_msgs::PlanResult;

class Planner : public planner::PlannerImplementation {
 public:
  Planner() : planner::PlannerImplementation("qp", "QP planner") {}
  ~Planner() {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // planner configuration params
  bool faceforward_;      // Face-forward trajectory?
  bool check_obstacles_;  // Perform obstacle checking
  bool uniform_time_;
  bool time_optiization_;

  float desired_vel_;    // Soft limit on velocity
  float desired_accel_;  // Soft limit on accel
  float desired_omega_;  // Soft limit on omega
  float desired_alpha_;  // Soft limit on alpha
  float control_rate_;   // Control frequency
  double max_time_;      // Max generation time
  double traj_it_{1.0};
  double gap_threshold_;
  int max_iterations_;

  ros::Publisher cloud_pub_, viz_pub_;
  ros::NodeHandle *nh_;
  ros::Subscriber pcl_sub_;

  std::vector<traj_opt_msgs::Trajectory> traj_history_;

 protected:
  bool InitializePlanner(ros::NodeHandle *nh) {
    // Grab some configuration parameters for this node from the LUA config
    cfg_.Initialize(GetPrivateHandle(), "mobility/planner_qp.config");
    cfg_.Listen(boost::bind(&Planner::ReconfigureCallback, this, _1));
    cfg_mapper_.Initialize(GetPrivateHandle(), "mobility/mapper.config");
    cfg_mapper_.Listen(boost::bind(&Planner::ReconfigureCallbackMapper, this, _1));

    // Setup a timer to forward diagnostics
    timer_d_ =
        nh->createTimer(ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
                        &Planner::DiagnosticsCallback, this, false, true);
    nh_ = nh;

    viz_pub_ = nh->advertise<visualization_msgs::MarkerArray>(
        "/mob/planner_qp/safe_flight_cooridor", 5, true);

    // Setup a timer to animate
    timer_a_ =
        nh->createTimer(ros::Duration(ros::Rate(10.0)),
                        &Planner::AnimateTimer, this, false, true);

    pcl_sub_ = nh->subscribe(TOPIC_MAPPER_OCTOMAP_INFLATED_CLOUD, 5,
                             &Planner::map_callback, this);

    // This timer will be used to send feedback to clients while planning
    timer_fb_ = nh->createTimer(ros::Duration(ros::Rate(1.5*DEFAULT_DIAGNOSTICS_RATE)),
      &Planner::SendFeedback, this, false, false);

    // cloud_pub_ = nh->advertise<pcl::PointCloud<pcl::PointXYZ> >
    // ("obs_points", 5, true);
    // Success
    return true;
  }

  // Send useless feedback to client
  void SendFeedback(ros::TimerEvent const& event) {
    ff_msgs::PlanFeedback fb;
    PlanFeedback(fb);
  }

  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    cfg_.Reconfigure(config);

    if (!cfg_.Get<double>("iteration_replay", traj_it_)) traj_it_ = 1.0;
    return true;
  }

  bool ReconfigureCallbackMapper(dynamic_reconfigure::Config &config) {
    cfg_.Reconfigure(config);
    return true;
  }

  void AnimateTimer(ros::TimerEvent const& event) {
    static uint num_it = 0;
    if (traj_history_.size() > 0 &&
        TrajRosBridge::are_subscribers("/mob/planner_qp/trajectory_debug")) {
      uint curr = num_it++ % traj_history_.size();
      TrajRosBridge::publish_msg(traj_history_.at(curr), "world",
        nh_->getNamespace() + std::string("/mob/planner_qp/trajectory_debug"));
    }
  }

  // sorts vertices's to produce triangularization
  traj_opt::Vec3Vec sort_verticies(const traj_opt::Vec3Vec &in) {
    using traj_opt::Vec3Vec;
    using traj_opt::Vec3;

    Vec3 cen = Vec3::Zero();
    for (auto &v : in) cen += v;

    cen /= static_cast<decimal_t>(in.size());

    Vec3 v0 = (in.front() - cen).normalized();
    Vec3 normal = Vec3::UnitZ();
    for (uint i = 0; i < in.size(); i++) {
      Vec3 vf = (in.back() - cen).normalized();
      if (std::abs(v0.dot(vf) > 1e-3)) {
        normal = v0.cross(vf).normalized();
        break;
      }
    }

    Vec3Vec out = in;

    // this is horrendous to read
    auto angle = [](const Vec3 &v, const Vec3 &v0_, const Vec3 &cen_,
                    const Vec3 &normal_) {
      return std::atan2((v - cen_).normalized().cross(v0_).dot(normal_),
                        (v - cen_).normalized().dot(v0_));
    };
    auto cost = [&cen, &v0, &normal, &angle](const Vec3 &a, const Vec3 &b) {
      return angle(a, v0, cen, normal) < angle(b, v0, cen, normal);
    };

    std::sort(out.begin(), out.end(), cost);

    return out;
  }

  void viz_cooridor(
      const std::vector<std::pair<traj_opt::MatD, traj_opt::VecD> > &cons_3d) {
    visualization_msgs::MarkerArray viz;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    clear.ns = "sfc_viz";

    decimal_t d_col = 1.0 / cons_3d.size();
    int c = 0;
    for (auto &con : cons_3d) {
      visualization_msgs::Marker poly;
      poly.header.frame_id = "world";
      poly.ns = "sfc_viz";
      poly.color.r = d_col * static_cast<decimal_t>(c);
      poly.id = c++;
      poly.color.a = 0.5;
      poly.color.g = 1.0;
      poly.color.b = 0.5;
      poly.type = visualization_msgs::Marker::TRIANGLE_LIST;
      poly.scale.x = 1.0;
      poly.scale.y = 1.0;
      poly.scale.z = 1.0;
      poly.pose.orientation.w = 1.0;

      auto &A = con.first;
      auto &b = con.second;

      // ROS_ERROR_STREAM("A " << con.first);
      // ROS_ERROR_STREAM("b " << b);

      // traj_opt::MatD A = traj_opt::MatD::Zero(6,3);
      // A.block<3,3>(0,0)= traj_opt::Mat3::Identity();
      // A.block<3,3>(3,0)= -traj_opt::Mat3::Identity();
      // traj_opt::VecD b = traj_opt::VecD::Ones(6,1)*static_cast<decimal_t>(c);

      // ROS_ERROR_STREAM("A true " << A);

      for (int i = 0; i < A.rows(); i++) {
        traj_opt::Vec3Vec face_points;

        traj_opt::Mat3 subA = traj_opt::Mat3::Zero();
        traj_opt::Vec3 subb = traj_opt::Vec3::Zero();
        for (int k = 0; k < A.rows(); k++) {
          for (int j = 0; j < A.rows(); j++) {
            if (j != i && k != i && j < k) {
              // silly combinatorial algo
              subA.block<1, 3>(0, 0) = A.block<1, 3>(i, 0);
              subA.block<1, 3>(1, 0) = A.block<1, 3>(j, 0);
              subA.block<1, 3>(2, 0) = A.block<1, 3>(k, 0);
              subb(0) = b(i);
              subb(1) = b(j);
              subb(2) = b(k);
              // traj_opt::Vec3 x = subA.inverse()*subb;
              // traj_opt::Vec3 x = subA.ldlt().solve(subb);
              traj_opt::Vec3 x = subA.colPivHouseholderQr().solve(subb);
              // ROS_ERROR_STREAM("A sub " << subA);
              // ROS_ERROR_STREAM("b sub " << subb.transpose());

              if (!std::isnan(x.sum())) {  //} && subA.determinant() > 1e-7)  {
                traj_opt::VecD err = A.block(0, 0, A.rows(), 3) * x - b;
                traj_opt::VecD err2 = subA * x - subb;
                if (err.maxCoeff() <= 1e-3 && err2.norm() < 1e-3) {
                  // if(err2.norm() < 1e-3) {
                  // ROS_ERROR_STREAM("sub error mains " << err.maxCoeff() );
                  face_points.push_back(x);
                } else {
                  // ROS_ERROR_STREAM("err t " << err.transpose());
                }
              }
            }
          }
        }
        // ROS_ERROR_STREAM("face points " << face_points.size());
        traj_opt::Vec3 centroid = traj_opt::Vec3::Zero();
        for (auto &v : face_points) centroid += v;
        centroid /= static_cast<decimal_t>(face_points.size());
        traj_opt::VecD err = A.block(0, 0, A.rows(), 3) * centroid - b;

        // add to triangle, reject interior polyies
        // ROS_ERROR_STREAM("max coeff " << err.maxCoeff());
        if (face_points.size() >= 3) {  //&& err.maxCoeff() < -0.1) {
          traj_opt::Vec3Vec sorted_points = sort_verticies(face_points);
          for (uint f = 0; f < face_points.size() - 2; f++) {
            geometry_msgs::Point pt;
            pt.x = sorted_points.front()(0);
            pt.y = sorted_points.front()(1);
            pt.z = sorted_points.front()(2);
            poly.points.push_back(pt);
            for (int l = 1; l < 3; l++) {
              // geometry_msgs::Point pt;
              pt.x = sorted_points.at(l + f)(0);
              pt.y = sorted_points.at(l + f)(1);
              pt.z = sorted_points.at(l + f)(2);
              poly.points.push_back(pt);
            }
          }
        }
      }
      viz.markers.push_back(poly);
    }

    viz_pub_.publish(viz);
  }

  void DiagnosticsCallback(const ros::TimerEvent &event) {
    SendDiagnostics(cfg_.Dump());
  }

  void PlanCallback(ff_msgs::PlanGoal const &goal) override {
    ff_msgs::PlanResult plan_result;
    timer_fb_.start();
    // get important vars
    const std::vector<geometry_msgs::PoseStamped> &states = goal.states;
    if (!LoadActionParams(goal)) {
      ROS_ERROR_STREAM("Planner params are bad");
      plan_result.response = RESPONSE::BAD_ARGUMENTS;
      PlanResult(plan_result);
      timer_fb_.stop();
      return;
    }

    OUTPUT_DEBUG("PlannerQP: Face forward: " << faceforward_);
    // get error quaternion
    tf::Quaternion start_orientation, end_orientation;
    tf::quaternionMsgToTF(states.front().pose.orientation, start_orientation);
    tf::quaternionMsgToTF(states.back().pose.orientation, end_orientation);
    tf::Quaternion error_quat = start_orientation.inverse() * end_orientation;
    error_quat.normalize();  // yes, this is needed badly to fix error when axis
                             // returned is 0
    double error_angle = error_quat.getAngle();
    if (error_angle > M_PI) error_angle -= 2 * M_PI;

    for (auto &s : states) {
      OUTPUT_DEBUG("PlannerQP: state: " << s.pose.position.x << " "
                                        << s.pose.position.y << " "
                                        << s.pose.position.z);
      OUTPUT_DEBUG("PlannerQP: stateo: "
                   << s.pose.orientation.w << " " << s.pose.orientation.x << " "
                   << s.pose.orientation.y << " " << s.pose.orientation.z);
    }

    // pack into  qp_traj_opt format
    traj_opt::Vec4 start_eig, end_eig;
    start_eig << states.front().pose.position.x, states.front().pose.position.y,
        states.front().pose.position.z, 0;
    end_eig << states.back().pose.position.x, states.back().pose.position.y,
        states.back().pose.position.z, error_angle;
    start_orientation_ = start_orientation;
    axis_ = error_quat.getAxis();
    OUTPUT_DEBUG("PlannerQP: Axis " << axis_.x() << " " << axis_.y() << " "
                                    << axis_.z());

    // set z to be the same if in granite lab
    bool use_2d;
    if (!cfg_.Get<bool>("two_d", use_2d)) use_2d = true;
    if (use_2d) end_eig(2) = start_eig(2);

    // try to optimize trajectory, return on failure
    if (!generate_trajectory(start_eig, end_eig, &plan_result)) {
      PlanResult(plan_result);
      timer_fb_.stop();
      return;
    }


    // Check face forward
    if (!calculate_time_scale()) {
      plan_result.response = RESPONSE::BAD_ARGUMENTS;
      PlanResult(plan_result);
      timer_fb_.stop();
      return;
    }

    sample_trajectory(&plan_result.segment);

    if (faceforward_)
      add_face_forward_trajectories(states, &plan_result.segment);

    plan_result.response = RESPONSE::SUCCESS;
    PlanResult(plan_result);
    timer_fb_.stop();
    return;
  }

  // Called to interrupt the process
  void CancelCallback() {}

 protected:
  ff_util::ConfigServer cfg_;
  ff_util::ConfigServer cfg_mapper_;
  ros::Timer timer_d_, timer_a_, timer_fb_;
  boost::shared_ptr<traj_opt::NonlinearTrajectory> trajectory_;
  tf::Quaternion start_orientation_;
  tf::Vector3 axis_;

  double planner_dt_{0.5};  // how often to sample trajectory for output
  double map_res_{0.5};     // map resolution

 private:
  std::shared_ptr<JPS::VoxelMapUtil> jps_map_util_;
  std::unique_ptr<EllipseDecomp> decomp_util_;
  std::unique_ptr<JPS::JPS3DUtil> jps_planner_;

  double norm_vector3(const geometry_msgs::Vector3 &vec) {
    return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
  }
  ff_msgs::ControlState getState(double time, double dt) {
    if (trajectory_ == NULL) return ff_msgs::ControlState();

    tf::Vector3 axis = axis_;
    if (std::isnan(axis.getX())) axis.setZero();
    tf::Quaternion orientation = start_orientation_;

    ff_msgs::ControlState state;
    traj_opt::MatD state_mat;
    trajectory_->getCommand(time, 3, state_mat);
    state.when = ros::Time(time);

    tf::Vector3 axis_world_frame = tf::Transform(start_orientation_) * axis;

    // OUTPUT_DEBUG("PlannerQP: State mat " << state_mat);
    state.pose.position.x = state_mat(0, 0),
    state.pose.position.y = state_mat(1, 0),
    state.pose.position.z = state_mat(2, 0);

    if (time < trajectory_->getTotalTime()) {
      state.twist.linear.x = state_mat(0, 1),
      state.twist.linear.y = state_mat(1, 1),
      state.twist.linear.z = state_mat(2, 1);

      state.accel.linear.x = state_mat(0, 2),
      state.accel.linear.y = state_mat(1, 2),
      state.accel.linear.z = state_mat(2, 2);
    }

    // load into control state msg
    if (!faceforward_) {
      orientation = start_orientation_ * tf::Quaternion(axis, state_mat(3, 0));
      if (time < trajectory_->getTotalTime()) {
        tf::vector3TFToMsg(axis_world_frame * state_mat(3, 1),
                           state.twist.angular);
        tf::vector3TFToMsg(axis_world_frame * state_mat(3, 2),
                           state.accel.angular);
      }
    } else {
      double timea = time;

      // if velocity is too small, use nearby
      traj_opt::MatD state_a;
      trajectory_->getCommand(timea, 3, state_a);
      int loop_count = 0;
      while (state_a.block<3, 1>(0, 1).norm() < 1e-7 && loop_count < 100) {
        if (timea <= trajectory_->getTotalTime() * 0.5)
          timea += dt;
        else
          timea -= dt;
        trajectory_->getCommand(timea, 3, state_a);
        OUTPUT_DEBUG("PlannerQP: Using alternate v " << timea);
        loop_count++;
      }
      if (loop_count >= 1000) {
        ROS_ERROR(
            "face_forward mode failed to calculate due to numerical issues");
        throw std::runtime_error(
            "face_forward mode failed to calculate due to numerical issues");
      }
      state_mat.block<3, 1>(0, 1) = state_a.block<3, 1>(0, 1);

      orientation = calculate_face_foward(state_mat.block<4, 1>(0, 1));
      // TODO(mfwatter): calculate twists ?
    }

    tf::quaternionTFToMsg(orientation, state.pose.orientation);
    return state;
  }
  bool calculate_time_scale() {
    if (trajectory_ == NULL) return false;

    Eigen::Array<traj_opt::decimal_t, 4, 1>
        limits_normalized;  // we normalize the limits so time scaling is
                            // proportional for all
    limits_normalized << desired_vel_, std::sqrt(desired_accel_),
        desired_omega_, std::sqrt(desired_alpha_);

    Eigen::Array<traj_opt::decimal_t, 4, 1> worst =
        Eigen::Array<traj_opt::decimal_t, 4, 1>::Zero();

    ff_msgs::ControlState last_state;
    for (double t = 0; t <= trajectory_->getTotalTime();
         t += 0.01) {  // should be sample rate of control
      ff_msgs::ControlState state = getState(t, 0.01);
      Eigen::Array<traj_opt::decimal_t, 4, 1> current_normalized;
      // same a normalization for limits

      current_normalized << norm_vector3(state.twist.linear),
          std::sqrt(norm_vector3(state.accel.linear)),
          norm_vector3(state.twist.angular),
          std::sqrt(norm_vector3(state.accel.angular));
      if (faceforward_ && t > 0) {
        tf::Quaternion start_orientation, end_orientation;
        tf::quaternionMsgToTF(state.pose.orientation, start_orientation);
        tf::quaternionMsgToTF(last_state.pose.orientation, end_orientation);
        tf::Quaternion error_quat =
            start_orientation.inverse() * end_orientation;
        error_quat.normalize();  // yes, this is needed badly to fix error when
                                 // axis returned is 0
        double error_angle = error_quat.getAngle();
        if (error_angle > M_PI) error_angle -= 2 * M_PI;
        current_normalized(2) = error_angle * 100;
        // this is 100 because = 1/0.01 for checking
      }
      last_state = state;
      worst = worst.max(current_normalized.abs());
      // should data types be complex?
    }
    Eigen::Array<traj_opt::decimal_t, 4, 1> ratio = worst / limits_normalized;
    // this is coefficent wise for arrays
    OUTPUT_DEBUG("PlannerQP: Violation ratio: " << ratio.transpose());
    OUTPUT_DEBUG("PlannerQP: Worst: " << worst.transpose());
    OUTPUT_DEBUG("PlannerQP: Limits: " << limits_normalized.transpose());

    double time_scale = ratio.block<2, 1>(0, 0).maxCoeff();
    time_scale = std::max(time_scale, ratio(3) * 2.0);
    trajectory_->scaleTime(time_scale);
    OUTPUT_DEBUG("PlannerQP: Scaling to: " << time_scale);
    if (time_scale > 600 || std::isnan(time_scale)) {
      ROS_ERROR_STREAM("Time is way too long! :" << time_scale);
      time_scale = std::max(time_scale, ratio(3) * 1.5);
      return false;
    }

    return true;
  }
  bool generate_trajectory(const traj_opt::Vec4 &start,
                           const traj_opt::Vec4 &goal,
                           ff_msgs::PlanResult *result) {
    OUTPUT_DEBUG("PlannerQP: Planning from " << start.transpose()
                                             << " to: " << goal.transpose());
    // clear trajectory
    trajectory_ = boost::shared_ptr<traj_opt::NonlinearTrajectory>();

    if (!cfg_.Get<bool>("time_optimization", time_optiization_))
      time_optiization_ = false;
    if (!cfg_.Get<bool>("uniform_time_allocation", uniform_time_))
      uniform_time_ = false;

    if (!cfg_.Get<double>("duality_gap_threshold", gap_threshold_))
      gap_threshold_ = 1e-8;
    if (!cfg_.Get<int>("maximum_iterations", max_iterations_))
      max_iterations_ = 200;

    // try to get zones
    if (!load_map()) {
      ROS_ERROR("Planner::QP: Planner failed to load keepins and keepouts");
      return false;
    }

    traj_opt::Vec3 start3 = start.block<3, 1>(0, 0);
    traj_opt::Vec3 goal3 = goal.block<3, 1>(0, 0);

    traj_opt::Vec3 diff = goal3 - start3;
    diff << std::abs(diff(0)), std::abs(diff(1)), std::abs(diff(2));
    diff -= traj_opt::Vec3::Ones() * map_res_;

    vec_Vec3f path;
    bool close = false;

    if (diff(0) < 0 && diff(1) < 0 && diff(2) < 0) {
      ROS_INFO_STREAM(
          "Start and goal are within map resolution: " << diff.transpose());
      path.push_back(start3);
      path.push_back(goal3);
      close = true;
      // result->response = RESPONSE::ALREADY_THERE;
    } else {
      OUTPUT_DEBUG("PlannerQP: JPS running");
      if (!jps_planner_->plan(start3, goal3)) {
        ROS_ERROR("Planner::QP: Jump point search failed!");
        return false;
      }
      path = jps_planner_->getPath();
    }
    // get constraints and repackage as dynamic sized arrays
    OUTPUT_DEBUG("PlannerQP: decomp running on path length " << path.size());
    decomp_util_->decomp(path);

    for (auto &p : path) OUTPUT_DEBUG("PlannerQP: Path: " << p.transpose());
    vec_LinearConstraint3f cons_3d = decomp_util_->get_constraints();
    std::vector<std::pair<traj_opt::MatD, traj_opt::VecD> > cons;
    for (auto &ci : cons_3d) {
      traj_opt::MatD A = traj_opt::MatD::Zero(ci.first.rows(), 4);
      traj_opt::VecD b =
          traj_opt::VecD::Zero(ci.second.rows(), ci.second.cols());
      // OUTPUT_DEBUG("PlannerQP: Ci size " << ci.second.rows() << " "<<
      // ci.second.cols() );
      if (!close) {
        A.block(0, 0, ci.first.rows(), ci.first.cols()) = ci.first;
        b.block(0, 0, ci.second.rows(), ci.second.cols()) = ci.second;
      }
      OUTPUT_DEBUG("PlannerQP: A: " << A);
      OUTPUT_DEBUG("PlannerQP: b: " << b.transpose());
      cons.push_back(std::pair<traj_opt::MatD, traj_opt::VecD>(A, b));
    }

    // Package higher order waypoints
    traj_opt::Waypoint start_way, goal_way;
    start_way.pos = start;
    start_way.use_pos = true;
    start_way.use_vel = true;
    start_way.use_acc = true;
    start_way.use_jrk = true;
    start_way.knot_id = 0;

    goal_way.pos = goal;
    goal_way.use_pos = true;
    goal_way.use_vel = true;
    goal_way.use_acc = true;
    goal_way.use_jrk = true;
    goal_way.knot_id = -1;

    std::vector<traj_opt::Waypoint> con;
    con.push_back(start_way);
    con.push_back(goal_way);

    boost::shared_ptr<std::vector<double> > ds =
        boost::make_shared<std::vector<double> >(cons.size(), 1.0);

    for (uint i = 1; i < path.size(); i++) {
      if (!uniform_time_) ds->at(i - 1) = (path.at(i) - path.at(i - 1)).norm();
      traj_opt::VecD p1(traj_opt::VecD::Zero(4, 1));
      traj_opt::VecD p2(traj_opt::VecD::Zero(4, 1));
      p1.block<3, 1>(0, 0) = path.at(i - 1);
      p2.block<3, 1>(0, 0) = path.at(i);
      traj_opt::VecD diff1 = cons.at(i - 1).first * p1 - cons.at(i - 1).second;
      traj_opt::VecD diff2 = cons.at(i - 1).first * p2 - cons.at(i - 1).second;
      OUTPUT_DEBUG("i: " << i << " diff1: " << diff1.maxCoeff()
                         << " diff2: " << diff2.maxCoeff());
      // OUTPUT_DEBUG("i: " << i << " diff1: " << diff1.transpose() << "
      // diff2: " << diff2.transpose());
    }

    try {
      trajectory_.reset(new traj_opt::NonlinearTrajectory(
          con, cons, 7, 3, ds, boost::shared_ptr<traj_opt::VecDVec>(),
          time_optiization_, gap_threshold_, max_iterations_));
    } catch (std::runtime_error &e) {
      ROS_ERROR_STREAM("QP::Planner failed with error: " << e.what());
      return false;
    } catch (...) {
      ROS_ERROR_STREAM("QP::Planner failed with unknown error");
      return false;
    }

    if (time_optiization_ && !trajectory_->isSolved()) {
      ROS_WARN_STREAM("Time optimization diverged, re running with out it");
      try {
        trajectory_.reset(new traj_opt::NonlinearTrajectory(
            con, cons, 7, 3, ds, boost::shared_ptr<traj_opt::VecDVec>(),
            false, gap_threshold_, max_iterations_));
      } catch (std::runtime_error &e) {
        ROS_ERROR_STREAM("QP::Planner failed with error: " << e.what());
        return false;
      } catch (...) {
        ROS_ERROR_STREAM("QP::Planner failed with unknown error");
        return false;
      }
    }

    std::string pass = trajectory_->isSolved() ? "solved" : "failed";
    // viz topics
    // VisualizeRectangularPolytopes::fromGraph(graph.get(),
    // path_solver->spath);
    // QPRosBridge::publish_msg(trajectory_->serialize());

    OUTPUT_DEBUG(
        "PlannerQP: Planner::QP: Finished solving with status: " << pass);

    // publish visualization
    {
      viz_cooridor(cons);
      TrajRosBridge::publish_msg(
          trajectory_->serialize(), "world",
          nh_->getNamespace() + std::string("/mob/planner_qp/trajectory"));
      std::vector<traj_opt::TrajData> history;
      TrajRosBridge::publish_msg(
          trajectory_->getInfo(&history),
          nh_->getNamespace() + std::string("/mob/planner_qp/solver_info"));

      traj_history_.clear();
      for (auto &traji : history)
        traj_history_.push_back(TrajRosBridge::convert(traji));
    }

    // ROS_ERROR_STREAM("name resolution " << nh_->getNamespace() );
    return trajectory_->isSolved();
  }
  tf::Quaternion calculate_face_foward(const traj_opt::Vec4 &vel) {
    // x axis of body is alligend with velocity, if velocity is zero, use
    // discrete value
    traj_opt::Vec3 x_b = vel.block<3, 1>(0, 0);
    x_b.normalize();
    // if (x_b.norm() <= 1e-5) {
    //   x_b = vela.block<3, 1>(0, 0);
    //   x_b.normalize();
    //   OUTPUT_DEBUG("PlannerQP: Using alternate v " <<
    //   vela.transpose());
    //   OUTPUT_DEBUG("PlannerQP: Correspoing v " << vel.transpose());
    // }
    // else {
    // }

    // find z orthrogoal to x clossest to "up"
    // TODO(mfwatter) add some hopf
    traj_opt::Vec3 z_w;
    z_w << 0, 0, 1;
    traj_opt::Vec3 z_b = z_w - x_b.dot(z_w) * z_w;
    z_b.normalize();

    // use cross product to find thrid basis vector
    traj_opt::Vec3 y_b = z_b.cross(x_b);

    // pack into matrix and transform into quaternion
    traj_opt::Mat3 R = traj_opt::Mat3::Zero();
    R.block<3, 1>(0, 0) = x_b;
    R.block<3, 1>(0, 1) = y_b;
    R.block<3, 1>(0, 2) = z_b;

    // OUTPUT_DEBUG("PlannerQP: R " << R);
    traj_opt::Quat quat(R);
    tf::Quaternion tf_quat(quat.x(), quat.y(), quat.z(), quat.w());
    return tf_quat;
  }
  bool sample_trajectory(std::vector<ff_msgs::ControlState> *controls) {
    // check for trajectory generation failure
    if (trajectory_ == NULL) return false;

    // get sample increment
    planner_dt_ = 1.0 / control_rate_;

    // increment time
    double time = 0;
    if (DEBUG)
      std::cout << "Total time of trajectory: " << trajectory_->getTotalTime()
                << std::endl;
    do {
      ff_msgs::ControlState state = getState(time, planner_dt_);
      // << " acc " <<  acc.transpose() << std::endl;
      // add to list
      controls->push_back(state);

      time += planner_dt_;
    } while (time < trajectory_->getTotalTime());
    time = trajectory_->getTotalTime();
    ff_msgs::ControlState state = getState(time, planner_dt_);
    controls->push_back(state);

    return true;
  }
  void add_face_forward_trajectories(
      const std::vector<geometry_msgs::PoseStamped> &states,
      std::vector<ff_msgs::ControlState> *controls) {
    std::vector<ff_msgs::ControlState> new_states;
    double dt = 1.0 / control_rate_;
    // Do front
    tf::Quaternion start_orientation, end_orientation;
    tf::quaternionMsgToTF(states.front().pose.orientation, start_orientation);
    tf::quaternionMsgToTF(controls->front().pose.orientation, end_orientation);
    tf::Quaternion error_quat = start_orientation.inverse() * end_orientation;
    error_quat.normalize();  // yes, this is needed badly to fix error when axis
                             // returned is 0
    double error_angle = error_quat.getAngle();
    if (error_angle > M_PI) error_angle -= 2 * M_PI;
    // look at all these magic numbers!! TODO(mfwatter) document these
    double T1 = (35.0 / 16.0) * std::abs(error_angle) / desired_omega_;
    double T2 = std::sqrt(84.0 / 25.0 * std::sqrt(5.0) * std::abs(error_angle) /
                          desired_alpha_);
    double T = std::max(T1, T2);
    tf::Vector3 axis = error_quat.getAxis();
    tf::Vector3 axis_world_frame = tf::Transform(start_orientation_) * axis;
    if (T > dt) {
      for (double ds = 0; ds <= 1.0; ds += dt / T) {
        ff_msgs::ControlState state;
        double p = (ds * ds * ds * ds) * error_angle * 3.5E1 -
                   (ds * ds * ds * ds * ds) * error_angle * 8.4E1 +
                   (ds * ds * ds * ds * ds * ds) * error_angle * 7.0E1 -
                   (ds * ds * ds * ds * ds * ds * ds) * error_angle * 2.0E1;
        double v = (ds * ds * ds) * error_angle * 1.4E2 -
                   (ds * ds * ds * ds) * error_angle * 4.2E2 +
                   (ds * ds * ds * ds * ds) * error_angle * 4.2E2 -
                   (ds * ds * ds * ds * ds * ds) * error_angle * 1.4E2;
        double a = (ds * ds) * error_angle * 4.2E2 -
                   (ds * ds * ds) * error_angle * 1.68E3 +
                   (ds * ds * ds * ds) * error_angle * 2.1E3 -
                   (ds * ds * ds * ds * ds) * error_angle * 8.4E2;
        v /= T;
        a /= T * T;
        state.pose.position = controls->front().pose.position;
        tf::quaternionTFToMsg(start_orientation_ * tf::Quaternion(axis, p),
                              state.pose.orientation);
        tf::vector3TFToMsg(axis_world_frame * v, state.twist.angular);
        tf::vector3TFToMsg(axis_world_frame * a, state.accel.angular);
        state.when = ros::Time(ds * T);
        new_states.push_back(state);
        // OUTPUT_DEBUG("PlannerQP: theta z1: " <<
        // state.pose.orientation.z);
        // OUTPUT_DEBUG("PlannerQP: w z1: " << state.twist.angular.z <<
        // " max: " <<
        // max_w);
        // OUTPUT_DEBUG("PlannerQP: v : " << v << " ds: " << ds);
      }
    } else {
      T = 0;
    }
    for (uint i = 1; i < new_states.size(); i++) {
      auto &state = new_states.at(i);
      // calulate angular vel
      tf::quaternionMsgToTF(new_states.at(i - 1).pose.orientation,
                            start_orientation);
      tf::quaternionMsgToTF(state.pose.orientation, end_orientation);
      error_quat = start_orientation.inverse() * end_orientation;
      error_quat.normalize();  // yes, this is needed badly to fix error when
                               // axis returned is 0
      error_angle = error_quat.getAngle();
      if (error_angle > M_PI) error_angle -= 2 * M_PI;
      axis = error_quat.getAxis();
      axis_world_frame = tf::Transform(start_orientation) * axis;
      tf::vector3TFToMsg(axis_world_frame * error_angle / dt,
                         state.twist.angular);
    }
    // append calculated trajectory to middle
    for (auto &state : *controls) {
      // OUTPUT_DEBUG("PlannerQP: theta z2: " <<
      // state.pose.orientation.z);
      state.when += ros::Duration(T);
    }
    controls->insert(controls->begin(), new_states.begin(), new_states.end());
    // double Te = T + trajectory_->getTotalTime();
    double Te = controls->back().when.toSec();
    // add end trajectory
    tf::quaternionMsgToTF(controls->back().pose.orientation, start_orientation);
    tf::quaternionMsgToTF(states.back().pose.orientation, end_orientation);
    error_quat = start_orientation.inverse() * end_orientation;
    error_quat.normalize();  // yes, this is needed badly to fix error when axis
                             // returned is 0
    error_angle = error_quat.getAngle();
    if (error_angle > M_PI) error_angle -= 2 * M_PI;
    T1 = (35.0 / 16.0) * std::abs(error_angle) / desired_omega_;
    T2 = std::sqrt(84.0 / 25.0 * std::sqrt(5.0) * std::abs(error_angle) /
                   desired_alpha_);
    T = std::max(T1, T2);
    axis = error_quat.getAxis();
    axis_world_frame = tf::Transform(start_orientation_) * axis;
    if (T > dt) {
      for (double dds = 0; dds <= 1.1; dds += dt / T) {
        double ds = dds;
        if (ds > 1.0) ds = 1.0;
        ff_msgs::ControlState state;
        double p = (ds * ds * ds * ds) * error_angle * 3.5E1 -
                   (ds * ds * ds * ds * ds) * error_angle * 8.4E1 +
                   (ds * ds * ds * ds * ds * ds) * error_angle * 7.0E1 -
                   (ds * ds * ds * ds * ds * ds * ds) * error_angle * 2.0E1;
        double v = (ds * ds * ds) * error_angle * 1.4E2 -
                   (ds * ds * ds * ds) * error_angle * 4.2E2 +
                   (ds * ds * ds * ds * ds) * error_angle * 4.2E2 -
                   (ds * ds * ds * ds * ds * ds) * error_angle * 1.4E2;
        double a = (ds * ds) * error_angle * 4.2E2 -
                   (ds * ds * ds) * error_angle * 1.68E3 +
                   (ds * ds * ds * ds) * error_angle * 2.1E3 -
                   (ds * ds * ds * ds * ds) * error_angle * 8.4E2;
        v /= T;
        a /= T * T;
        state.pose.position = controls->back().pose.position;
        tf::quaternionTFToMsg(start_orientation * tf::Quaternion(axis, p),
                              state.pose.orientation);
        tf::vector3TFToMsg(axis_world_frame * v, state.twist.angular);
        tf::vector3TFToMsg(axis_world_frame * a, state.accel.angular);
        state.when = ros::Time(ds * T + Te);
        controls->push_back(state);
        // OUTPUT_DEBUG("PlannerQP: theta z3: " <<
        // state.pose.orientation.z);
      }
    }
  }
  bool load_map() {
    // Get obstacle map
    float resf = 0.08;
    pcl::PointCloud<pcl::PointXYZ> points;
    if (!GetObstacleMap(&points, &resf)) {
      ROS_ERROR_STREAM("PlannerQP: Failed to get points from mapper service");
      return false;
    }
    mapper_points_.clear();
    mapper_points_.reserve(points.size());

    for (auto &p : points) {
      mapper_points_.push_back(Vec3f(p.x, p.y, p.z));
    }
    // Declare keepout points
    vec_Vec3f keepout_points_mapper = mapper_points_;

    // Get zones map
    std::vector<signed char> map;
    Vec3f origin;
    Vec3i dim;
    if (!GetZonesMap(map, origin, dim, map_res_)) {
      ROS_ERROR_STREAM("PlannerQP: Failed to get points from zones service");
      return false;
    }

    jps_map_util_.reset(new JPS::VoxelMapUtil());
    jps_map_util_->setMap(origin, dim, map, map_res_);

    // Set all remaining space as free
    jps_map_util_->freeUnKnown();

    // Add obstacle map points
    jps_map_util_->dilate(map_res_ / 2, map_res_ / 2);    // sets dilating radius
    jps_map_util_->add3DPoints(keepout_points_mapper);    // add keepout points from mapper

    OUTPUT_DEBUG("PlannerQP: Map origin " << origin.transpose() << " dim "
                                          << dim.transpose() << " resolution "
                                          << map_res_);

    // inflate map for planning further from walls, the minimum inflation is the map resolution,
    // this parameter is adjusted to generate trajectories further from the walls
    double planner_distance = map_res_;
    if (!cfg_.Get<double>("planner_distance", planner_distance)) {
      ROS_ERROR_STREAM("planner_distance not defined");
    }
    if (planner_distance < map_res_)
      planner_distance = map_res_;

    jps_map_util_->dilate(planner_distance, planner_distance);    // sets dilating radius
    jps_map_util_->dilating();                                    // dillate entire map


    jps_planner_.reset(new JPS::JPS3DUtil(false));
    jps_planner_->setMapUtil(jps_map_util_.get());

    decomp_util_.reset(new EllipseDecomp(
        jps_map_util_->getOrigin(),
        jps_map_util_->getDim().cast<decimal_t>() * jps_map_util_->getRes(),
        false));
    decomp_util_->set_obstacles(jps_map_util_->getCloud());

    // debugCloud();

    return true;
  }
  /*  void debugCloud(){
      // vec_Vec3f free = jps_map_util_->getFreeCloud();
      vec_Vec3f free = jps_map_util_->getCloud();
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
    pcl::PointCloud<pcl::PointXYZ>);
      cloud->header.frame_id = "world";

      for(auto &p:free){
        cloud->push_back(pcl::PointXYZ(p(0),p(1),p(2)));
        // std::cout << p.transpose() << std::endl;
      }

      // pcl::io::savePCDFileASCII ("iss_free.pcd", *cloud);
      cloud_pub_.publish(clzoud);
    }*/

  bool LoadActionParams(const ff_msgs::PlanGoal &goal) {
    faceforward_ = goal.faceforward;
    check_obstacles_ = goal.check_obstacles;
    desired_vel_ = goal.desired_vel;
    desired_accel_ = goal.desired_accel;
    desired_omega_ = goal.desired_omega;
    desired_alpha_ = goal.desired_alpha;
    control_rate_ = goal.desired_rate;
    max_time_ = goal.max_time.toSec();
    // control_rate_*=10;
    OUTPUT_DEBUG("PlannerQP: Control period " << control_rate_);

    // check validity of params
    if (control_rate_ == 0.0 || desired_vel_ == 0.0 || desired_accel_ == 0.0 ||
        desired_omega_ == 0.0 || desired_alpha_ == 0.0)
      return false;

    return true;
  }

 private:
  vec_Vec3f mapper_points_;

  // Voxel map values
  char val_occ_ = 100;         // Assume occupied cell has value 100
  char val_free_ = 0;          // Assume free cell has value 0
  char val_unknown_ = -1;      // Assume unknown cell has value -1

  void map_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg) {
    // clear old points
    mapper_points_.clear();
    mapper_points_.reserve(msg->size());

    for (auto &p : *msg) {
      mapper_points_.push_back(Vec3f(p.x, p.y, p.z));
    }
  }
};

PLUGINLIB_EXPORT_CLASS(planner_qp::Planner, nodelet::Nodelet);

}  // namespace planner_qp
