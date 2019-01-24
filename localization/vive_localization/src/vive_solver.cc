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

// This include
#include "./vive_solver.h"

/**
 * \ingroup localization
 */
namespace vive_localization {

Solver::Solver(ros::NodeHandle nh) {
  // Read the vive configuration
  config_reader::ConfigReader cfg;
  cfg.AddFile("localization/vive_localization.config");

  // Try and open the Lua file
  if (!cfg.ReadFiles())
    ROS_FATAL("cannot read Lua config");

  // Read lighthouse calibration information
  if (!ReadLighthouseConfig(&cfg, calibration_))
    ROS_FATAL("cannot read lighthouse config");

  // Read tracker extrinsics
  if (!ReadTrackerConfig(&cfg, trackers_))
    ROS_FATAL("cannot read tracker config");

  // Read tracker extrinsics
  if (!ReadRegistrationConfig(&cfg, wTv_))
    ROS_FATAL("cannot read registration config");

  // Get the basic parameters
  if (!cfg.GetPosReal("resolution", &res_))
    ROS_FATAL("Failed to get resolution parameter.");
  if (!cfg.GetBool("correct", &correct_))
    ROS_FATAL("Failed to get correct parameter.");
  if (!cfg.GetReal("smoothing", &smooth_))
    ROS_FATAL("Failed to get smoothing parameter.");

  // What to do as part of the solver
  if (!cfg.GetBool("calibrate", &calibrate_))
    ROS_FATAL("Failed to get calibrate parameter.");
  if (!cfg.GetBool("refine", &refine_))
    ROS_FATAL("Failed to get refine parameter.");
  if (!cfg.GetBool("register", &register_))
    ROS_FATAL("Failed to get register parameter.");

  // What to refine
  if (!cfg.GetBool("refine_extrinsics", &refine_extrinsics_))
    ROS_FATAL("Failed to get refine_extrinsics parameter.");
  if (!cfg.GetBool("refine_sensors", &refine_sensors_))
    ROS_FATAL("Failed to get refine_sensors parameter.");
  if (!cfg.GetBool("refine_head", &refine_head_))
    ROS_FATAL("Failed to get refine_head parameter.");
  if (!cfg.GetBool("refine_params", &refine_params_))
    ROS_FATAL("Failed to get refine_params parameter.");

  // Get the thresholds
  int cnt = 0;
  if (!cfg.GetPosReal("threshold_min_angle", &min_angle_))
    ROS_FATAL("Failed to get threshold_min_angle parameter.");
  if (!cfg.GetPosReal("threshold_max_angle", &max_angle_))
    ROS_FATAL("Failed to get threshold_max_angle parameter.");
  if (!cfg.GetPosReal("threshold_duration", &duration_))
    ROS_FATAL("Failed to get threshold_duration parameter.");
  if (!cfg.GetInt("threshold_count", &cnt))
    ROS_FATAL("Failed to get threshod_count parameter.");
  count_ = static_cast<size_t>(cnt);

  // Setup the solver options
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  if (!cfg.GetPosReal("solver_max_time", &options_.max_solver_time_in_seconds))
    ROS_FATAL("Failed to get the solver_max_time parameter.");
  if (!cfg.GetInt("solver_max_iterations", &options_.max_num_iterations))
    ROS_FATAL("Failed to get the solver_max_iterations parameter.");
  if (!cfg.GetInt("solver_threads", &options_.num_threads))
    ROS_FATAL("Failed to get the solver_threads parameter.");
  if (!cfg.GetBool("solver_debug", &options_.minimizer_progress_to_stdout))
    ROS_FATAL("Failed to get the solver_debug parameter.");
}

// Add trackers
void Solver::Add(ff_hw_msgs::ViveLighthouses::ConstPtr msg, ros::Time const& t) {
  LighthouseCallback(msg, lighthouses_);
  for (auto & lighthouse : lighthouses_) {
    // Find the calibration data based on the serial
    auto cal = calibration_.find(lighthouse.second.serial);
    if (cal == calibration_.end())
      continue;
    // Save the lighthouse calibration datae
    Eigen::AngleAxisd axis_angle(cal->second.linear());
    lighthouse.second.vTl[0] = cal->second.translation()[0];
    lighthouse.second.vTl[1] = cal->second.translation()[1];
    lighthouse.second.vTl[2] = cal->second.translation()[2];
    lighthouse.second.vTl[3] = axis_angle.angle() * axis_angle.axis()[0];
    lighthouse.second.vTl[4] = axis_angle.angle() * axis_angle.axis()[1];
    lighthouse.second.vTl[5] = axis_angle.angle() * axis_angle.axis()[2];
  }
}

// Add Lighthouses
void Solver::Add(ff_hw_msgs::ViveTrackers::ConstPtr msg, ros::Time const& t) {
  TrackerCallback(msg, trackers_);
}

// Add a light measurement
void Solver::Add(ff_hw_msgs::ViveLight::ConstPtr msg, ros::Time const& t) {
  ros::Time dtu = ros::Time(round(t.toSec() / res_) * res_);
  std::string const& tracker = msg->header.frame_id;
  uint8_t const& lighthouse = msg->lighthouse;
  uint8_t const& axis = msg->axis;
  // Some garbage filtering
  for (auto p = msg->pulses.begin(); p != msg->pulses.end(); p++) {
    if (p->angle < min_angle_ / 180.0 * M_PI ||
        p->angle > max_angle_ / 180.0 * M_PI ||
        p->duration > duration_ / 1e6) continue;
    light_[dtu][lighthouse][tracker][p->sensor][axis].push_back(p->angle);
  }
}

// Add a correction
void Solver::Add(geometry_msgs::PoseStamped::ConstPtr msg, ros::Time const& t) {
  ros::Time dtu = ros::Time(round(t.toSec() / res_) * res_);
  Eigen::Quaterniond quaternion(
    msg->pose.orientation.w,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z);
  Eigen::AngleAxisd axis_angle(quaternion.toRotationMatrix());
  correction_[dtu][0] = msg->pose.position.x;
  correction_[dtu][1] = msg->pose.position.y;
  correction_[dtu][2] = msg->pose.position.z;
  correction_[dtu][3] = axis_angle.angle() * axis_angle.axis()[0];
  correction_[dtu][4] = axis_angle.angle() * axis_angle.axis()[1];
  correction_[dtu][5] = axis_angle.angle() * axis_angle.axis()[2];
}

// Solve the localization problem
bool Solver::Calibrate() {
  //////////////////////////
  // INTERMEDIATE STORAGE //
  //////////////////////////

  typedef std::map<uint8_t,           // Lighthouse
      std::map<ros::Time,             // Time
        double[6]                     // Transform
    >
  > PoseMap;

  PoseMap poses;

  ///////////////////
  // BOOTSTRAPPING //
  ///////////////////

  ROS_INFO("Using P3P to bootstrap tracker trajectories in lighthouse frame");
  {
    // Iterate over lighthouses
    for (auto it = light_.begin(); it != light_.end(); it++) {
      // Iterate over timesteps
      for (auto lt = it->second.begin(); lt != it->second.end(); lt++) {
        // For calculating the lTb transform
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img;
        // Iterate over trackers
        for (auto tt = lt->second.begin(); tt != lt->second.end(); tt++) {
          // Iterate over sensors
          for (auto st = tt->second.begin(); st != tt->second.end(); st++) {
            // Check that we have both axes
            if (st->second.find(0) == st->second.end() ||
                st->second.find(1) == st->second.end() ||
                st->second[0].size() < count_          ||
                st->second[1].size() < count_) continue;
            // Get the angles
            double angles[2] = {
              std::accumulate(st->second[0].begin(), st->second[0].end(), 0.0)
                / static_cast<double>(st->second[0].size()),
              std::accumulate(st->second[1].begin(), st->second[1].end(), 0.0)
                / static_cast<double>(st->second[1].size())
            };
            // Correct the angles using the lighthouse parameters
            Correct(lighthouses_[lt->first].params, angles, correct_);
            // Get the sensor position in the tracker frame
            double x[3] = {
              trackers_[tt->first].sensors[st->first * 6  + 0],
              trackers_[tt->first].sensors[st->first * 6  + 1],
              trackers_[tt->first].sensors[st->first * 6  + 2]
            };
            // Transform the sensor position to the body frame
            InverseTransformInPlace(trackers_[tt->first].tTh, x);
            TransformInPlace(trackers_[tt->first].bTh, x);
            // Push on the correct world sensor position
            obj.push_back(cv::Point3f(x[0], x[1], x[2]));
            img.push_back(cv::Point2f(tan(angles[0]), tan(angles[1])));
          }
        }
        // If we have at least 4 sensors across all trackers
        Eigen::Affine3d lTb = Eigen::Affine3d::Identity();
        if (SolvePnP(obj, img, lTb)) {
          Eigen::AngleAxisd aa(lTb.linear());
          poses[lt->first][it->first][0] = lTb.translation()[0];
          poses[lt->first][it->first][1] = lTb.translation()[1];
          poses[lt->first][it->first][2] = lTb.translation()[2];
          poses[lt->first][it->first][3] = aa.angle() * aa.axis()[0];
          poses[lt->first][it->first][4] = aa.angle() * aa.axis()[1];
          poses[lt->first][it->first][5] = aa.angle() * aa.axis()[2];
        }
      }
    }
  }

  ///////////////
  // ALIGNMENT //
  ///////////////

  ROS_INFO("Using Kabsch to find the master -> slave lighthouse transform");
  {
    auto pm = poses.begin();
    for (auto pt = poses.begin(); pt != poses.end(); pt++) {
      // Special case: master lighthouse
      auto lt = lighthouses_.find(pt->first);
      if (pt == pm) {
        for (size_t i = 0; i < 6; i++)
          lt->second.vTl[i] = 0.0;
        continue;
      }
      // General case: correspondences with the master lighthouse
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corresp;
      for (auto it = pt->second.begin(); it != pt->second.end(); it++) {
        if (pm->second.find(it->first) == pm->second.end())
          continue;
        // If we get here, then we have a correspondence
        corresp.push_back(std::pair<Eigen::Vector3d, Eigen::Vector3d>(
          Eigen::Vector3d(
            pt->second[it->first][0],
            pt->second[it->first][1],
            pt->second[it->first][2]),
          Eigen::Vector3d(
            pm->second[it->first][0],
            pm->second[it->first][1],
            pm->second[it->first][2])));
      }
      if (corresp.size() > 3) {
        Eigen::Matrix<double, 3, Eigen::Dynamic> pti(3, corresp.size());
        Eigen::Matrix<double, 3, Eigen::Dynamic> ptj(3, corresp.size());
        for (size_t i = 0; i < corresp.size(); i++) {
          pti.block<3, 1>(0, i) = corresp[i].first;
          ptj.block<3, 1>(0, i) = corresp[i].second;
        }
        Eigen::Affine3d A;
        if (Kabsch<double>(pti, ptj, A, false)) {
          lt->second.vTl[0] = A.translation()[0];
          lt->second.vTl[1] = A.translation()[1];
          lt->second.vTl[2] = A.translation()[2];
          Eigen::AngleAxisd aa(A.linear());
          lt->second.vTl[3] =  aa.angle() * aa.axis()[0];
          lt->second.vTl[4] =  aa.angle() * aa.axis()[1];
          lt->second.vTl[5] =  aa.angle() * aa.axis()[2];
        }
      }
    }
  }
  // Success
  return true;
}

// Solve the localization problem
bool Solver::Refine() {
  ceres::Problem problem;

  //////////////
  // SOLUTION //
  //////////////

  ROS_INFO("Using non-linear least squares to refine solution");
  {
    // Iterate over lighthouses
    for (auto it = light_.begin(); it != light_.end(); it++) {
      // Iterate over timesteps
      for (auto lt = it->second.begin(); lt != it->second.end(); lt++) {
        // For calculating the lTb transform
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img;
        // Iterate over trackers
        for (auto tt = lt->second.begin(); tt != lt->second.end(); tt++) {
          // Iterate over sensors
          for (auto st = tt->second.begin(); st != tt->second.end(); st++) {
            // Check that we have both axes
            if (st->second.find(0) == st->second.end() ||
                st->second.find(1) == st->second.end() ||
                st->second[0].size() < count_          ||
                st->second[1].size() < count_) continue;
            // Get the angle measurement
            double angles[2] = {
              std::accumulate(st->second[0].begin(), st->second[0].end(), 0.0)
                / static_cast<double>(st->second[0].size()),
              std::accumulate(st->second[1].begin(), st->second[1].end(), 0.0)
                / static_cast<double>(st->second[1].size())
            };
            // Add the cost function
            ceres::CostFunction* cost =
              new ceres::AutoDiffCostFunction<LightCost, 2, 6, 6, 6,
                6, NUM_SENSORS * 6, NUM_PARAMS * 2>(new LightCost(
                  st->first, angles, correct_));
            // Add the residual block
            problem.AddResidualBlock(cost, new ceres::CauchyLoss(1.0),
              reinterpret_cast<double*>(lighthouses_[lt->first].vTl),
              reinterpret_cast<double*>(trajectory_[it->first]),
              reinterpret_cast<double*>(trackers_[tt->first].bTh),
              reinterpret_cast<double*>(trackers_[tt->first].tTh),
              reinterpret_cast<double*>(trackers_[tt->first].sensors),
              reinterpret_cast<double*>(lighthouses_[lt->first].params));
          }
        }
      }
    }
    // If we have a previous node, then link with a motion cost
    for (auto it = trajectory_.begin(); it != trajectory_.end(); it++) {
      // If we are smoothing
      if (smooth_ > 0) {
        auto pt = std::prev(it);
        if (it != trajectory_.end() && pt != trajectory_.end() && pt != it) {
          // Create a cost function to represent motion
          ceres::CostFunction* cost = new ceres::AutoDiffCostFunction
            <SmoothCost, 6, 6, 6>(new SmoothCost(smooth_));
          // Add a residual block for error
          problem.AddResidualBlock(cost, new ceres::CauchyLoss(1.0),
            reinterpret_cast<double*>(pt->second),   // Previous state
            reinterpret_cast<double*>(it->second));  // Next state
        }
      }
    }
    // Fix tracker parameters
    for (auto tt = trackers_.begin(); tt != trackers_.end(); tt++) {
      if (!refine_extrinsics_)
        problem.SetParameterBlockConstant(tt->second.bTh);
      if (!refine_head_)
        problem.SetParameterBlockConstant(tt->second.tTh);
      if (!refine_sensors_)
        problem.SetParameterBlockConstant(tt->second.sensors);
    }

    // Fix lighthouse parameters
    for (auto lt = lighthouses_.begin(); lt != lighthouses_.end(); lt++) {
      problem.SetParameterBlockConstant(lt->second.vTl);
      if (!refine_params_)
        problem.SetParameterBlockConstant(lt->second.params);
    }

    // Solve the problem
    ceres::Solver::Summary summary;
    ceres::Solve(options_, &problem, &summary);
    return summary.IsSolutionUsable();
  }
  return false;
}

// Solve the localization problem
bool Solver::Register() {
  ROS_INFO("Using Kabsch to find the vive -> world transform");
  {
    std::vector<ros::Time> corresp;
    for (auto tt = correction_.begin(); tt != correction_.end(); tt++)
      if (trajectory_.find(tt->first) != trajectory_.end())
        corresp.push_back(tt->first);
    if (corresp.size() > 3) {
      ROS_INFO_STREAM("- " << corresp.size() << " correspondences found");
      Eigen::Matrix<double, 3, Eigen::Dynamic> pti(3, corresp.size());
      Eigen::Matrix<double, 3, Eigen::Dynamic> ptj(3, corresp.size());
      for (size_t i = 0; i < corresp.size(); i++) {
        ros::Time const& t = corresp[i];
        pti.block<3, 1>(0, i) = Eigen::Vector3d(
          trajectory_[t][0], trajectory_[t][1], trajectory_[t][2]);
        ptj.block<3, 1>(0, i) = Eigen::Vector3d(
          correction_[t][0], correction_[t][1], correction_[t][2]);
      }
      Eigen::Affine3d A;
      if (Kabsch<double>(pti, ptj, A, false)) {
        ROS_INFO("- Solution Found");
        wTv_[0] = A.translation()[0];
        wTv_[1] = A.translation()[1];
        wTv_[2] = A.translation()[2];
        Eigen::AngleAxisd aa(A.linear());
        wTv_[3] =  aa.angle() * aa.axis()[0];
        wTv_[4] =  aa.angle() * aa.axis()[1];
        wTv_[5] =  aa.angle() * aa.axis()[2];
      } else {
        ROS_WARN("- Solution not found");
      }
    } else {
      ROS_WARN("- Not enough correspondences");
    }
  }
  return true;
}

// Solve the localization problem
bool Solver::Solve() {
  // Run calibration to solve for the lighthouse -> vive transforms
  if (calibrate_) {
    if (!Calibrate()) {
      ROS_WARN_STREAM("Lighthouse calibration failed");
    }
  }

  // Local solution pre-average
  std::map<ros::Time, std::vector<Eigen::Affine3d>> local;

  // Initial solution for body -> vive transforms
  for (auto it = light_.begin(); it != light_.end(); it++) {
    // Iterate over lighthouses
    for (auto lt = it->second.begin(); lt != it->second.end(); lt++) {
      // For calculating the lTb transform
      std::vector<cv::Point3f> obj;
      std::vector<cv::Point2f> img;
      // Iterate over trackers
      for (auto tt = lt->second.begin(); tt != lt->second.end(); tt++) {
        // Iterate over sensors
        for (auto st = tt->second.begin(); st != tt->second.end(); st++) {
          // Check that we have both axes
          if (st->second.find(0) == st->second.end() ||
              st->second.find(1) == st->second.end() ||
              st->second[0].size() < count_          ||
              st->second[1].size() < count_) continue;
          // Get the angle measurement
          double angles[2] = {
            std::accumulate(st->second[0].begin(), st->second[0].end(), 0.0)
              / static_cast<double>(st->second[0].size()),
            std::accumulate(st->second[1].begin(), st->second[1].end(), 0.0)
              / static_cast<double>(st->second[1].size())
          };
          // Correct the angles using the lighthouse parameters
          Correct(lighthouses_[lt->first].params, angles, correct_);
          // Get the sensor in the body frame
          double x[3] = {
            trackers_[tt->first].sensors[st->first * 6  + 0],
            trackers_[tt->first].sensors[st->first * 6  + 1],
            trackers_[tt->first].sensors[st->first * 6  + 2]
          };
          InverseTransformInPlace(trackers_[tt->first].tTh, x);
          TransformInPlace(trackers_[tt->first].bTh, x);
          // Push on the correspondence
          obj.push_back(cv::Point3f(x[0], x[1], x[2]));
          img.push_back(cv::Point2f(tan(angles[0]), tan(angles[1])));
        }
      }
      // Get an initial estimate for the timestep based on the lighthouse
      Eigen::Affine3d lTb = Eigen::Affine3d::Identity();
      if (SolvePnP(obj, img, lTb)) {
        // Convert vTl transform from double to Eigen
        Eigen::Vector3d v(lighthouses_[lt->first].vTl[3],
                          lighthouses_[lt->first].vTl[4],
                          lighthouses_[lt->first].vTl[5]);
        Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
        if (v.norm() > 0) {
          aa.angle() = v.norm();
          aa.axis() = v.normalized();
        }
        Eigen::Affine3d vTl = Eigen::Affine3d::Identity();
        vTl.linear() = aa.toRotationMatrix();
        vTl.translation()[0] = lighthouses_[lt->first].vTl[0];
        vTl.translation()[1] = lighthouses_[lt->first].vTl[1];
        vTl.translation()[2] = lighthouses_[lt->first].vTl[2];
        // Merge the two poses
        local[it->first].push_back(vTl * lTb);
      }
    }
  }

  // Average the transforms across all lighthouses, under the assumption that
  // the multiple solutions are extremely close to each other.
  for (auto const& timestamp : local) {
    trajectory_[timestamp.first][0] = 0.0;
    trajectory_[timestamp.first][1] = 0.0;
    trajectory_[timestamp.first][2] = 0.0;
    trajectory_[timestamp.first][3] = 0.0;
    trajectory_[timestamp.first][4] = 0.0;
    trajectory_[timestamp.first][5] = 0.0;
    double s = 1.0 / static_cast<double>(timestamp.second.size());
    for (auto const& pose : timestamp.second) {
      Eigen::AngleAxisd aa = Eigen::AngleAxisd(pose.linear());
      trajectory_[timestamp.first][0] += s * (pose.translation()[0]);
      trajectory_[timestamp.first][1] += s * (pose.translation()[1]);
      trajectory_[timestamp.first][2] += s * (pose.translation()[2]);
      trajectory_[timestamp.first][3] += s * (aa.angle() * aa.axis()[0]);
      trajectory_[timestamp.first][4] += s * (aa.angle() * aa.axis()[1]);
      trajectory_[timestamp.first][5] += s * (aa.angle() * aa.axis()[2]);
    }
  }

  // Run refinement, if needed
  if (refine_) {
    if (!Refine()) {
      ROS_WARN_STREAM("Body pose refining failed");
    }
  }

  // Run registration if needed
  if (register_) {
    if (!Register()) {
      ROS_WARN_STREAM("World registration failed");
    }
  }

  // Update the static transforms
  SendTransforms(FRAME_NAME_WORLD, FRAME_NAME_VIVE, FRAME_NAME_BODY,
    wTv_, lighthouses_, trackers_);

  // Notify and return
  ROS_INFO("Solution found.");
  ROS_INFO("Put this REGISTRATION in your world config:");
  ROS_INFO_STREAM("  world_vive_registration = ");
  PrintTransform(wTv_);
  ROS_INFO("Put these LIGHTHOUSES in your world config:");
  for (auto const& lighthouse : lighthouses_) {
    ROS_INFO_STREAM("  serial = \"" << lighthouse.second.serial << "\", pose = ");
    PrintTransform(lighthouse.second.vTl);
  }
  return true;
}

// Print a human readable transform
void Solver::PrintTransform(const double tf[6]) {
  Eigen::Vector3d v(tf[3], tf[4], tf[5]);
  Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
  if (v.norm() > 0) {
    aa.angle() = v.norm();
    aa.axis() = v.normalized();
  }
  Eigen::Quaterniond q(aa.toRotationMatrix());
  ROS_INFO_STREAM("    transform(vec3(" << std::setprecision(6)
    << tf[0] << ", "
    << tf[1] << ", "
    << tf[2] << "), quat("
    << q.x() << ", "
    << q.y() << ", "
    << q.z() << ", "
    << q.w() << "))");
}

// Get the truthful path
void Solver::GetTruth(nav_msgs::Path & path) {
  geometry_msgs::PoseStamped ps;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = FRAME_NAME_WORLD;
  for (auto it = correction_.begin(); it != correction_.end(); it++) {
    Eigen::Vector3d v(it->second[3], it->second[4], it->second[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    ps.header.stamp = it->first;
    ps.header.frame_id = FRAME_NAME_WORLD;
    ps.pose.position.x = it->second[0];
    ps.pose.position.y = it->second[1];
    ps.pose.position.z = it->second[2];
    ps.pose.orientation.w = q.w();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    path.poses.push_back(ps);
  }
}

// Get the Vive path
void Solver::GetVive(nav_msgs::Path & path) {
  // Get the wTv transformation
  Eigen::Affine3d wTv = Eigen::Affine3d::Identity();
  Eigen::Vector3d v(wTv_[3], wTv_[4], wTv_[5]);
  Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
  if (v.norm() > 0) {
    aa.angle() = v.norm();
    aa.axis() = v.normalized();
  }
  wTv.linear() = aa.toRotationMatrix();
  wTv.translation()[0] = wTv_[0];
  wTv.translation()[1] = wTv_[1];
  wTv.translation()[2] = wTv_[2];
  // Generate the path
  geometry_msgs::PoseStamped ps;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = FRAME_NAME_WORLD;
  for (auto it = trajectory_.begin(); it != trajectory_.end(); it++) {
    // Get the vTb transformation
    Eigen::Vector3d v(it->second[3], it->second[4], it->second[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Affine3d vTb = Eigen::Affine3d::Identity();
    vTb.linear() = aa.toRotationMatrix();
    vTb.translation()[0] = it->second[0];
    vTb.translation()[1] = it->second[1];
    vTb.translation()[2] = it->second[2];
    // Chain the transform to find wTb
    Eigen::Affine3d wTb = wTv * vTb;
    Eigen::Quaterniond q(wTb.linear());
    // Add the pose to the navigation path
    ps.header.stamp = it->first;
    ps.header.frame_id = FRAME_NAME_WORLD;
    ps.pose.position.x = wTb.translation()[0];
    ps.pose.position.y = wTb.translation()[1];
    ps.pose.position.z = wTb.translation()[2];
    ps.pose.orientation.w = q.w();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    path.poses.push_back(ps);
  }
}

}  // namespace vive_localization
