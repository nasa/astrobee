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

#ifndef VISUALEYEZ_VISUALEYEZ_SERVER_H_
#define VISUALEYEZ_VISUALEYEZ_SERVER_H_

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_broadcaster.h>

// To visualize in RVIZ
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Google logging
#include <glog/logging.h>

// FSW utils
#include <ff_util/ff_names.h>

// FSW messages
#include <ff_msgs/VisualeyezConfig.h>
#include <ff_msgs/VisualeyezFeedbackArray.h>

// Visualeyez implementation
#include <visualeyez/visualeyez.h>

// C math
#include <cmath>

// C++ includes
#include <memory>

using ceres::CostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

namespace visualeyez {

// Marker
class Marker {
 public:
  Marker() {
    pos[0] = 0;
    pos[1] = 0;
    pos[2] = 0;
  }
  explicit Marker(Eigen::Vector3d const& _pos) {
    pos[0] = _pos(0);
    pos[1] = _pos(1);
    pos[2] = _pos(2);
  }
  explicit Marker(double x, double y, double z) {
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
  }
  virtual ~Marker() {}
  double pos[3];
};
typedef std::map < VZIndex, Marker > MarkerMap;
typedef std::vector < Marker > ObservationVec;

// This is an error representing LED projections
class ProjectionError {
 public:
  // Constructor for this error function
  explicit ProjectionError(ObservationVec const& obs) : obs_(obs) {}

  // Called every solver iteration state = [t:3, R:3, X1:3 ... Xn:3]
  template < typename T >
  bool operator()(T const * const * states, T* res) const {
    // Transform the measurements
    T x_rot[3];
    for (size_t i = 0; i < obs_.size(); i++) {
      ceres::AngleAxisRotatePoint(states[1], states[2+i], x_rot);
      res[3*i+0] = (x_rot[0] + states[0][0] - obs_[i].pos[0]);
      res[3*i+1] = (x_rot[1] + states[0][1] - obs_[i].pos[1]);
      res[3*i+2] = (x_rot[2] + states[0][2] - obs_[i].pos[2]);
    }
    return true;
  }

  // Add a residual block for the given observation
  static ceres::CostFunction* Create(ObservationVec const& obs) {
    ceres::DynamicAutoDiffCostFunction<ProjectionError, 3> *cost_func =
      new ceres::DynamicAutoDiffCostFunction<ProjectionError, 3> (new ProjectionError(obs));
    cost_func->AddParameterBlock(3);              // Translation
    cost_func->AddParameterBlock(3);              // Rotation
    for (size_t i = 0; i < obs.size(); i++)
      cost_func->AddParameterBlock(3);            // Estimated position of marker obs(i)
    cost_func->SetNumResiduals(3 * obs.size());   // Residual error for the observations
    return static_cast<ceres::CostFunction*> (cost_func);
  }

 private:
  ObservationVec obs_;
};

// Possible states the system can be in
enum VisualeyezServerState {
  GROUNDING,                        // Occurs once before
  TRACKING,                         // Tracking in progress
  RECORDING,                        // Recording in progress
  CALIBRATING,                      // Calibration in progress
};

class VisualeyezServer : public nodelet::Nodelet {
 public:
  VisualeyezServer() : state_(GROUNDING) {
    // Initialize Google Logging
    google::InitGoogleLogging("visualeyez_calibrate");
    // Read the config file to get the targets
    config_reader::ConfigReader config_params;
    config_params.AddFile("tools/visualeyez.config");
    if (!config_params.ReadFiles())
      ROS_FATAL("Couldn't read config file");
    // Publish a TF2 message with the truth of the robot
    if (!config_params.GetBool("pub_tf", &pub_tf_))
      ROS_FATAL("Could not get the put_tf parameters");
    // Groundings
    config_reader::ConfigReader::Table table;
    if (!config_params.GetTable("grounding", &table))
      ROS_FATAL("Could not get the grounding parameters");
    grounding_ = VisualeyezUtils::GetMarkers(&table);
    if (grounding_.empty())
      ROS_FATAL("Could not read the grounding data");
    // Read the calibration file
    std::string cfgfile;
    if (!config_params.GetStr("calibration_file", &cfgfile))
      ROS_FATAL("Could not read the calibration file");
    // Read the target information
    if (!config_params.GetTable("targets", &table))
      ROS_FATAL("Could not get the targets parameters");
    for (int i = 0; i < table.GetSize(); i++) {
      config_reader::ConfigReader::Table group, mtable;
      if (!table.GetTable(i + 1, &group))
        ROS_FATAL("Could not read parameter table row");
      std::string name;
      if (!group.GetStr("name", &name))
        ROS_FATAL("Could not read parameter: name");
      if (!group.GetTable("markers", &mtable))
        ROS_FATAL("Could not read parameter: markers");
      targets_[name] = VisualeyezUtils::GetMarkers(&mtable);
      if (targets_[name].empty())
        ROS_FATAL("Could not get parse target matches");
    }
    // Read the calibration for the given set of targets
    if (!VisualeyezUtils::ReadConfig(cfgfile, targets_))
      ROS_INFO("No calibration file");
  }
  virtual ~VisualeyezServer() {}

 protected:
  // Called when nodelet boots
  void onInit() {
    sub_ = getNodeHandle().subscribe(TOPIC_VISUALEYEZ_DATA, 10,
      &VisualeyezServer::DataCallback, this);
    pub_feedback_ = getNodeHandle().advertise < ff_msgs::VisualeyezFeedbackArray > (
      TOPIC_VISUALEYEZ_CALIBRATE_FEEDBACK, 1, false);
    pub_grounding_ = getNodeHandle().advertise < visualization_msgs::MarkerArray > (
      TOPIC_VISUALEYEZ_RVIZ_GROUNDING, 1, true);
    pub_markers_ = getNodeHandle().advertise < visualization_msgs::MarkerArray > (
      TOPIC_VISUALEYEZ_RVIZ_MARKERS, 1, false);
    // Add a publisher for every target
    for (VZTargetMarkers::iterator it = targets_.begin(); it != targets_.end(); it++) {
      pub_truth_[it->first] = getNodeHandle().advertise < geometry_msgs::PoseStamped > (
        std::string("/") + it->first + std::string("/") + TOPIC_LOCALIZATION_TRUTH, 1, false);
    }
  }

  // Simple callback to store a data sample
  void DataCallback(const ff_msgs::VisualeyezDataArray::ConstPtr& msg) {
    // If we are busy calibrating, then ignore the message
    if (state_ == CALIBRATING)
      return;

    /////////////////////////////////////////////
    // Separate into ground and target markers //
    /////////////////////////////////////////////

    // Collect the grounding and target matches
    VZMatches grounding_matches;
    VZTargetMatches target_matches;
    // Iterate over the number of rows of raw data
    VZMarkers measurement;
    for (size_t i = 0; i < msg->measurements.size(); i++) {
      // We are only interested in valid measurements
      if (!VisualeyezUtils::IsValidMarkerValue(msg->measurements[i].position))
        continue;
      // Get the VZIndex for this measurement
      VZIndex index(msg->measurements[i].tcmid, msg->measurements[i].ledid);
      // Now, set the value of the point
      measurement[index](0) = msg->measurements[i].position.x;
      measurement[index](1) = msg->measurements[i].position.y;
      measurement[index](2) = msg->measurements[i].position.z;
      // Record all calibration match indices
      if (grounding_.find(index) != grounding_.end())
        grounding_matches.push_back(index);
      // Record all target match indices
      for (VZTargetMarkers::iterator it = targets_.begin(); it != targets_.end(); it++) {
        if (it->second.find(index) != it->second.end())
          target_matches[it->first].push_back(index);
      }
    }

    ////////////////////
    // Grounding mode //
    ////////////////////

    if (state_ == GROUNDING) {
      if (grounding_matches.size() < 3)
        return;
      // Preallocate correspondence matrices
      Eigen::Matrix3Xd ptsI(3, grounding_matches.size());
      Eigen::Matrix3Xd ptsO(3, grounding_matches.size());
      // Extract the correct measurements from the raw data
      for (size_t i = 0; i < grounding_matches.size(); i++) {
        ptsI.block<3, 1>(0, i) = measurement[grounding_matches[i]];   // Measured position
        ptsO.block<3, 1>(0, i) = grounding_[grounding_matches[i]];    // Truthful position
      }
      // Find the transform that explains the observation
      if (VisualeyezUtils::Kabsch <double> (ptsI, ptsO, transform_, true)) {
        // Shift to tracking state
        state_ = TRACKING;
        // Only allow configuration now
        srv_ = getNodeHandle().advertiseService(SERVICE_VISUALEYEZ_CALIBRATE_CONFIG,
          &VisualeyezServer::ConfigureCallback, this);
        // Publish the grounding to RVIZ
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < grounding_matches.size(); i++) {
          visualization_msgs::Marker marker;
          marker.header.frame_id = "world";
          marker.header.stamp = ros::Time::now();
          marker.ns = "visualeyez_grounding";
          marker.id = i;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = ptsO(0, i);
          marker.pose.position.y = ptsO(1, i);
          marker.pose.position.z = ptsO(2, i);
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = 0.1;
          marker.scale.y = 0.1;
          marker.scale.z = 0.1;
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0;
          marker.lifetime = ros::Duration();
          marker_array.markers.push_back(marker);
        }
        pub_grounding_.publish(marker_array);
      }
    }

    ///////////////////
    // Tracking mode //
    ///////////////////

    if (state_ == TRACKING) {
      // Loop over only the targets having markers that were actually in view
      for (VZTargetMatches::iterator it = target_matches.begin(); it != target_matches.end(); it++) {
        if (it->second.size() < 3)
          continue;
        // Get correspondence matrices
        Eigen::Matrix3Xd ptsI(3, it->second.size());
        Eigen::Matrix3Xd ptsO(3, it->second.size());
        for (size_t i = 0; i < it->second.size(); i++) {
          ptsI.block<3, 1>(0, i) = targets_[it->first][it->second[i]];        // WORLD REFERENCE
          ptsO.block<3, 1>(0, i) = transform_ * measurement[it->second[i]];   // VISUALEYEZ -> WORLD
        }
        // Execute the Kabsch algorithm on the measurements/truths to find a transform,
        // from WORLD TO BODY, disallowing the scale component of the transform.
        Eigen::Affine3d transform;
        if (VisualeyezUtils::Kabsch <double> (ptsI, ptsO, transform, false)) {
          // Extract the translation and rotation components
          Eigen::Vector3d translation(transform.translation());
          Eigen::Quaterniond rotation(transform.rotation());
          // Publish a ground truth for this robot
          if (pub_tf_) {
            static tf2_ros::TransformBroadcaster tf;
            static geometry_msgs::TransformStamped msg_tf;
            msg_tf.header.stamp = msg->header.stamp;
            msg_tf.header.frame_id = "world";
            msg_tf.child_frame_id = it->first + "/truth";
            msg_tf.transform.translation.x = translation(0);
            msg_tf.transform.translation.y = translation(1);
            msg_tf.transform.translation.z = translation(2);
            msg_tf.transform.rotation.w = rotation.w();
            msg_tf.transform.rotation.x = rotation.x();
            msg_tf.transform.rotation.y = rotation.y();
            msg_tf.transform.rotation.z = rotation.z();
            tf.sendTransform(msg_tf);
          }
          // Publish the pose on the robot truth topic
          if (pub_truth_.find(it->first) != pub_truth_.end()) {
            static geometry_msgs::PoseStamped msg_ps;
            msg_ps.header.stamp = msg->header.stamp;
            msg_ps.header.frame_id = "world";
            msg_ps.pose.position.x = translation(0);
            msg_ps.pose.position.y = translation(1);
            msg_ps.pose.position.z = translation(2);
            msg_ps.pose.orientation.w = rotation.w();
            msg_ps.pose.orientation.x = rotation.x();
            msg_ps.pose.orientation.y = rotation.y();
            msg_ps.pose.orientation.z = rotation.z();
            pub_truth_[it->first].publish(msg_ps);
          } else {
            NODELET_WARN_STREAM("Found pose but no publisher for " << it->first);
          }
          // Publish the complete marker set to RVIZ in RED
          size_t counter = 0;
          Eigen::Matrix3Xd ptsA(3, targets_[it->first].size());
          for (VZMarkers::iterator jt = targets_[it->first].begin(); jt != targets_[it->first].end(); jt++)
            ptsA.block<3, 1>(0, counter++) = jt->second;
          ptsA = transform * ptsA;
          visualization_msgs::MarkerArray marker_array;
          for (size_t i = 0; i < targets_[it->first].size(); i++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "visualeyez_" + it->first;
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = ptsA(0, i);
            marker.pose.position.y = ptsA(1, i);
            marker.pose.position.z = ptsA(2, i);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            marker_array.markers.push_back(marker);
          }
          pub_markers_.publish(marker_array);
        }
      }
    }

    ////////////////////
    // Recording mode //
    ////////////////////
    if (state_ == RECORDING) {
      // Create an observation of the target from the full set of measurements
      VZMarkers observation;
      for (VZMatches::iterator it = target_matches[name_].begin(); it != target_matches[name_].end(); it++)
        observation[*it] = transform_ * measurement[*it];
      // Only add the measurement if there are three or more correspondences
      if (observation.size() < 3)
        return;
      measurements_.push_back(observation);
      for (VZMarkers::iterator it = observation.begin(); it != observation.end(); it++)
        count_[it->first]++;

      // Keep track of how many valid measurements were received so that the
      // callee knows when to start calibration
      ff_msgs::VisualeyezFeedbackArray msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "visualeyez";
      for (VZMarkers::iterator it = targets_[name_].begin(); it != targets_[name_].end(); it++) {
        ff_msgs::VisualeyezFeedback marker;
        marker.tcmid = it->first.first;
        marker.ledid = it->first.second;
        if (count_.find(it->first) != count_.end())
          marker.count = count_[it->first];
        msg.feedback.push_back(marker);
      }
      pub_feedback_.publish(msg);

      // Publish markers to RVIZ
      visualization_msgs::MarkerArray marker_array;
      size_t counter = 0;
      for (VZMarkers::iterator it = targets_[name_].begin(); it != targets_[name_].end(); it++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "visualeyez_" + name_;
        marker.id = counter++;
        marker.type = visualization_msgs::Marker::SPHERE;
        if (observation.find(it->first) == observation.end()) {
          marker.action = visualization_msgs::Marker::DELETE;
        } else {
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = observation[it->first](0);
          marker.pose.position.y = observation[it->first](1);
          marker.pose.position.z = observation[it->first](2);
        }
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker_array.markers.push_back(marker);
      }
      pub_markers_.publish(marker_array);
    }
  }

  // Calibrate callback simply puts the system back in calibration mode
  bool ConfigureCallback(ff_msgs::VisualeyezConfig::Request &req, ff_msgs::VisualeyezConfig::Response &res) {
    // Special case: avoid configuring before gorounding
    if (state_ == GROUNDING) {
      ROS_WARN_STREAM("Configuration cannot be performed until grounding has completed");
      return false;
    }
    // Toggle the TF2 publisher if needed
    pub_tf_ = req.pub_tf;
    // Grounding has completed, so we can now support actions
    switch (req.action) {
    default:
      return false;
    // Start recording data
    case ff_msgs::VisualeyezConfig::Request::TRACK:
      ROS_WARN_STREAM("Return to target tracking mode");
      state_ = TRACKING;
      return true;
    // Start recording data
    case ff_msgs::VisualeyezConfig::Request::RECORD:
      if (targets_.find(req.name) == targets_.end()) {
        ROS_WARN_STREAM("Cannot find target " << req.name);
        return false;
      }
      ROS_INFO_STREAM("Recording data for target " << req.name);
      name_ = req.name;
      count_.clear();
      markers_.clear();
      measurements_.clear();
      state_ = RECORDING;
      return true;
    // Load calibration data
    case ff_msgs::VisualeyezConfig::Request::LOAD:
      if (!VisualeyezUtils::ReadConfig(req.name, targets_)) {
        ROS_INFO_STREAM("Calibration file " << req.name << " could not be read");
        return false;
      }
      ROS_INFO_STREAM("Reading calibration from file " << req.name);
      return true;
    // Save calibration data
    case ff_msgs::VisualeyezConfig::Request::SAVE:
      if (!VisualeyezUtils::WriteConfig(req.name, targets_))
        return false;
      ROS_INFO_STREAM("Writing calibration to " << req.name);
      return true;
    // Perform calibration
    case ff_msgs::VisualeyezConfig::Request::CALIBRATE:
      ROS_INFO_STREAM("Calibrating target " << name_ << " using recorded data");
      state_ = CALIBRATING;
      break;
    }

    // We will be solving for the transform (to the world rotation origin), relative marker positions
    // and the rotation from the reference positions to the observation frame
    ceres::Problem problem;
    double *transform = new double[3];
    double *rotations = new double[3*measurements_.size()];
    size_t counter = 0;

    // Bootstrap the transform center
    counter = 0;
    Eigen::Vector3d centroid;
    for (std::vector<VZMarkers>::iterator it = measurements_.begin(); it != measurements_.end(); it++) {
      for (VZMarkers::iterator jt = it->begin(); jt != it->end(); jt++) {
        centroid += jt->second;
        counter++;
      }
    }
    centroid /= counter;
    transform[0] = centroid(0);
    transform[1] = centroid(1);
    transform[2] = centroid(2);

    // Create a ceres problem to solve for the relative marker coordinates
    Eigen::Affine3d R = Eigen::Affine3d::Identity();
    for (size_t i = 0; i < measurements_.size(); i++) {
      ObservationVec observations;
      std::vector < double* > parameters;
      parameters.push_back(transform);
      parameters.push_back(&rotations[3*i]);
      for (VZMarkers::iterator jt = measurements_[i].begin(); jt != measurements_[i].end(); jt++) {
        parameters.push_back(markers_[jt->first].pos);
        observations.push_back(Marker(jt->second));
      }
      problem.AddResidualBlock(ProjectionError::Create(observations), new ceres::HuberLoss(1.0), parameters);
      if (i > 0) {
        // Get the correspondences with the last frame
        std::vector<VZIndex> correspondences;
        for (VZMarkers::iterator jt = measurements_[i-1].begin(); jt != measurements_[i-1].end(); jt++)
          if (measurements_[i].find(jt->first) != measurements_[i].end())
            correspondences.push_back(jt->first);
        if (correspondences.size() > 3) {
          Eigen::Matrix < double, 3, Eigen::Dynamic > prev(3, correspondences.size());
          Eigen::Matrix < double, 3, Eigen::Dynamic > curr(3, correspondences.size());
          for (size_t j = 0; j < correspondences.size(); j++) {
            prev.block<3, 1>(0, j) = measurements_[i-1][correspondences[j]];
            curr.block<3, 1>(0, j) = measurements_[i][correspondences[j]];
          }
          // Perform a KABSCH transform on the two matrices
          Eigen::Affine3d A;
          if (VisualeyezUtils::Kabsch < double > (prev, curr, A, false))
            R = R.rotate(A.linear());
        }
      }
      Eigen::AngleAxisd aa(R.linear());
      rotations[3*i+0] = aa.angle() * aa.axis()(0);
      rotations[3*i+1] = aa.angle() * aa.axis()(1);
      rotations[3*i+2] = aa.angle() * aa.axis()(2);
    }

    // We fix the first rotation to (0,0,0)
    problem.SetParameterBlockConstant(rotations);

    // Solve the problem
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    // options.function_tolerance = 1e-9;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    if (!summary.IsSolutionUsable()) {
      ROS_WARN("The solver completed but no solution was found");
    } else {
      ROS_WARN("The solver found a solution");
      for (VZMarkers::iterator it = targets_[name_].begin(); it != targets_[name_].end(); it++) {
        ROS_INFO_STREAM("[" << (int)it->first.first << ":" << (int)it->first.second << "] "
          << markers_[it->first].pos[0] - it->second(0) << ", "
          << markers_[it->first].pos[1] - it->second(1) << ", "
          << markers_[it->first].pos[2] - it->second(2));
        it->second = Eigen::Vector3d(
          markers_[it->first].pos[0],    // POS_X
          markers_[it->first].pos[1],    // POS_Y
          markers_[it->first].pos[2]);   // POS_Z
      }
    }

    // Print out the final centroid guess
    ROS_INFO_STREAM("Centroid initial guess: " << centroid);
    centroid(0) = transform[0];
    centroid(1) = transform[1];
    centroid(2) = transform[2];
    ROS_INFO_STREAM("Centroid final guess: " << centroid);

    // Clear up memory
    delete[] rotations;
    delete[] transform;

    // return to tracking mode
    state_ = TRACKING;

    // Make for great success
    return true;
  }

 private:
  // Publish a TF2 message
  bool pub_tf_;                                       // Publish a TF2 message?
  // ROS stuff
  ros::Subscriber sub_;                               // Data callback
  ros::ServiceServer srv_;                            // Trigger service
  ros::Publisher pub_grounding_;                      // Grounding
  ros::Publisher pub_markers_;                        // Markers
  ros::Publisher pub_feedback_;                       // Counter feedback
  std::map<std::string, ros::Publisher> pub_truth_;   // Truth publisher (1 per target)
  // System state
  VisualeyezServerState state_;                       // What state are we in
  std::string name_;                                  // Name of the target currently being tracked
  // Targets and grounding
  VZTargetMarkers targets_;                           // The targets we wish to track
  VZMarkers grounding_;                               // Grounding points
  std::vector < VZMarkers > measurements_;            // Measurements
  // Calibration-specific variables
  MarkerMap markers_;                                 // A list of markers
  std::map<VZIndex, uint32_t>  count_;                // Keep track of the valid marker count
  // Projection from VISUALEYEZ -> WORLD frame
  Eigen::Affine3d transform_;                         // Transform: VISUALEYEZ -> WORLD
};

// Register the nodelet witht he system
PLUGINLIB_DECLARE_CLASS(visualeyez, VisualeyezServer,
                        visualeyez::VisualeyezServer, nodelet::Nodelet);

}  // namespace visualeyez

#endif  // VISUALEYEZ_VISUALEYEZ_SERVER_H_

