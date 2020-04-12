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

#ifndef EKF_EKF_WRAPPER_H_
#define EKF_EKF_WRAPPER_H_

#include <ekf/ekf.h>

#include <Eigen/Geometry>
#include <config_reader/config_reader.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/DepthLandmarks.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/SetEkfInput.h>
#include <ff_msgs/VisualLandmarks.h>
#include <ff_msgs/FlightMode.h>
#include <ff_util/perf_timer.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <atomic>
#include <condition_variable> // NOLINT
#include <list>
#include <mutex>
#include <map>
#include <string>
#include <vector>
#include <functional>

namespace ekf {

/**
 * @brief ROS wrapper around EKF GNC code.
 * @details ROS wrapper around EKF GNC code.
 */
class EkfWrapper {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit EkfWrapper(ros::NodeHandle* nh, std::string const& name);
  ~EkfWrapper();

  void Run(std::atomic<bool> const& killed);

  /**
   * This moves the EKF forward one time step
   * and publishes the EKF pose.
   *
   * Returns zero if EKF was not run, non-zero if it was.
   */
  int Step();

 protected:
  void ReadParams(void);
  /**
   * Initialize services and topics besides IMU.
   **/
  void InitializeEkf(void);
  /**
   * Publishes a ROS message containing the state of the EKF.
   **/
  void PublishState(const ff_msgs::EkfState & state);

  /**
   * Resets the EKF.
   **/
  bool ResetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);  //NOLINT

  bool ResetHandrailService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**

   * Initializes the bias and saves the results to a file.
   **/
  bool InitializeBiasService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);  //NOLINT
  /**
   * Set the input mode between mapped landmarks, AR tags, and handrail.
   **/
  bool SetInputService(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res);  //NOLINT

  /**
   * Actually does the bias estimation, called from IMU callback.
   **/
  void EstimateBias(sensor_msgs::Imu::ConstPtr const& imu);

  /**
   * Callback functions. These all record the information
   * and then pass it on to the EKF the next time the Step function
   * is called.
   **/
  void ImuCallBack(sensor_msgs::Imu::ConstPtr const& imu);
  void OpticalFlowCallBack(ff_msgs::Feature2dArray::ConstPtr const& of);
  void VLVisualLandmarksCallBack(ff_msgs::VisualLandmarks::ConstPtr const& vl);
  void ARVisualLandmarksCallBack(ff_msgs::VisualLandmarks::ConstPtr const& vl);
  void DepthLandmarksCallBack(ff_msgs::DepthLandmarks::ConstPtr const& vl);
  void VLRegisterCamera(ff_msgs::CameraRegistration::ConstPtr const& cr);
  void ARRegisterCamera(ff_msgs::CameraRegistration::ConstPtr const& cr);
  void RegisterDepthCamera(ff_msgs::CameraRegistration::ConstPtr const& cr);
  void RegisterOpticalFlowCamera(ff_msgs::CameraRegistration::ConstPtr const& cr);
  void GroundTruthCallback(geometry_msgs::PoseStamped::ConstPtr const& truth);
  void GroundTruthTwistCallback(geometry_msgs::TwistStamped::ConstPtr const& truth);
  void FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode);

  /**
   * Callback when a subscriber joins or disconnects from the feature topic
   **/
  void SubscriberCallback();
  void PublishFeatures(ff_msgs::VisualLandmarks::ConstPtr const& l);
  void PublishFeatures(ff_msgs::DepthLandmarks::ConstPtr const& l);

  /**
   * Callback when the EKF resets
   **/
  void ResetCallback();

  /** Variables **/
  // the actual EKF
  Ekf ekf_;
  ff_msgs::EkfState state_;

  bool ekf_initialized_;

  // most recent imu message
  sensor_msgs::Imu imu_;
  geometry_msgs::Quaternion quat_;
  int imus_dropped_;

  // Truth messages
  geometry_msgs::PoseStamped truth_pose_;
  geometry_msgs::TwistStamped truth_twist_;

  // EKF wrapper needs to capture the first landmark message to do PnP before
  // it can initialize the EKF autocode and let it run. This variable holds
  // that state and is atomic so that it doesn't require a mutex.
  std::atomic<bool> have_imu_;
  std::atomic<int> input_mode_;

  /** Configuration Constants **/

  /** Ros **/
  config_reader::ConfigReader config_;
  ff_util::PerfTimer pt_ekf_;
  ros::Timer config_timer_;

  ros::NodeHandle* nh_;
  // topic subscribers
  ros::Subscriber imu_sub_, of_sub_, vl_sub_, ar_sub_, dl_sub_, truth_sub_, truth_twist_sub_;
  ros::Subscriber vl_reg_sub_, ar_reg_sub_, of_reg_sub_, dl_reg_sub_;
  ros::Subscriber flight_mode_sub_;

  // publisher
  ros::Publisher state_pub_;
  ros::Publisher feature_pub_;
  ros::Publisher pose_pub_, twist_pub_;
  ros::Publisher reset_pub_;
  tf2_ros::TransformBroadcaster transform_pub_;
  ros::ServiceServer reset_srv_, bias_srv_, input_mode_srv_, reset_hr_srv_;

  /** Threading **/

  // mutex for msgs, concurrency protection
  std::mutex mutex_act_msg_;
  std::mutex mutex_imu_msg_;
  std::mutex mutex_of_msg_;
  std::mutex mutex_vl_msg_;
  std::mutex mutex_dl_msg_;
  std::mutex mutex_truth_msg_;

  // cv to wait for an imu reading
  std::condition_variable cv_imu_;

  /** IMU Bias reset variables **/
  std::string bias_file_;
  bool estimating_bias_;
  float bias_reset_sums_[6];
  int bias_reset_count_;
  int bias_required_observations_;

  /** Feature drawing **/
  std::string platform_name_;
  bool disp_features_;
  sensor_msgs::PointCloud2 features_;

  // Prevents needing to call ros::ok() from a thread
  std::atomic<bool> killed_;
};

}  // end namespace ekf

#endif  // EKF_EKF_WRAPPER_H_
