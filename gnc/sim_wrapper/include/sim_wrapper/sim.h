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

#ifndef SIM_WRAPPER_SIM_H_
#define SIM_WRAPPER_SIM_H_

#include <camera/camera_params.h>
#include <config_reader/config_reader.h>
#include <gnc_autocode/sim.h>
#include <gnc_autocode/sim_csv.h>
#include <ff_hw_msgs/PmcCommand.h>
#include <ff_hw_msgs/PmcTelemetry.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/DepthLandmarks.h>
#include <ff_msgs/VisualLandmarks.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <ff_msgs/SetBool.h>

#include <sensor_msgs/Imu.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/time.h>

#include <mutex>

namespace sim_wrapper {
class Sim {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Sim(ros::NodeHandle *nh);
  ~Sim();
  void Step();

 protected:
  void PmcFamCallBack(ff_hw_msgs::PmcCommand::ConstPtr const& pmc);

  /**
   * Publish the ground truth location
   */
  void PublishTruth();
  /**
   * pull imu message values from gnc sim into ros data type
   * @return true if value is updated and should be published
   */
  bool PullImuMsg();

  /**
   * pull pmc state message value from gnc sim into ros data type
   * @return true if value is updated and should be published
   */
  bool PullPmcMsg();

  /**
   * pull the registration pulse, look for a landmark blip
   */
  bool PullLandmarkPulseMsg();

  /**
   * pull out the landmarks that have been generated.
   */
  bool PullLandmarkMsg();

  /**
   * callback to enable or disable the mapped landmark localization
   */
  bool LandmarkEnable(ff_msgs::SetBool::Request &req, ff_msgs::SetBool::Response &res);

 /**
   * pull the registration pulse, look for ar tag blip
   */
  bool PullTagPulseMsg();

  /**
   * pull out the ar tag landmarks that have been generated.
   */
  bool PullTagMsg();

  /**
   * callback to enable or disable the AR tag localization
   */
  bool TagEnable(ff_msgs::SetBool::Request &req, ff_msgs::SetBool::Response &res);

  /**
   * pull the registration pulse, look for a landmark blip
   */
  bool PullOpticalPulseMsg();

  /**
   * pull out the landmarks that have been generated.
   */
  bool PullOpticalMsg();

  /**
   * callback to enable or disable the optical flow
   */
  bool OpticalEnable(ff_msgs::SetBool::Request &req, ff_msgs::SetBool::Response &res);

  /**
   * pull out a depth image.
   */
  bool PullDepthMsg();

  /**
   * pull the registration pulse, look for depth blip
   */
  bool PullDepthPulseMsg();

    /**
   * callback to enable or disable depth localization
   */
  bool DepthEnable(ff_msgs::SetBool::Request &req, ff_msgs::SetBool::Response &res);


  void ReadParams(void);

  gnc_autocode::GncSimAutocode gnc_;

  config_reader::ConfigReader config_;
  ros::Timer config_timer_;
  camera::CameraParameters cam_params_;

  // Pre-allocated message structure so we don't hit the allocator.
  sensor_msgs::Imu ros_imu_;
  ff_hw_msgs::PmcTelemetry ros_pmc_;
  ff_msgs::Feature2dArray ros_optical_;
  ff_msgs::VisualLandmarks ros_landmark_;
  ff_msgs::VisualLandmarks ros_tag_;
  ff_msgs::DepthLandmarks ros_depth_;
  ff_msgs::CameraRegistration ros_reg_pulse_;
  geometry_msgs::PoseStamped ros_truth_pose_;
  geometry_msgs::TwistStamped ros_truth_twist_;
  geometry_msgs::AccelStamped ros_truth_accel_;

  // variables used to detect change
  ros::Time cur_time_, last_landmark_time_, last_tag_time_, last_optical_time_, last_depth_time_;
  bool last_landmark_pulse_, last_tag_pulse_, last_optical_pulse_, last_depth_pulse_;
  unsigned int ml_camera_count_, ar_camera_count_, of_camera_count_, dl_camera_count_;
  bool en_landmark_, en_tag_, en_optical_, en_depth_;

  int pmc_command_id_;

  size_t ml_max_features_, ar_max_features_, hr_max_features_, of_history_size_, of_max_features_;
  ros::Publisher pub_imu_, pub_pmc_, pub_clock_, pub_landmark_pulse_,
    pub_landmarks_, pub_optical_pulse_, pub_optical_,  pub_truth_pose_,
    pub_depth_, pub_depth_pulse_, pub_landmark_camera_, pub_ar_tags_camera_,
    pub_ar_tags_pulse_, pub_ar_tags_, pub_depth_camera_;
  ros::Subscriber sub_fam_, sub_fam_pmc_;
  ros::ServiceServer srv_landmark_enable_, srv_optical_enable_, srv_ar_tags_enable_, srv_depth_enable_;
};
}  // end namespace sim_wrapper

#endif  // SIM_WRAPPER_SIM_H_
