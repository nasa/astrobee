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

#include <marker_tracking_node/marker_tracker.h>

#include <ff_msgs/CameraRegistration.h>
#include <ff_util/ff_names.h>
#include <marker_tracking/marker_detector.h>
#include <msg_conversions/msg_conversions.h>

#include <alvar/Marker.h>
#include <camera/camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <string>

namespace marker_tracking_node {

MarkerTracker::MarkerTracker(ros::NodeHandle* nh, ros::NodeHandle* private_nh,
                             const std::string& nm)
    : it_(*nh),
      nodelet_name_(nm),
      camera_param_(Eigen::Vector2i::Zero(),  // Here because I deleted the
                    Eigen::Vector2d::Ones(),  // default constructor
                    Eigen::Vector2d::Zero()) {
  config_.AddFile("cameras.config");
  config_.AddFile("localization/marker_tracker.config");
  ReadParams();

  // Resolve the full path to the AR tag file specified for the current world
  std::string ar_tag_file;
  if (!config_.GetStr("world_vision_ar_tag_filename", &ar_tag_file))
    ROS_ERROR("Cannot read world_vision_ar_tag_filename from LUA config");

  // Check to see if this is an XML or LUA file
  bool use_config_ar = true;
  if (ar_tag_file.substr(ar_tag_file.find_last_of(".") + 1) == "xml")
    use_config_ar = false;
  if (use_config_ar) {
    if (ar_tag_file.size() == 0) {
      ROS_FATAL("No AR tag specified.");
      exit(0);
    }
    config_.AddFile(ar_tag_file.c_str());
    ReadParams();
  }

  // Poll the config file for changes
  config_timer_ = nh->createTimer(
      ros::Duration(1),
      [this](ros::TimerEvent e) {
        config_.CheckFilesUpdated(std::bind(&MarkerTracker::ReadParams, this));
      },
      false, true);

  // Load mechanism depends on XML or config format
  if (!use_config_ar) {
    // Load using XML format:
    ROS_INFO_NAMED(nodelet_name_, "Loading AR markers from XML file: %s",
                   ar_tag_file.c_str());
    marker_tracking::LoadARTagLocation(ar_tag_file, &ar_tags_);
  } else {
    // Load using new Config format:
    ROS_INFO_NAMED(nodelet_name_, "Loading AR makers configuation from: %s",
                   ar_tag_file.c_str());
    marker_tracking::LoadARTagsConfig(&config_, &ar_tags_);
  }
  // Create our publishers
  landmark_publisher_ = nh->advertise<ff_msgs::VisualLandmarks>(
      TOPIC_LOCALIZATION_AR_FEATURES, 5);
  reg_publisher_ = nh->advertise<ff_msgs::CameraRegistration>(
      TOPIC_LOCALIZATION_AR_REGISTRATION, 5);

  enable_srv_ = nh->advertiseService(SERVICE_LOCALIZATION_AR_ENABLE,
                                     &MarkerTracker::EnableService, this);
}

MarkerTracker::~MarkerTracker(void) {}

void MarkerTracker::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }

  camera_param_ = camera::CameraParameters(&config_, "dock_cam");
  detector_.reset(new marker_tracking::MarkerCornerDetector(camera_param_));

  if (!config_.GetInt("marker_tracker_valid_x_min", &x_min_))
    ROS_FATAL("Unspecified marker_tracker_valid_x_min.");
  if (!config_.GetInt("marker_tracker_valid_x_max", &x_max_))
    ROS_FATAL("Unspecified marker_tracker_valid_x_max.");
}

bool MarkerTracker::EnableService(ff_msgs::SetBool::Request& req,
                                  ff_msgs::SetBool::Response& res) {
  if (req.enable)
    image_sub_ = it_.subscribe(TOPIC_HARDWARE_DOCK_CAM, 1,
                               &MarkerTracker::VideoCallback, this);
  else
    image_sub_.shutdown();
  res.success = true;
  return true;
}

int MarkerTracker::EstimatePose(ff_msgs::VisualLandmarks* msg) {
  // Count the number of landmarks that were given to the EKF, early exit if
  // there are not enough.
  if (msg->landmarks.size() < 4) return 1;

  // Extract the landmarks into a structure that can be used by OpenCV.
  std::vector<cv::Point3f> object_points(msg->landmarks.size());
  std::vector<cv::Point2f> image_points(msg->landmarks.size());
  for (unsigned int i = 0; i < msg->landmarks.size(); i++) {
    object_points[i].x = msg->landmarks[i].x;
    object_points[i].y = msg->landmarks[i].y;
    object_points[i].z = msg->landmarks[i].z;
    image_points[i].x = msg->landmarks[i].u;
    image_points[i].y = msg->landmarks[i].v;
  }

  // Extract the intrinsic matrix for our undistorted_c camera into the opencv
  // format.
  cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
  cv::eigen2cv(camera_param_.GetIntrinsicMatrix<camera::UNDISTORTED_C>(),
               camera_matrix);
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::Mat distortion(4, 1, cv::DataType<double>::type, cv::Scalar(0));
  cv::solvePnPRansac(object_points, image_points, camera_matrix, distortion,
                     rvec, tvec);
  Eigen::Vector3d pos;
  cv::cv2eigen(tvec, pos);
  Eigen::Matrix3d rotation;
  camera::RodriguesToRotation(
      Eigen::Vector3d(rvec.at<double>(0), rvec.at<double>(1),
                      rvec.at<double>(2)),
      &rotation);

  // Extract the transform that is world_T_cam
  Eigen::Vector3d rodrigues;
  rodrigues << rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2);
  Eigen::Affine3d cam_t_global;
  cam_t_global.setIdentity();
  cam_t_global.translate(pos);
  cam_t_global.rotate(rotation);
  cam_t_global = cam_t_global.inverse();  // actually world to cam

  Eigen::Quaterniond quat(cam_t_global.rotation());
  msg->pose.position =
      msg_conversions::eigen_to_ros_point(cam_t_global.translation());
  msg->pose.orientation = msg_conversions::eigen_to_ros_quat(quat);

  return 0;
}

bool MarkerTracker::IsMarkerValid(const alvar::MarkerData & marker) {
  marker_tracking::ARTagMap::const_iterator xyz_iter =
      ar_tags_.find(marker.GetId());
  if (xyz_iter == ar_tags_.end())
    return false;
  for (size_t j = 0; j < 4; j++) {
    Eigen::Vector2d p(marker.marker_corners_img[j].x,
                      marker.marker_corners_img[j].y);
    if (p.x() < x_min_ || p.x() > x_max_)
      return false;
  }
  return true;
}

/*
  This callback is called when an image is received.
  It tries to detect the markers and pass that information to process marker.
*/
void MarkerTracker::VideoCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  ff_msgs::CameraRegistration r;
  ros::Time timestamp = ros::Time::now();
  r.header = std_msgs::Header();
  r.header.stamp = timestamp;
  r.camera_id = 0;
  reg_publisher_.publish(r);
  ros::spinOnce();

  // Convert the image
  cv_bridge::CvImageConstPtr cv_ptr_ =
      cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
  IplImage ipl_image = cv_ptr_->image;

  // Detect our AR Tags
  detector_->Detect(&ipl_image, 0.08, 0.2);

  // No markers? Early exit.
  if (!detector_->NumMarkers()) return;

  // Identify how many of the AR tags we actually have a location for.
  size_t valid_marker_count = 0;
  // std::cout << "AR makers detected: ";
  for (size_t i = 0; i < detector_->NumMarkers(); i++) {
    if (IsMarkerValid(detector_->GetMarker(i)))
      valid_marker_count++;
  }
  // std::cout << "(n=" << valid_marker_count << ")" << std::endl;

  // Identify the AR tags that we have seen and transmit their positions.
  ff_msgs::VisualLandmarks msg;
  msg.header = std_msgs::Header();
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "dock/body";
  msg.camera_id = 0;
  msg.landmarks.resize(4 * valid_marker_count);
  size_t write_idx = 0;
  for (size_t i = 0; i < detector_->NumMarkers(); i++) {
    auto const& marker = detector_->GetMarker(i);
    if (!IsMarkerValid(marker))
      continue;
    marker_tracking::ARTagMap::const_iterator xyz_iter =
        ar_tags_.find(marker.GetId());
    if (xyz_iter != ar_tags_.end()) {
      for (size_t j = 0; j < 4; j++) {
        auto& landmark = msg.landmarks[write_idx];
        landmark.x = xyz_iter->second.row(j).x();
        landmark.y = xyz_iter->second.row(j).y();
        landmark.z = xyz_iter->second.row(j).z();

        // EKF expects measurements in the UNDISTORTED_C coordinate frame
        Eigen::Vector2d undistorted_c;
        camera_param_.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(
            Eigen::Vector2d(marker.marker_corners_img[j].x,
                            marker.marker_corners_img[j].y),
            &undistorted_c);
        landmark.u = undistorted_c[0];
        landmark.v = undistorted_c[1];

        write_idx++;
      }
    }
  }
  EstimatePose(&msg);
  landmark_publisher_.publish(msg);
}

};  // namespace marker_tracking_node
