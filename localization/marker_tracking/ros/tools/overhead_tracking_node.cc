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

#include <ff_common/init.h>
#include <config_reader/config_reader.h>
#include <geometry_msgs/PoseStamped.h>
#include <marker_tracking/arxmlio.h>
#include <msg_conversions/msg_conversions.h>

#include <ff_util/ff_names.h>

#ifndef CV__ENABLE_C_API_CTORS
#define CV__ENABLE_C_API_CTORS
#endif
#include <opencv2/imgproc.hpp>

#include <alvar/Camera.h>
#include <alvar/MarkerDetector.h>

#include <Eigen/Geometry>

// Whole bunch of ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class RosMarkerTrackingAdaptor {
  config_reader::ConfigReader config_;
  ros::Timer config_timer_;
  ros::Publisher vo_publisher_;
  std::shared_ptr<alvar::Camera> camera_;
  alvar::MarkerDetector<alvar::MarkerData> detector_;
  std::string cam_filename_;
  std::string ar_tag_xml_filename_;
  marker_tracking::ARTagMap ar_tag_map_;
  Eigen::Affine3d body_T_ar_, world_T_cam_;
  double marker_height_;

 public:
  explicit RosMarkerTrackingAdaptor(ros::NodeHandle *nh, ros::NodeHandle *priv_nh) :
            detector_() {
    config_.AddFile("geometry.config");
    config_.AddFile("overhead_tracker.config");
    ReadParams();
    config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&RosMarkerTrackingAdaptor::ReadParams, this));
    }, false, true);

    vo_publisher_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_LOCALIZATION_TRUTH, 1);

    // Load up te AR tags
    marker_tracking::LoadARTagLocation(ar_tag_xml_filename_, &ar_tag_map_);
  }

  void ReadParams(void) {
    if (!config_.ReadFiles()) {
      ROS_ERROR("Failed to read config files.");
      return;
    }

    if (!msg_conversions::config_read_transform(&config_, "ar_tag_transform", &body_T_ar_))
      ROS_FATAL("Unspecified body to AR tag transform.");
    if (!msg_conversions::config_read_transform(&config_, "overhead_cam_transform", &world_T_cam_))
      ROS_FATAL("Unspecified overhead camera transform.");
    if (!config_.GetReal("marker_height", &marker_height_))
      ROS_FATAL("Unspecified marker height.");

    std::string path, filename = "";
    path = ros::package::getPath("marker_tracking");
    if (!config_.GetStr("ar_tag_xml", &filename)) {
      ROS_FATAL("Overhead tracking: ar tag xml file not specified!");
    }
    ar_tag_xml_filename_ = path + "/" + filename;

    filename = "";
    if (!config_.GetStr("cam", &filename)) {
      ROS_FATAL("Overhead tracking: cam file not specified!");
    }
    cam_filename_ = path + "/" + filename;
  }

  void PublishVO(ros::Time const& timestamp) {
    // Identify how many AR tags were detected that we have locations recorded for them.
    size_t ar_tags_w_position = 0;
    for (size_t i = 0; i < detector_.markers->size(); i++) {
      if (ar_tag_map_.count(detector_.markers->operator[](i).GetId())) {
        ar_tags_w_position++;
      }
    }

    if (!ar_tags_w_position) {
      ROS_DEBUG_THROTTLE(5, "Failed to find AR tags with known prior position.");
      return;
    }

    ROS_DEBUG_THROTTLE(5, "Reduced found markers to %u", static_cast<unsigned int>(ar_tags_w_position));

    // TODO(zmoratto): Make these member variables?
    std::vector<cv::Point2f> image_points(ar_tags_w_position * 4);
    std::vector<cv::Point3f> world_points(ar_tags_w_position * 4);

    // Copy into the solving structure
    size_t insert_idx = 0;

    for (size_t i = 0; i < detector_.markers->size(); i++) {
      auto const& marker = detector_.markers->operator[](i);
      auto const& corners_it =
        ar_tag_map_.find(marker.GetId());
      if (corners_it != ar_tag_map_.end()) {
        for (size_t j = 0; j < 4; j++) {
          world_points[insert_idx].x = corners_it->second.row(j)[0];
          world_points[insert_idx].y = corners_it->second.row(j)[1];
          world_points[insert_idx].z = corners_it->second.row(j)[2];
          image_points[insert_idx].x = marker.marker_corners_img[j].x;
          image_points[insert_idx].y = marker.marker_corners_img[j].y;
          insert_idx++;
        }
      }
    }
    assert(insert_idx == 4 * ar_tags_w_position);

    // Solve the PnP problem, OpenCV only works with doubles. Gah!
    cv::Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F);
    cv::Mat k(3, 3, CV_64F, camera_->calib_K_data),
    d(4, 1, CV_64F, camera_->calib_D_data);
    cv::solvePnPRansac(world_points, image_points, k, d, rvec, tvec);

    // Extract the solution which is camera_T_ar
    Eigen::Vector3d rodrigues;
    rodrigues << rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2);
    Eigen::Quaterniond camera_q_ar(Eigen::AngleAxisd(rodrigues.norm(),
                                   rodrigues.normalized()));


    // Oleg's fine tuning approach with fixed P3 height and z direction assumption
    // Improve camera_T_ar based on the fact that we know
    // that it must point straight down and its distance to
    // the granite table.
    camera_q_ar.z() = 0;
    camera_q_ar.w() = 0;
    camera_q_ar.normalize();
    // Center of the AR tag in tag's coordinate system
    // (it is obviously 0,0,0).
    Eigen::Vector3d ctr;
    ctr[0] = (world_points[0].x + world_points[2].x)/2.0;
    ctr[1] = (world_points[0].y + world_points[2].y)/2.0;
    ctr[2] = (world_points[0].z + world_points[2].z)/2.0;
    Eigen::Vector3d C(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    double ht = fabs(world_T_cam_.translation()(2)) - marker_height_;
    double t = (ht - ctr[2])/(C[2] - ctr[2]);

    // Move along ray
    C = t*(C-ctr) + ctr;
    Eigen::Affine3d camera_T_ar = Eigen::Affine3d(
                                    Eigen::Translation3d(C[0], C[1], C[2])) *
                                  Eigen::Affine3d(camera_q_ar);

    // Calculate the transform that is world_T_body
    Eigen::Affine3d world_T_body = world_T_cam_ * camera_T_ar * body_T_ar_.inverse();

    geometry_msgs::PoseStamped odo_msg;
    odo_msg.header.stamp = timestamp;
    odo_msg.header.frame_id = "world";
    Eigen::Quaterniond quat(world_T_body.rotation());
    odo_msg.pose.orientation = msg_conversions::eigen_to_ros_quat(quat);
    odo_msg.pose.position = msg_conversions::eigen_to_ros_point(world_T_body.translation());
    vo_publisher_.publish(odo_msg);
  }

  void VideoCallback(const sensor_msgs::ImageConstPtr & image_msg) {
    // Convert the image
    auto cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    IplImage ipl_image = cvIplImage(cv_ptr->image);

    // Check that the camera has been loaded
    if (!camera_) {
      camera_.reset(new alvar::Camera());
      if (cam_filename_.empty()) {
        camera_->SetRes(ipl_image.width, ipl_image.height);
      } else {
        camera_->SetCalib(cam_filename_.c_str(), ipl_image.width, ipl_image.height);
      }
    }

    // Detect our AR Tags
    if (detector_.Detect(&ipl_image, camera_.get(), true) > 0) {
      // If we got markers .. let's publish an estimate
      PublishVO(image_msg->header.stamp);
    }
  }
};

int main(int argc, char *argv[]) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  // Initialize ROS
  ros::init(argc, argv, "overhead_tracking");

  // Attach our publisher and subscriber
  ros::NodeHandle nh, priv_nh("~");
  RosMarkerTrackingAdaptor adaptor(&nh, &priv_nh);
  image_transport::ImageTransport it(nh);
  auto cam_sub = it.subscribe(TOPIC_LOCALIZATION_OVERHEAD_IMAGE, 1,
                              &RosMarkerTrackingAdaptor::VideoCallback, &adaptor);

  ros::spin();

  return 0;
}
