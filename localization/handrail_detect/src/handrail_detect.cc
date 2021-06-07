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

// Core ROS include
#include <ros/ros.h>

// Shared includes
#include <ff_common/init.h>
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>
#include <ff_util/ff_nodelet.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// ROS messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <ff_msgs/SetBool.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/DepthLandmarks.h>

// TF2 includes
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>

// OpenCV/ROS includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Eigen incliudes
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>

// C++ STL includes
#include <algorithm>
#include <iterator>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

/**
 * \ingroup localization
 */
namespace handrail_detect {

enum handrailStatus {NOT_FOUND, BOTH_ENDS, FIRST_END, SECOND_END, NO_END};

class HandrailDetect : public ff_util::FreeFlyerNodelet {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  HandrailDetect() : ff_util::FreeFlyerNodelet(NODE_HANDRAIL_DETECT, true),
    curr_handrail_status_(NOT_FOUND), x_scale_(0.001), y_scale_(0.001), dist_to_handrail_(1.5),
    x_step_(2), y_step_(2), depth_width_(224), depth_height_(171), frame_count_(0), max_num_handrails_(1) {
    // Setup iostream precision
    std::cout << std::fixed;
    std::cout.precision(6);
  }
  ~HandrailDetect() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    // Store the node handle for future use
    nh_ = nh;
    // Read the configuration and setup timers
    config_.AddFile("localization/handrail_detect.config");
    config_.AddFile("geometry.config");
    ReadParams();
    config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&HandrailDetect::ReadParams, this));
    }, false, true);

    // Subscribe depth sensor data - the pico_driver is now subscriber-driven. This means that
    // we shouldn't have a subscriber active which we are not using, as we will needlessly
    // sample the camera, which costs ~10% of our CPU resources. So, we're going to create
    // an enable/disable service to do this for us.
    enable_srv_ = nh->advertiseService(SERVICE_LOCALIZATION_HR_ENABLE, &HandrailDetect::EnableCallback, this);

    // Publish registration
    registration_pub_ = nh->advertise<ff_msgs::CameraRegistration>(TOPIC_LOCALIZATION_HR_REGISTRATION, 1);

    // Publish landmarks
    landmark_pub_ = nh->advertise<ff_msgs::DepthLandmarks>(TOPIC_LOCALIZATION_HR_FEATURES, 1);

    // Publish markers for rviz visualization
    marker_pub_ = nh->advertise<visualization_msgs::Marker>(TOPIC_LOCALIZATION_HR_MARKER, 12);

    // transform from body frame to the handrail frame
    r2h_center_ = Eigen::Affine3f::Identity();
  }

  // Read configuration parameters from the LUA config file
  void ReadParams() {
    bool find_vertical_handrail;
    // Read config files into lua
    if (!config_.ReadFiles()) {
      ROS_ERROR("Error loading handrail detect parameters.");
      return;
    }

    // Display
    if (!config_.GetBool("disp_computation_time", &disp_computation_time_))
      ROS_FATAL("Unspecified disp_computation_time.");
    if (!config_.GetBool("disp_marker", &disp_marker_))
      ROS_FATAL("Unspecified disp_marker.");
    if (!config_.GetBool("disp_pcd_and_tf", &disp_pcd_and_tf_))
      ROS_FATAL("Unspecified disp_pcd_and_tf.");
    if (!config_.GetBool("disp_depth_image", &disp_depth_image_))
      ROS_FATAL("Unspecified disp_depth_image.");

    // This algorithm can detect both vertical (between 45deg ~ 90deg) and
    // horizontal (between 0 deg ~45 deg)handrails
    // For just detecting the vertical handrail, set max_num_handrails_ to 1
    // For detecting both directional handrails (and selecting the one with better shape),
    // set max_num_handrails_ to 2.
    // However, since we are going to detect only vertical one for Astrobee project,
    // find_vertical_handrail in the config file should be set to 'true'.
    if (!config_.GetBool("find_vertical_handrail", &find_vertical_handrail))
      ROS_FATAL("Unspecified find_vertical_handrail.");
    if (!find_vertical_handrail)
      max_num_handrails_ = 2;

    // Geometric distance related constants
    if (!config_.GetReal("close_dist_to_handrail", &close_dist_to_handrail_))
      ROS_FATAL("Unspecified close_dist_to_handrail.");
    if (!config_.GetReal("handrail_wall_min_gap", &handrail_wall_min_gap_))
      ROS_FATAL("Unspecified handrail_wall_min_gap.");
    if (!config_.GetReal("handrail_width", &handrail_width_))
      ROS_FATAL("Unspecified handrail_width.");
    if (!config_.GetReal("handrail_group_gap", &handrail_group_gap_))
      ROS_FATAL("Unspecified handrail_group_gap.");
    if (!config_.GetReal("handrail_length", &fix_handrail_length_))
      ROS_FATAL("Unspecified handrail_length.");

    // Minimum rate of the num of inliner points of the plane from the total num of points
    if (!config_.GetReal("pcd_plane_rate", &pcd_plane_rate_))
      ROS_FATAL("Unspecified pcd_plane_rate.");

    if (!config_.GetReal("width_filter_rate", &width_filter_rate_))
      ROS_FATAL("Unspecified width_filter_rate.");

    // Perching arm length to set the target perching pose
    if (!config_.GetReal("arm_length", &arm_length_))
      ROS_FATAL("Unspecified arm_length.");

    // Minimun distance in meter between neighboring pixels
    if (!config_.GetReal("min_measure_gap", &min_measure_gap_))
      ROS_FATAL("Unspecified min_measure_gap.");

    // Minimum z-axis distance
    if (!config_.GetReal("min_depth_dist", &min_depth_dist_))
      ROS_FATAL("Unspecified min_depth_dist.");

    // Maximum z-axis distance
    if (!config_.GetReal("max_depth_dist", &max_depth_dist_))
      ROS_FATAL("Unspecified max_depth_dist.");

    // Maximum RANSAC iteration for line estimation
    if (!config_.GetInt("RANSAC_line_iteration", &RANSAC_line_iteration_))
      ROS_FATAL("Unspecified RANSAC_line_iteration.");

    // RANSAC threshold for line estimation
    if (!config_.GetReal("RANSAC_line_thres", &RANSAC_line_thres_))
      ROS_FATAL("Unspecified RANSAC_line_thres.");

    // Maximum RANSAC iteration for plane estimation
    if (!config_.GetInt("RANSAC_plane_iteration", &RANSAC_plane_iteration_))
      ROS_FATAL("Unspecified RANSAC_plane_iteration.");

    // RANSAC threshold for plane estimation
    if (!config_.GetReal("RANSAC_plane_thres", &RANSAC_plane_thres_))
      ROS_FATAL("Unspecified RANSAC_plane_thres.");

    // Frame name of the perch cam
    if (!config_.GetStr("perch_image_frame", &perch_image_frame_))
      ROS_FATAL("Unspecified perch_image_frame.");

    // Frame name of the perch cam
    if (!config_.GetReal("outlier_tolerance", &outlier_tolerance_))
      ROS_FATAL("Unspecified outlier_tolerance.");

    // Get transform from the robot body to the perch cam in config
    Eigen::Affine3d tmp_r2i;
    if (!msg_conversions::config_read_transform(&config_, "perch_cam_transform", &tmp_r2i)) {
      ROS_FATAL("Unspecified body to perch_cam transform.");
      return;
    }

    // Transfrom from the robot body to the perch cam
    r2i_ = static_cast<Eigen::Affine3f>(tmp_r2i);

    ///////////////////////////////////////////////////////////////////////////////
    // Andrew moved the lines below from Initialize() to ensure that the variables
    // are changed whenever a LUA file update occurs.
    //////////////////////////////////////////////////////////////////////////////

    // We need to make this code platform-aware, so we don't collide on namespaces
    body_frame_ = FRAME_NAME_BODY;
    handrail_frame_ = FRAME_NAME_HANDRAIL;
    if (!GetPlatform().empty()) {
      body_frame_ = GetPlatform() + std::string("/") + body_frame_;
      handrail_frame_ = GetPlatform() + std::string("/") + handrail_frame_;
    }

    // Minimum rate of the num of inliner points of the plane from the total num of points
    pcd_plane_rate_ = std::max(static_cast<float>(0.1), pcd_plane_rate_);

    // Set the point cloud header frame
    cloud_.header.frame_id = perch_image_frame_;

    // Gap distance between the plane and the handrail
    ave_handrail_wall_gap_ = handrail_wall_min_gap_ + 1.5 * handrail_width_;

    // Setting for publishing depth image
    if (disp_depth_image_) {
      image_.header.frame_id = perch_image_frame_;
      image_.encoding = sensor_msgs::image_encodings::MONO8;
      image_.width = depth_height_;
      image_.height = depth_width_;
      image_.step = image_.width;
      image_.data.resize(depth_width_ * depth_height_);
      image_pub_ = nh_->advertise<sensor_msgs::Image>(TOPIC_LOCALIZATION_HR_IMAGE, 1);
    } else {
      image_pub_ = ros::Publisher();
    }

    // For debugging, publish total 14 point clouds, one for each processing step.
    // I was bored to set the name for each point cloud.
    std::string cloud_topic("cloud_pub");
    for (int i = 0; i < static_cast<int>(cloud_pub_.size()); ++i) {
      if (disp_pcd_and_tf_)
        cloud_pub_[i] = nh_->advertise<sensor_msgs::PointCloud>(
          ((std::string)TOPIC_LOCALIZATION_HR_CLOUD) + std::to_string(i), 1);
      else
        cloud_pub_[i] = ros::Publisher();
    }
  }

  // Enable or disable handrail detection
  bool EnableCallback(ff_msgs::SetBool::Request  &req, ff_msgs::SetBool::Response &res) {
    if (req.enable) {
      depth_sub_ = nh_->subscribe<sensor_msgs::PointCloud2>((std::string) TOPIC_HARDWARE_PICOFLEXX_PREFIX
        + (std::string) TOPIC_HARDWARE_NAME_PERCH_CAM + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX, 1,
           &HandrailDetect::PointCloud2Callback, this);
    } else {
      depth_sub_.shutdown();
    }
    res.success = true;
    return true;
  }

  // Called when a new point cloud arrives
  void PointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& depth_msg) {
    // Convert from pointcloud2 to pointcloud for easy access of the point cloud
    sensor_msgs::convertPointCloud2ToPointCloud(*depth_msg, cloud_);
    HandrailCallback();
  }

  // Performs handrail detection
  void HandrailCallback() {
    // Publish registration
    ff_msgs::CameraRegistration r;
    ros::Time timestamp = ros::Time::now();
    r.header = std_msgs::Header();
    r.header.stamp = timestamp;
    r.camera_id = frame_count_;
    registration_pub_.publish(r);
    ros::spinOnce();
    dl_.landmarks.clear();
    dl_.end_seen = 0;

    // Publish depth image
    if (disp_depth_image_)
      PublishDepthImage();
    if (disp_pcd_and_tf_)
      PublishTF(r2i_, body_frame_, perch_image_frame_);


    if (!FindHandrail()) {
      curr_handrail_status_ = NOT_FOUND;
      if (disp_marker_)
        PublishNoMarker();
    }

    if (curr_handrail_status_ == BOTH_ENDS || curr_handrail_status_ == FIRST_END
      || curr_handrail_status_ == SECOND_END)
      dl_.end_seen = 1;
    else if (curr_handrail_status_ == NOT_FOUND)
      dl_.end_seen = 2;

    dl_.header = std_msgs::Header();
    dl_.header.stamp = ros::Time::now();
    dl_.camera_id = frame_count_;
    landmark_pub_.publish(dl_);  // Publish landmarks
    ++frame_count_;              // Increment the camera id
  }

  // Code that locates the handrail in the points
  bool FindHandrail() {
    std::vector<int> downsample_points;
    if (!DownsamplePoints(&downsample_points)) {  // Get downsampled points
      ROS_ERROR("[Handrail] downsample points are less than 1 percent of the pcd: %d",
                static_cast<int>(downsample_points.size()));
      return false;
    }
    PublishCloud(downsample_points, cloud_, &cloud_pub_[0]);

    std::vector<int> plane_inliers, plane_outliers;
    Eigen::Vector3f plane_vector;
    Eigen::Vector4f plane_parameter;
    float plane_vector_length = 0;

    // Find a plane first, and then find a line from points that are apart from the plane with a certain dist
    if (!FindPlane(downsample_points, cloud_, &plane_inliers, &plane_outliers,
                   &plane_parameter, &plane_vector, &plane_vector_length)) {
      ROS_WARN("[Handrail] No plane found");
      PublishCloud(plane_inliers, cloud_, &cloud_pub_[1]);
      PublishCloud(plane_outliers, cloud_, &cloud_pub_[2]);
      return false;
    }

    GetXYScale(plane_inliers);
    PublishCloud(plane_inliers, cloud_, &cloud_pub_[1]);
    PublishCloud(plane_outliers, cloud_, &cloud_pub_[2]);

    Eigen::Vector3f line_vector = Eigen::Vector3f::Zero();
    Eigen::Vector3f line_center = Eigen::Vector3f::Zero();

    // Find a line from the plane outliers
    std::vector<int> line_inliers;

    if (!FindLine(plane_outliers, plane_parameter, plane_vector, plane_vector_length,
                  &line_vector, &line_center, &line_inliers)) {
      ROS_WARN("[Handrail] No handrail found");

      if (disp_marker_) {
        std::vector<int> no_pnt;
        PublishCloud(no_pnt, cloud_, &cloud_pub_[9]);
      }
      return false;
    }

    // Find end point(s) of the handrail
    std::vector<geometry_msgs::Point> end_point;
    UpdateEndPoint(line_inliers[0], line_inliers[line_inliers.size() - 1],
                   line_vector, line_center, &end_point);

    // Get target poise for perching
    Eigen::Vector3f target_pos_err;
    Eigen::Affine3f i2h;
    GetBodyTargetPose(line_vector, line_center, end_point, plane_vector, &target_pos_err, &i2h);

    // Get features from line inliers and plane inliers
    GetFeaturePoints(plane_inliers, line_inliers);

    Eigen::Quaternionf rtoq(i2h.linear());

    // Modulo 5 without risking to reach size limit
    if (rolling_window_cnt_ >= 5) {
      if (!start_)
        start_ = true;
      rolling_window_cnt_ = 0;
    }

    // Set up previous index since % operator is not modulo in C (uuuuuugh)
    int previous_index = rolling_window_cnt_ - 1;
    if (previous_index < 0)
      previous_index = 4;

    // If we have 5 previous positions saved
    if (start_) {
      // Mesure average of last 5 positions
      double average_pos[3];
      average_pos[0] = 0.0;
      average_pos[1] = 0.0;
      average_pos[2] = 0.0;
      for (int i = 0; i < 5; i++) {
        average_pos[0] += rolling_window_pos_(0, i);
        average_pos[1] += rolling_window_pos_(1, i);
        average_pos[2] += rolling_window_pos_(2, i);
      }
      average_pos[0] /= 5;
      average_pos[1] /= 5;
      average_pos[2] /= 5;

      // Judge if point is outlying (if violation of average +/- tolerance
      // on any axis, then copy previous point for all axes)
      if ((i2h.translation()(0) > (average_pos[0] + outlier_tolerance_)
            || i2h.translation()(0) < (average_pos[0] - outlier_tolerance_))
          || (i2h.translation()(1) > (average_pos[1] + outlier_tolerance_)
            || i2h.translation()(1) < (average_pos[1] - outlier_tolerance_))
          || (i2h.translation()(2) > (average_pos[2] + outlier_tolerance_)
            || i2h.translation()(2) < (average_pos[2] - outlier_tolerance_))) {
        // Count the amount of rejected features in a row.
        reject_cnt_++;

        // Reject current position and save previous
        // position: rolling_window_cnt_ - 1, modulo 5 to loop
        ROS_WARN("[Handrail] Refusing handrail outlier");
        rolling_window_pos_(0, rolling_window_cnt_) = rolling_window_pos_(0, previous_index);
        rolling_window_pos_(1, rolling_window_cnt_) = rolling_window_pos_(1, previous_index);
        rolling_window_pos_(2, rolling_window_cnt_) = rolling_window_pos_(2, previous_index);
        dl_.local_pose.position.x = rolling_window_pos_(0, previous_index);
        dl_.local_pose.position.y = rolling_window_pos_(1, previous_index);
        dl_.local_pose.position.z = rolling_window_pos_(2, previous_index);
        // Don't publish landmark if it refuses point
        dl_.landmarks.clear();

        // Since we copy the previous point in the hope of rejecting 1 or two wrong samples
        // We have to stop if we copy too many points in a row. This would imply we are lost
        if (reject_cnt_ >= 5) {
          ROS_ERROR("[Handrail] Rejected 5 or more handrail positions in a row");
          // Restart handrail detect if we are lost
          start_ = false;
          //  Don't publish landmark if we are lost
          dl_.landmarks.clear();
          return false;
        }
      } else {
        // Reset the counter of rejected features.
        reject_cnt_ = 0;

        // Accept current position
        rolling_window_pos_(0, rolling_window_cnt_) = i2h.translation()(0);
        rolling_window_pos_(1, rolling_window_cnt_) = i2h.translation()(1);
        rolling_window_pos_(2, rolling_window_cnt_) = i2h.translation()(2);
        dl_.local_pose.position.x = i2h.translation()(0);
        dl_.local_pose.position.y = i2h.translation()(1);
        dl_.local_pose.position.z = i2h.translation()(2);
      }
    } else {
      // If we don't have 5 positions saved, the outlier rejection averaging filter
      // can't work so we just copy the points.
      rolling_window_pos_(0, rolling_window_cnt_) = i2h.translation()(0);
      rolling_window_pos_(1, rolling_window_cnt_) = i2h.translation()(1);
      rolling_window_pos_(2, rolling_window_cnt_) = i2h.translation()(2);
      dl_.local_pose.position.x = i2h.translation()(0);
      dl_.local_pose.position.y = i2h.translation()(1);
      dl_.local_pose.position.z = i2h.translation()(2);
    }
    dl_.local_pose.orientation.x = rtoq.x();
    dl_.local_pose.orientation.y = rtoq.y();
    dl_.local_pose.orientation.z = rtoq.z();
    dl_.local_pose.orientation.w = rtoq.w();

    rolling_window_cnt_++;

    if (disp_marker_)
      PublishMarker(target_pos_err, i2h, end_point);

    return true;
  }


  bool DownsamplePoints(std::vector<int>* downsample_points) {
    // J.L Proposed Change
    // Keep only 1 point every 2x2 grid -> 75% reduction in sample size
    // This does not taking geometry into consideration
    // For example, past a certain angle with the plane, take 1 point every 1x2 grid ?
    // (lose resolution!)
    for (int i = 0; i < depth_height_; i++) {
      for (int j = 0; j < depth_width_; j++) {
        // Skip the first 5% of columns (width lines) to not see the arm (arm shadowing)
        if (j > depth_width_ / 20) {
          if (i % 2 == 0) {
            if (cloud_.points[i * depth_width_ + j].z < max_depth_dist_ &&
                cloud_.points[i * depth_width_ + j].z > min_depth_dist_ &&
                !(std::isnan(cloud_.points[i * depth_width_ + j].x) ||
                  std::isnan(cloud_.points[i * depth_width_ + j].y) ||
                  std::isnan(cloud_.points[i * depth_width_ + j].z))) {
              downsample_points->push_back(i * depth_width_ + j);
            }
          }
        }
      }
    }

    // If downsampled points are less than 1% of the point cloud, return false
    int downsample_size_thres = static_cast<int>(depth_width_ * depth_height_ * 0.01);
    if (static_cast<int>(downsample_points->size()) < downsample_size_thres) {
      return false;
    }
    return true;
  }

  bool FindPlane(const std::vector<int>& downsample_points,
                                 const sensor_msgs::PointCloud& filtered_cloud,
                                 std::vector<int>* plane_inliers, std::vector<int>* plane_outliers,
                                 Eigen::Vector4f* plane_parameter, Eigen::Vector3f* plane_vector,
                                 float* plane_vector_length) {
    std::vector<int> potential_plane_outliers;
    // Find plane using RANSAC
    // Set the minimum point numbers for the plane
    int plane_size_thres = downsample_points.size() * pcd_plane_rate_;
    int r1, r2, r3, num_data = downsample_points.size();
    int diff_thres = plane_size_thres / 20;
    // if plane points are larger than 70% of the downsampled points, stop iteration
    int max_thres = static_cast<int>(num_data * 0.7);
    Eigen::MatrixX4f A(3, 4);
    Eigen::Matrix4f ATA;

    for (int i = 0; i < RANSAC_plane_iteration_; ++i) {
      // Sample three points to model a normal vector of a plane
      do {
        r1 = rand_r(&seed_) % num_data;
        r2 = rand_r(&seed_) % num_data;
        r3 = rand_r(&seed_) % num_data;
      } while (r1 == r2 || r1 == r3 || r2 == r3);
      int ID1 = downsample_points[r1], ID2 = downsample_points[r2], ID3 = downsample_points[r3];
      // Make a form Ap = 0 where A is the matrix of the homogeneous representation of three randomly selected points,
      // and p is the plane parameter vector
      // Since v is the null space of A and rank(A) = 3, null(A) = 1, a unique plane can be defined.
      // Adding a constraint, |p| = 1, the null space p is the eigenvector of A'A that
      // corresponds the smallest eigenvalue
      A << filtered_cloud.points[ID1].x, filtered_cloud.points[ID1].y, filtered_cloud.points[ID1].z, 1,
      filtered_cloud.points[ID2].x, filtered_cloud.points[ID2].y, filtered_cloud.points[ID2].z, 1,
      filtered_cloud.points[ID3].x, filtered_cloud.points[ID3].y, filtered_cloud.points[ID3].z, 1;
      ATA = A.transpose() * A;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> eigensolver(ATA);
      Eigen::Vector4f tmp_plane_parameter = eigensolver.eigenvectors().col(0);  // Normalized plane parameter.
      Eigen::Vector3f tmp_normal_vector;  // Normal vector to the plane (not normalized)
      tmp_normal_vector << tmp_plane_parameter(0), tmp_plane_parameter(1), tmp_plane_parameter(2);

      tmp_normal_vector /= tmp_normal_vector.norm();
      std::vector<int> tmp_inliers, tmp_outliers;
      tmp_inliers.reserve(downsample_points.size());
      tmp_outliers.reserve(downsample_points.size());
      Eigen::Vector4f pnt_homo;
      for (auto const& itr : downsample_points) {
        // The dist between a point X=[x y z]' and a plane with parameter a, b, c, d is
        // res = |ax + by + cz + d| / sqrt(a^2 + b^2 + c^2)
        // If Y = [X' 1]', v = [a b c]', p = [v' d]' then
        // res = (Y' * p).norm / v.norm
        pnt_homo << filtered_cloud.points[itr].x, filtered_cloud.points[itr].y, filtered_cloud.points[itr].z, 1;
        if ((pnt_homo.transpose() * tmp_plane_parameter).norm() / tmp_normal_vector.norm() < RANSAC_plane_thres_)
          tmp_inliers.push_back(itr);   // Plane inliers
        else
          tmp_outliers.push_back(itr);  // Plane outliers which contain potential line points
      }
      // If tmp_inliers vector has more points than plane_inliers vector, update the plane_inliers vector
      if (tmp_inliers.size() > plane_inliers->size()) {
        // std::cout << "itr / pnt: " << i << "/" << tmp_inliers.size() << std::endl;
        int prev_size = plane_inliers->size();
        int curr_size = tmp_inliers.size();
        *plane_inliers = tmp_inliers;
        *plane_parameter = tmp_plane_parameter;
        potential_plane_outliers = tmp_outliers;

        // Stop iteration if the plane has enough points or it converges
        if (curr_size > max_thres) {
          // std::cout << "Enough points for plane! Finish at itr: " << i << "\n";
          break;
        } else if (i > RANSAC_plane_iteration_ / 2 && curr_size > plane_size_thres
                   && curr_size - prev_size < diff_thres) {
          // std::cout << "Plane converges! Finish at itr: " << i << "\n";
          break;
        }
      }
    }

    // If the number of plane points is less than a threshold, discard the result
    if (static_cast<int>(plane_inliers->size()) < plane_size_thres) {
      std::cout << "[1] Small plane points: " << static_cast<int>(plane_inliers->size())
                << " thres = " << plane_size_thres << std::endl;
      return false;
    }
    // std::cout << "plane points: " << static_cast<int>(plane_inliers->size()) << std::endl;
    // Refine plane parameter
    Eigen::MatrixX4f B(plane_inliers->size(), 4);
    Eigen::Matrix4f BTB;
    int row_cnt = 0;
    for (auto const& itr : (*plane_inliers)) {
      B.row(row_cnt) << cloud_.points[itr].x, cloud_.points[itr].y, cloud_.points[itr].z, 1;
      ++row_cnt;
    }
    BTB = B.transpose() * B;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> eigensolver(BTB);
    (*plane_parameter) = eigensolver.eigenvectors().col(0);  // Normalized plane parameter.
    // direct the normal vector of the plane to minus z axis
    if (eigensolver.eigenvectors()(2, 0) > 0)
      (*plane_parameter) = -1.0 * eigensolver.eigenvectors().col(0);

    (*plane_vector) << (*plane_parameter)(0), (*plane_parameter)(1), (*plane_parameter)(2);
    (*plane_vector_length) = plane_vector->norm();
    (*plane_vector) /= plane_vector->norm();

    // Get final plane outliers that are within the handrail range
    for (auto const& itr : potential_plane_outliers) {
      // Measure dist of the point from the plane
      Eigen::Vector4f test_point;
      test_point << cloud_.points[itr].x, cloud_.points[itr].y, cloud_.points[itr].z, 1;
      float dist_to_plane = test_point.dot((*plane_parameter)) / (*plane_vector_length);
      if (fabs(dist_to_plane - ave_handrail_wall_gap_) < handrail_width_)
        plane_outliers->push_back(itr);
    }
    return true;
  }

  bool ClusterLinePoints(const std::array<std::vector<int>, 2>& line_inliers,
                                         const Eigen::Vector4f& plane_parameter,
                                         const sensor_msgs::PointCloud& filtered_cloud, const int& line_size_thres,
                                         std::array<std::vector<int>, 2>* clustered_line_inliers) {
    std::array<std::vector<int>, 2> tmp_clustered_line_inliers;
    std::array<bool, 2> cluster_chk = {true, true};
    bool x_check = true;
    for (int i = 0; i < 2; ++i) {
      std::vector<int> line_itr = line_inliers[i];
      if (line_itr.size() == 0) {
        cluster_chk[i] = false;
        continue;
      }
      std::vector<int> tmp_line_inliers;
      if (i == 1)
        x_check = false;

      float gap_dist = handrail_group_gap_;
      ClusterPotentialLinePoints(line_itr, filtered_cloud, x_check, gap_dist, &tmp_line_inliers);

      // Locate the first and the second end points of the handrail
      // to the first and the last location of line_inliers vector
      if (static_cast<int>(tmp_line_inliers.size()) < line_size_thres) {
        cluster_chk[i] = false;
        continue;
      }

      int min_idx = tmp_line_inliers[0], max_idx = tmp_line_inliers[tmp_line_inliers.size() - 1];
      float sort_min = 100000, sort_max = -sort_min;
      for (auto const& itr : tmp_line_inliers) {
        float check_point_val = filtered_cloud.points[itr].y;
        if (i == 0) {  // vertical case
          check_point_val = filtered_cloud.points[itr].x;
        }
        if (check_point_val < sort_min) {
          sort_min = check_point_val;
          min_idx = itr;
        }
        if (check_point_val > sort_max) {
          sort_max = check_point_val;
          max_idx = itr;
        }
      }

      // Locate the first end point to the first position of the line vector
      tmp_clustered_line_inliers[i] = tmp_line_inliers;
      // Overlap the fst idx and the last idx
      tmp_clustered_line_inliers[i][0] = min_idx;
      tmp_clustered_line_inliers[i][tmp_line_inliers.size() - 1] = max_idx;

      if (static_cast<int>(tmp_clustered_line_inliers[i].size()) < line_size_thres) {
        tmp_clustered_line_inliers[i].clear();
        cluster_chk[i] = false;
      }
    }

    PublishCloud(tmp_clustered_line_inliers[0], filtered_cloud, &cloud_pub_[7]);
    PublishCloud(tmp_clustered_line_inliers[1], filtered_cloud, &cloud_pub_[8]);

    if (cluster_chk[0] == false && cluster_chk[1] == false) {
      return false;
    } else {
      (*clustered_line_inliers) = tmp_clustered_line_inliers;
      return true;
    }
  }

  void ClusterPotentialLinePoints(const std::vector<int>& potential_line_inliers,
      const sensor_msgs::PointCloud& filtered_cloud, bool x_check, float gap_dist,
      std::vector<int>* clustered_line_inliers) {
    std::vector<int> tmp_clustered_line_inliers;
    // 1. Sort the line inliers by their x or y values
    std::vector<float> axis_vals;
    axis_vals.reserve(potential_line_inliers.size());
    for (auto const& itr : potential_line_inliers) {
      if (x_check)
        axis_vals.push_back(filtered_cloud.points[itr].x);
      else
        axis_vals.push_back(filtered_cloud.points[itr].y);
    }
    std::sort(axis_vals.begin(), axis_vals.end());

    // 2. Find groups that are apart from each other with handrail_group_gap_ dist
    //    Store min and max values as well as number of members of each group
    std::vector<float> min_val, max_val;
    std::vector<int> group_cnt;
    int grofst_end_ID = 0;
    int max_element = 1, max_grofst_end_ID = 0;
    min_val.push_back(axis_vals[0]);
    group_cnt.push_back(1);
    for (size_t i = 1; i < axis_vals.size(); ++i) {
      if (fabs(axis_vals[i - 1] - axis_vals[i]) < gap_dist) {
        // std::cout << "Dist: " << fabs(axis_vals[i - 1] - axis_vals[i]) << std::endl;
        ++group_cnt[grofst_end_ID];
      } else {
        if (group_cnt[grofst_end_ID] > max_element) {
          max_element = group_cnt[grofst_end_ID];
          max_grofst_end_ID = grofst_end_ID;
        }
        ++grofst_end_ID;
        max_val.push_back(axis_vals[i - 1]);
        min_val.push_back(axis_vals[i]);
        group_cnt.push_back(1);
      }
    }
    max_val.push_back(axis_vals[axis_vals.size() - 1]);
    if (group_cnt[group_cnt.size() - 1] > max_element) {  // Check the last group whether it has
      max_grofst_end_ID = group_cnt.size() - 1;           // the maximum number of members
    }

    if (group_cnt.size() == 1) {
      tmp_clustered_line_inliers = potential_line_inliers;
    } else {
      tmp_clustered_line_inliers.reserve(potential_line_inliers.size());
      for (auto const& itr : potential_line_inliers) {
        float check_point_val;
        if (x_check)
          check_point_val = filtered_cloud.points[itr].x;
        else
          check_point_val = filtered_cloud.points[itr].y;
        if (check_point_val >= min_val[max_grofst_end_ID] && check_point_val <= max_val[max_grofst_end_ID]) {
          tmp_clustered_line_inliers.push_back(itr);  // not sorted!!!
        }
      }
    }
    (*clustered_line_inliers) = tmp_clustered_line_inliers;
  }


  bool FilterCloud(const std::vector<int>& sample_points,
                                   sensor_msgs::PointCloud* filtered_cloud) {
    int filter_size = 15;
    // Filter-related matrices and vector
    Eigen::MatrixXf I_mat_sm = Eigen::MatrixXf::Identity(filter_size - 1, filter_size - 1);
    Eigen::MatrixXf D_mat = -1 * Eigen::MatrixXf::Identity(filter_size - 1, filter_size);
    D_mat.block(0, 1, filter_size - 1, filter_size - 1) += I_mat_sm;
    Eigen::MatrixXf I_mat = Eigen::MatrixXf::Identity(filter_size, filter_size);
    Eigen::VectorXf L_vec_sq = Eigen::VectorXf::Zero(filter_size - 1);


    int half_filter_size = filter_size / 2;  // Half of each window is overlapped
    int sample_points_size = static_cast<int>(sample_points.size());
    float filter_min_lambda_sq = 0.01;
    float filter_max_lambda_sq = 100.0;

    if (sample_points_size < half_filter_size) {
      ROS_ERROR("Too small sample points");
      return false;
    }
    for (int i = 0; i < sample_points_size; i += half_filter_size) {
      Eigen::VectorXf b(filter_size);
      int strt_idx = 0, end_idx = sample_points_size - 1;
      if (i + filter_size < sample_points_size) {
        strt_idx = i;
        end_idx = i + filter_size;
      } else {
        strt_idx = sample_points_size - half_filter_size;
        end_idx = sample_points_size;
      }
      end_idx = std::min(end_idx, sample_points_size - 1);

      // Set lambda matrix depending on the differential of data
      b(0) = cloud_.points[sample_points[strt_idx]].z;
      float filter_max_dist = handrail_wall_min_gap_ * 0.5;
      for (int cnt = 1, j = strt_idx + 1; j < end_idx; ++j, ++cnt) {
        b(cnt) = cloud_.points[sample_points[j]].z;
        if (fabs(b(cnt) - b(cnt-1)) < filter_max_dist)
          L_vec_sq(cnt - 1) = filter_max_lambda_sq;
        else
          L_vec_sq(cnt - 1) = filter_min_lambda_sq;
      }

      Eigen::MatrixXf L_mat_sq = L_vec_sq.asDiagonal();
      Eigen::MatrixXf F_mat = D_mat.transpose() * L_mat_sq * D_mat;
      Eigen::MatrixXf A_mat = I_mat + F_mat;

      // solve Ax = b
      Eigen::VectorXf x = A_mat.llt().solve(b);

      for (int cnt = 0, j = strt_idx; j < end_idx; ++j, ++cnt) {
        filtered_cloud->points[sample_points[j]].z = x(cnt);
      }
    }
    return true;
  }


  void PublishTF(const Eigen::Affine3f& publish_tf,
                                 std::string parent_frame, std::string child_frame) {
    static tf2_ros::TransformBroadcaster tf_publisher;
    Eigen::Quaternionf rtoq(publish_tf.linear());
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.transform.translation.x = static_cast<float>(publish_tf.translation()(0));
    transformStamped.transform.translation.y = static_cast<float>(publish_tf.translation()(1));
    transformStamped.transform.translation.z = static_cast<float>(publish_tf.translation()(2));
    transformStamped.transform.rotation.x = rtoq.x();
    transformStamped.transform.rotation.y = rtoq.y();
    transformStamped.transform.rotation.z = rtoq.z();
    transformStamped.transform.rotation.w = rtoq.w();
    tf_publisher.sendTransform(transformStamped);
  }

  bool FindPotentialLinePoints(const std::vector<int>& new_plane_outliers,
      const Eigen::Vector4f& plane_parameter, const Eigen::Vector3f& plane_vector,
      const int& line_size_thres, const float& plane_vector_length,
      std::vector<int>* potential_line_inliers) {
    Eigen::Vector3f test_point, cloud_point;
    Eigen::Vector4f tmp_homo_point;
    std::vector<int> side_points, fill_points;
    float plane_side_gap = handrail_wall_min_gap_;
    float handrail_wall_max_gap = handrail_wall_min_gap_ + 3 * handrail_width_;
    float plane_ang = atan2(-plane_vector(1) , -plane_vector(2));
    float long_side_gap = plane_side_gap + handrail_wall_max_gap * (1.0 - cos(plane_ang));

    int side_step = static_cast<int>(floor(handrail_width_ * sqrt(2) / x_scale_));
    int y_plus_step = side_step;
    int y_minus_step = side_step;
    int side_step2 = static_cast<int>(side_step * cos(plane_ang) +
                                      fabs(handrail_wall_max_gap * sin(plane_ang) / x_scale_));

    // extention of x axis sidegap is not considered
    std::array<float, 4> plane_side_thres = {plane_side_gap, plane_side_gap, plane_side_gap, plane_side_gap};
    if (plane_ang > 0) {
      y_plus_step = side_step2;
      plane_side_thres[0] = long_side_gap;
    } else if (plane_ang < 0) {
      y_minus_step = side_step2;
      plane_side_thres[1] = long_side_gap;
    }

    std::vector<int> cross_inliers, width_inliers;
    float handrail_min_gap = width_filter_rate_ * handrail_width_;
    float handrail_max_gap = handrail_width_ * sqrt(2);  // J.L -> Why is it not handrail_gap * sqrt(2) ?
                                                         // This is handrail_max_width.
    std::vector<int> plane_outliers = new_plane_outliers;

    while (plane_outliers.size() != 0) {
      int itr = plane_outliers.back();
      plane_outliers.pop_back();
      // Measure dist of the point from the plane
      // Ted & Katie: Removed the 1. Crashed Eigen into a bus full of school
      // kids.
      test_point << cloud_.points[itr].x, cloud_.points[itr].y, cloud_.points[itr].z;  // , 1;
      std::array<int, 2> side_cnt = {0, 0}, behind_cnt = {0, 0};
      std::array<int, 4> side_index = {itr - y_minus_step * depth_width_, itr + y_plus_step * depth_width_,
                                       itr - side_step, itr + side_step
                                      };

      // [Cross point test]: set side points for the test point
      // and check whether they are on the wall.
      // If both of them are on the wall, the testing point passes the test
      for (int yx_idx = 0; yx_idx < max_num_handrails_; ++yx_idx) {  // 0:y, 1:x
        int tmp_idx = 0;
        int edge_step = 1;

        if (yx_idx == 0) {   // Check out of bound side points in y axis
          edge_step = depth_width_;

          if (curr_handrail_status_ != NOT_FOUND && dist_to_handrail_ < close_dist_to_handrail_) {
            // In the case of vertical handrail with close distance for grasping,
            // the +y side checking points should be extened to +y direction
            // due to the location of the perchcam on the robot
            side_index[1] += (y_plus_step * depth_width_);
          }
          // +y side
          if (side_index[1] >= static_cast<int>(cloud_.points.size())) {
            side_index[1] = depth_width_ * (depth_height_ - 1) + side_index[1] % depth_width_;
          }
          // -y side
          if (curr_handrail_status_ != NOT_FOUND && dist_to_handrail_ < close_dist_to_handrail_) {
            // In the case of vertical handrail with close distance for grasping,
            // the -y side checking points are occluded by the handrail
            // due to the location of the perchcam on the robot.
            // So, the cross point test ignores -y side point in this case
            // by setting side_index[0] to side_index[1]
            side_index[0] = side_index[1];
          } else if (side_index[0] < 0) {
            side_index[0] = itr;
          }
        } else {  // Check out of bound side points in x axis
          // +x side
          if (side_index[3] % depth_width_ < itr % depth_width_) {
            side_index[3] = ((side_index[3] / depth_width_) + 1) * depth_width_ - 1;
          }
          // -x side
          if (side_index[2] < 0) {
            side_index[2] = 0;
          } else if (side_index[2] % depth_width_ > itr % depth_width_) {
            side_index[2] = (side_index[2] / depth_width_) * depth_width_;
          }
        }

        // Check garbage side points
        if (cloud_.points[side_index[2 * yx_idx]].z == 0) {
          tmp_idx = itr;
          for (int ti = side_index[2 * yx_idx]; ti <= itr; ti += edge_step) {
            if (cloud_.points[ti].z != 0) {
              tmp_idx = ti;
              break;
            }
          }
          side_index[2 * yx_idx] = tmp_idx;
        }
        if (cloud_.points[side_index[2 * yx_idx + 1]].z == 0) {
          tmp_idx = itr;
          for (int ti = side_index[2 * yx_idx + 1]; ti >= itr; ti -= edge_step) {
            if (cloud_.points[ti].z != 0) {
              tmp_idx = ti;
              break;
            }
          }
          side_index[2 * yx_idx + 1] = tmp_idx;
        }

        for (int j = 0; j < 2; ++j) {
          int side_itr = side_index[2 * yx_idx + j];
          tmp_homo_point << cloud_.points[side_itr].x, cloud_.points[side_itr].y, cloud_.points[side_itr].z, 1;
          float dist_side_plane = tmp_homo_point.dot(plane_parameter) / plane_vector_length;
          // using signed distance makes important role since one side of the handrail
          // has a deep gap back to the plane.
          // So the side points behind the plane (dist_side_plane < -plane_side_thres) should be counted.
          if (fabs(dist_side_plane) < plane_side_thres[2 * yx_idx + j])
            ++side_cnt[yx_idx];
          else if (dist_side_plane < 0)
            ++behind_cnt[yx_idx];
        }
        // Only at most one side of the points is allowd to be located behind of the plane
        if (side_cnt[yx_idx] == 1 && behind_cnt[yx_idx] == 1)
          ++side_cnt[yx_idx];
      }

      // If both do not have side points, return false
      if (side_cnt[0] < 2 && side_cnt[1] < 2) {
        continue;
      }

      // [Side width test]: For the point that passed the cross point test,
      // collect the points on the line between the cross points that are within
      // the range of the half of the handrail width.
      // [filling out empty space]: While checking side width test, find zero points
      // along the line between the left and the right cross points,
      // and replace them with their test point with left and right shift
      std::vector<int> neighbor_inliers;
      std::array<bool, 2> test_check = {true, true};

      for (int yx_idx = 0; yx_idx < max_num_handrails_; ++yx_idx) {
        int edge_step = y_step_ * depth_width_;
        if (yx_idx == 1)
          edge_step = x_step_;

        if (side_cnt[yx_idx] == 2) {
          side_points.push_back(side_index[2 * yx_idx]);       // right (-y) side points for vertical handrail
          side_points.push_back(side_index[2 * yx_idx + 1]);   // left (+y) side points for vertical handrail

          cross_inliers.push_back(itr);

          std::vector<int> tmp_edge_points;
          for (int i = side_index[2 * yx_idx] + edge_step; i < side_index[2 * yx_idx + 1]; i += edge_step) {
            cloud_point << cloud_.points[i].x, cloud_.points[i].y, cloud_.points[i].z;
            float gap_dist = (test_point - cloud_point).norm();
            if (gap_dist <= handrail_width_ * 0.5) {
              tmp_edge_points.push_back(i);
            } else if (cloud_.points[i].z == 0) {
              // Start filling out empty space by projecting 2D pixel to 3D position
              // Get fx (=focal_lehgth / x_scale) and fy from the test point (x_scale * u : focal_length = x : z)
              int itr_u = itr % depth_width_ - depth_width_ / 2;
              int itr_v = itr / depth_width_ - depth_height_ / 2;
              float fx = static_cast<float>(itr_u) * (test_point(2) / test_point(0));
              float fy = static_cast<float>(itr_v) * (test_point(2) / test_point(1));

              // Get u, v from image plane
              int u = i % depth_width_ - depth_width_ / 2;
              int v = i / depth_width_ - depth_height_ / 2;

              // Project 2D to 3D
              float new_x = static_cast<float>(u) * test_point(2) / fx;
              float new_y = static_cast<float>(v) * test_point(2) / fy;

              // Set the 3D position of the zero point with new position
              cloud_.points[i].x = new_x;
              cloud_.points[i].y = new_y;
              cloud_.points[i].z = test_point(2);

              tmp_edge_points.push_back(i);
              fill_points.push_back(i);
            }
          }
          if (tmp_edge_points.size() >= 2) {
            std::vector<float> val_vect;
            if (yx_idx == 0) {
              for (auto const& edge_itr : tmp_edge_points)
                val_vect.push_back(cloud_.points[edge_itr].y);
            } else {
              for (auto const& edge_itr : tmp_edge_points)
                val_vect.push_back(cloud_.points[edge_itr].x);
            }
            std::sort(val_vect.begin(), val_vect.end());
            float gap_dist = fabs(val_vect[0] - val_vect[val_vect.size() - 1]);
            if (gap_dist < handrail_min_gap || gap_dist > handrail_max_gap)
              test_check[yx_idx] = false;
            else
              neighbor_inliers.insert(neighbor_inliers.end(), tmp_edge_points.begin(), tmp_edge_points.end());
          } else if (side_index[2 * yx_idx] != side_index[2 * yx_idx + 1]) {
            test_check[yx_idx] = false;
          }
        }
      }
      if (test_check[0] && test_check[1]) {
        width_inliers.push_back(itr);
        width_inliers.insert(width_inliers.end(), neighbor_inliers.begin(), neighbor_inliers.end());
      }
    }

    PublishCloud(cross_inliers, cloud_, &cloud_pub_[3]);
    PublishCloud(width_inliers, cloud_, &cloud_pub_[4]);
    PublishCloud(side_points, cloud_, &cloud_pub_[10]);
    PublishCloud(fill_points, cloud_, &cloud_pub_[11]);

    if (static_cast<int>(width_inliers.size()) < line_size_thres)
      return false;

    (*potential_line_inliers) = width_inliers;
    return true;
  }

  bool FindBestLinePoints(const std::vector<int>& potential_line_inliers,
                                          const int& line_size_thres, const Eigen::Vector3f& plane_vector,
                                          sensor_msgs::PointCloud* new_filtered_cloud,
                                          std::array<std::vector<int>, 2>* best_line_inliers) {
    sensor_msgs::PointCloud filtered_cloud = (*new_filtered_cloud);
    int num_potential_data = potential_line_inliers.size();
    int min_line_size_thres = line_size_thres / 2;
    Eigen::Vector3f fst_line_point, scd_line_point, tmp_line_vector, point_diff, potential_point;

    // Line Estimation in xy plane since z values of the points are highly corrupted
    // due to unexpected interference and reflection
    std::array<std::vector<int>, 2> both_line_inliers;
    std::array<Eigen::Vector3f, 2> both_line_vectors;
    std::array<float, 2> both_line_angs = {M_PI_2, 0};
    int r1, r2, ID1, ID2;
    float m_pi_8 = M_PI / 8.0;
    for (int i = 0; i < RANSAC_line_iteration_; ++i) {
      do {
        r1 = rand_r(&seed_) % num_potential_data;
        r2 = rand_r(&seed_) % num_potential_data;
      } while (r1 == r2);
      ID1 = potential_line_inliers[r1];
      ID2 = potential_line_inliers[r2];

      // Set z value to zero for xy plane line estimation
      fst_line_point << filtered_cloud.points[ID1].x, filtered_cloud.points[ID1].y, 0;
      scd_line_point << filtered_cloud.points[ID2].x, filtered_cloud.points[ID2].y, 0;
      tmp_line_vector = fst_line_point - scd_line_point;
      tmp_line_vector /= tmp_line_vector.norm();

      // Calculate residual of potential points with respect to the sampled line vector
      // The dist between a line that passes x1 and x2 and a point x0 is defined as
      // d = |(x1 - x2) cross (x2 - x0)| / |(x1 - x2)|
      std::vector<int> tmp_line_inliers;
      tmp_line_inliers.reserve(num_potential_data);
      for (auto const& itr : potential_line_inliers) {
        // Set z value to zero for xy plane line estimation
        potential_point << filtered_cloud.points[itr].x, filtered_cloud.points[itr].y, 0;
        point_diff = scd_line_point - potential_point;
        if ((tmp_line_vector.cross(point_diff)).norm() < RANSAC_line_thres_)
          tmp_line_inliers.push_back(itr);
      }

      if (static_cast<int>(tmp_line_inliers.size()) < min_line_size_thres)
        continue;

      float tmp_line_ang = atan2(fabs(tmp_line_vector(0)), fabs(tmp_line_vector(1)));

      int idx = 0;
      if (fabs(tmp_line_ang - both_line_angs[0]) > fabs(tmp_line_ang - both_line_angs[1])) {
        if (max_num_handrails_ == 1)
          continue;
        idx = 1;
      }
      if (tmp_line_inliers.size() > both_line_inliers[idx].size()) {
        both_line_inliers[idx] = tmp_line_inliers;
        both_line_vectors[idx] = tmp_line_vector;
        both_line_angs[idx] = tmp_line_ang;
        if (max_num_handrails_ == 2 && both_line_inliers[idx].size() > both_line_inliers[1 - idx].size()) {
          float tmp_scd_ang = M_PI_2 - tmp_line_ang;
          if (fabs(both_line_angs[1 - idx] - tmp_scd_ang) > m_pi_8)
            both_line_inliers[1- idx].clear();
          both_line_angs[1 - idx] = tmp_scd_ang;
        }
      }
    }

    std::array<std::vector<int>, 2> fin_line_inliers;
    for (int i = 0; i < max_num_handrails_; ++i) {
      if (static_cast<int>(both_line_inliers[i].size()) < line_size_thres) {
        both_line_inliers[i].clear();
        continue;
      }

      // Project the line vector to the plane using the normal vector of the plane
      both_line_vectors[i] = both_line_vectors[i] - (both_line_vectors[i].dot(plane_vector) * plane_vector);
      both_line_vectors[i] /= both_line_vectors[i].norm();
      fin_line_inliers[i] = both_line_inliers[i];
    }
    PublishCloud(fin_line_inliers[0], filtered_cloud, &cloud_pub_[12]);
    (*new_filtered_cloud) = filtered_cloud;

    if (fin_line_inliers[0].size() == 0 && fin_line_inliers[1].size() == 0)
      return false;

    if (fin_line_inliers[0].size() == 0) {
      fin_line_inliers[0] = fin_line_inliers[1];
      fin_line_inliers[1].clear();
    } else if (both_line_angs[0] < M_PI_4) {
      std::vector<int> swap_inliers = fin_line_inliers[0];
      fin_line_inliers[0] = fin_line_inliers[1];
      fin_line_inliers[1] = swap_inliers;
    }

    (*best_line_inliers) = fin_line_inliers;
    return true;
  }

  float GetLineDist(const std::vector<int>& point_inliers,
                                    const sensor_msgs::PointCloud& filtered_cloud, Eigen::Vector3f* center_pos) {
    if (static_cast<int>(point_inliers.size()) < 2)
      return 0;

    int fst_ID = point_inliers[0];
    int scd_ID = point_inliers[point_inliers.size() - 1];
    Eigen::Vector3f fst_line_point, scd_line_point;
    fst_line_point << filtered_cloud.points[fst_ID].x, filtered_cloud.points[fst_ID].y, filtered_cloud.points[fst_ID].z;
    scd_line_point << filtered_cloud.points[scd_ID].x, filtered_cloud.points[scd_ID].y, filtered_cloud.points[scd_ID].z;
    (*center_pos) = (fst_line_point + scd_line_point) / 2.0;
    return (fst_line_point - scd_line_point).norm();
  }

  void UpdateEndPoint(const int& fst_end, const int& scd_end, const Eigen::Vector3f& line_vector,
                      const Eigen::Vector3f& line_center, std::vector<geometry_msgs::Point>* end_point) {
    geometry_msgs::Point geo_point;
    if (curr_handrail_status_ == BOTH_ENDS) {
      geo_point = FindCloseLinePoint(fst_end, line_vector, line_center);
      end_point->push_back(geo_point);
      geo_point = FindCloseLinePoint(scd_end, line_vector, line_center);
      end_point->push_back(geo_point);
    } else if (curr_handrail_status_ != NO_END) {
      if (curr_handrail_status_ == FIRST_END) {
        geo_point = FindCloseLinePoint(fst_end, line_vector, line_center);
        end_point->push_back(geo_point);
      } else if (curr_handrail_status_ == SECOND_END) {
        geo_point = FindCloseLinePoint(scd_end, line_vector, line_center);
        end_point->push_back(geo_point);
      }
    }
  }

  geometry_msgs::Point FindCloseLinePoint(const int& point_ID,
      const Eigen::Vector3f& line_vector, const Eigen::Vector3f& line_center) {
    geometry_msgs::Point geo_point;
    Eigen::Vector3f tmp_vector, tmp_point;
    tmp_point << cloud_.points[point_ID].x, cloud_.points[point_ID].y, cloud_.points[point_ID].z;
    tmp_vector = tmp_point - line_center;
    tmp_point = line_center + tmp_vector.dot(line_vector) * line_vector;
    geo_point.x = tmp_point(0);
    geo_point.y = tmp_point(1);
    geo_point.z = tmp_point(2);
    return geo_point;
  }

  bool FindLine(const std::vector<int>& plane_outliers, const Eigen::Vector4f& plane_parameter,
                                const Eigen::Vector3f& plane_vector, const float& plane_vector_length,
                                Eigen::Vector3f* line_vector, Eigen::Vector3f* line_center,
                                std::vector<int>* line_inliers) {
    float handrail_wall_max_gap = handrail_wall_min_gap_ + 3 * handrail_width_;
    float dist_plane_to_rob = std::max(static_cast<float>(fabs(plane_parameter(3) / plane_parameter(2))),
                                       handrail_wall_max_gap);
    float fov_x = 62.0, fov_y = 45.0;
    float fov_x_half_tan = static_cast<float>(tan(fov_x * 0.5 * (M_PI / 180.0)));
    float fov_y_half_tan = static_cast<float>(tan(fov_y * 0.5 * (M_PI / 180.0)));
    float half_tan = std::min(fov_x_half_tan, fov_y_half_tan);
    float measure_min_length = 2 * (dist_plane_to_rob - handrail_wall_max_gap) * half_tan;
    float handrail_min_length = std::min(static_cast<float>(0.6 * fix_handrail_length_), measure_min_length);

    int line_size_thres = ((handrail_min_length * handrail_width_ * 0.5) /
                           (3.0 * x_scale_ * x_step_ * y_scale_ * y_step_));
    if (line_size_thres <= 0) {
      std::cout << "[Line Fail 0] line thres is not positive: " << line_size_thres << std::endl;
      return false;
    }

    std::chrono::high_resolution_clock::time_point t_strt = std::chrono::high_resolution_clock::now();
    // 1. Find potential line inliers from plane outliers
    std::vector<int> potential_line_inliers;
    if (!FindPotentialLinePoints(plane_outliers, plane_parameter, plane_vector, line_size_thres, plane_vector_length,
                                 &potential_line_inliers)) {
      std::cout << "[Line Fail 1] no potential line inliers\n";
      std::cout << "[fail] line size/thres: " << potential_line_inliers.size() << "/" << line_size_thres << std::endl;
      return false;
    }
    if (disp_computation_time_) {
      std::chrono::high_resolution_clock::time_point t_potential_end = std::chrono::high_resolution_clock::now();
      auto potential_time = std::chrono::duration_cast<std::chrono::microseconds>(t_potential_end - t_strt).count();
      float potential_time_ms = static_cast<float>(potential_time) * 0.001;
      std::cout << "[Line] potential time: "<< potential_time_ms << std::endl;
    }

    if (!FilterCloud(potential_line_inliers, &cloud_))
      return false;

    t_strt = std::chrono::high_resolution_clock::now();
    // 2. Find a line model from potential_line_inliers using RANSAC
    std::array<std::vector<int>, 2> best_line_inliers;
    if (!FindBestLinePoints(potential_line_inliers, line_size_thres, plane_vector,
                            &cloud_, &best_line_inliers)) {
      ROS_WARN("[Handrail] [Line Fail 2] no best line inliers");
      return false;
    }
    if (disp_computation_time_) {
      std::chrono::high_resolution_clock::time_point t_best_end = std::chrono::high_resolution_clock::now();
      auto best_time = std::chrono::duration_cast<std::chrono::microseconds>(t_best_end - t_strt).count();
      float best_time_ms = static_cast<float>(best_time) * 0.001;
      std::cout << "[Line] ransac time: "<< best_time_ms << std::endl;
    }

    PublishCloud(best_line_inliers[0], cloud_, &cloud_pub_[5]);
    PublishCloud(best_line_inliers[1], cloud_, &cloud_pub_[6]);

    t_strt = std::chrono::high_resolution_clock::now();
    std::array<std::vector<int>, 2> group_line_inliers;
    if (!ClusterLinePoints(best_line_inliers, plane_parameter, cloud_, line_size_thres, &group_line_inliers)) {
      ROS_WARN("[Handrail] [Line Fail 3] no filtered line inliers");
      std::cout << "Inp: " << best_line_inliers[0].size() << "/"  << best_line_inliers[1].size() << std::endl;
      std::cout << "Out: " << group_line_inliers[0].size() << "/"  << group_line_inliers[1].size() << std::endl;
      return false;
    }

    std::vector<int> clustered_line_inliers;
    Eigen::Vector3f tmp_end_points_line_center = Eigen::Vector3f::Zero();
    handrail_length_ = 0;

    if (group_line_inliers[0].size() == 0) {
      handrail_length_ = GetLineDist(group_line_inliers[1], cloud_, &tmp_end_points_line_center);
      if (handrail_length_ > handrail_min_length) {
        clustered_line_inliers = group_line_inliers[1];
        vertical_handrail_detect_ = false;
      }
    } else if (group_line_inliers[1].size() == 0) {
      handrail_length_ = GetLineDist(group_line_inliers[0], cloud_, &tmp_end_points_line_center);
      if (handrail_length_ > handrail_min_length) {
        clustered_line_inliers = group_line_inliers[0];
        vertical_handrail_detect_ = true;
      }
    } else {
      std::array<float, 2> tmp_length = {0, 0};
      std::array<Eigen::Vector3f, 2> tmp_center;
      tmp_length[0] = GetLineDist(group_line_inliers[0], cloud_, &tmp_center[0]);
      tmp_length[1] = GetLineDist(group_line_inliers[1], cloud_, &tmp_center[1]);

      if (tmp_length[0] > handrail_min_length && tmp_length[1] > handrail_min_length) {
        clustered_line_inliers = group_line_inliers[0];
        handrail_length_ = tmp_length[0];
        tmp_end_points_line_center = tmp_center[0];
        vertical_handrail_detect_ = true;
      } else {
        if (tmp_length[0] > handrail_min_length) {
          clustered_line_inliers = group_line_inliers[0];
          handrail_length_ = tmp_length[0];
          tmp_end_points_line_center = tmp_center[0];
          vertical_handrail_detect_ = true;
        } else if (tmp_length[1] > handrail_min_length) {
          clustered_line_inliers = group_line_inliers[1];
          handrail_length_ = tmp_length[1];
          tmp_end_points_line_center = tmp_center[1];
          vertical_handrail_detect_ = false;
        }
      }
    }

    PublishCloud(clustered_line_inliers, cloud_, &cloud_pub_[9]);

    if (clustered_line_inliers.size() == 0) {
      ROS_WARN("[Handrail] [Line Fail 4] Handrail length is too short: %f/%f", handrail_length_,
               handrail_min_length);
      return false;
    } else if (handrail_length_ > 1.3 * fix_handrail_length_) {
      ROS_WARN("[Handrail] [Line Fail 5] Handrail length is too long: %f/%f", handrail_length_,
               1.3 * fix_handrail_length_);
      return false;
    }

    // Find median of inliers
    Eigen::Vector3f end_points_line_center = Eigen::Vector3f::Zero();
    std::array<std::vector<float>, 3> median_vec;
    for (auto const& itr : clustered_line_inliers) {
      median_vec[0].push_back(cloud_.points[itr].x);
      median_vec[1].push_back(cloud_.points[itr].y);
      median_vec[2].push_back(cloud_.points[itr].z);
    }
    std::sort(median_vec[0].begin(), median_vec[0].end());
    std::sort(median_vec[1].begin(), median_vec[1].end());
    std::sort(median_vec[2].begin(), median_vec[2].end());
    int median_idx = static_cast<int>(clustered_line_inliers.size() / 2);
    int quarter_idx = median_idx / 2;  // The reason of using the 1/4 the index for z val is that
    // the measured line is bent like a bow.
    // To avoid setting the z val behind the real handrail,
    // the z value with a quarter index is used.

    // Use the median point of the inlier as the representative point in the line
    end_points_line_center << median_vec[0][median_idx], median_vec[1][median_idx], median_vec[2][quarter_idx];




    // Continuously update gap between the handrail and the wall
    Eigen::Vector4f homo_line_center;
    homo_line_center << end_points_line_center(0),  end_points_line_center(1),  end_points_line_center(2), 1;
    float measure_handrail_wall_gap = fabs(homo_line_center.dot(plane_parameter)) / plane_vector_length;
    float gap_update_rate = 0.1;
    if (measure_handrail_wall_gap > handrail_wall_min_gap_ && measure_handrail_wall_gap < handrail_wall_max_gap)
      ave_handrail_wall_gap_ += (gap_update_rate * (measure_handrail_wall_gap - ave_handrail_wall_gap_));
    // std::cout << "ave: " << ave_handrail_wall_gap_ << std::endl;

    // Find line vector (v with |v| = 1)
    // d = |(x1 - x2) cross (x2 - x0)| / |(x1 - x2)|
    // If we set x2 as a line center, c, av = (x1 - c) where a is a scalar and v is a line vector, then
    // for any x0 = x, d = |av cross (c - x)| / |av| = |v cross (c - x)| / |v|
    // = |v cross (c - x)| = |(c - x) cross v|
    // To find v that minimizes d, make the equation to a form Av = 0 then v is the nullspace of A
    // A matrix is a stack of skew(c-x) matrix.
    // Since A is (3 * num_line_inliers) by 3 matrix, calculating A'Av = 0 is faster than Av = 0
    Eigen::MatrixX3f A(3 * clustered_line_inliers.size(), 3);
    float min_norm = 100;
    int row_cnt = 0;
    Eigen::Vector3f line_point, point_diff;
    for (auto const& itr : clustered_line_inliers) {
      line_point << cloud_.points[itr].x, cloud_.points[itr].y, cloud_.points[itr].z;
      point_diff = end_points_line_center - line_point;

      // Convert "v_s cross" to a matrix form
      A.row(row_cnt * 3)     << 0, -point_diff(2), point_diff(1);
      A.row(row_cnt * 3 + 1) << point_diff(2), 0, -point_diff(0);
      A.row(row_cnt * 3 + 2) << -point_diff(1), point_diff(0), 0;
      ++row_cnt;
      // Find the closest measured handrail center point
      if (point_diff.norm() < min_norm)
        min_norm = point_diff.norm();
    }
    Eigen::Matrix3f ATA = A.transpose() * A;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(ATA);
    Eigen::Vector3f tmp_line_vector = eigensolver.eigenvectors().col(0);
    if (vertical_handrail_detect_) {
      if (tmp_line_vector(0) < 0)
        tmp_line_vector *= -1;
    } else {
      if (tmp_line_vector(1) < 0)
        tmp_line_vector *= -1;
    }

    // Since the line vector should be orthogonal to the normal vector of the plane,
    // project the calcuated line vector, v, to the plane with the normal vector n
    // u = v - (v dot n)n
    *line_vector = tmp_line_vector - (tmp_line_vector.dot(plane_vector) * plane_vector);
    (*line_vector) /= (*line_vector).norm();
    *line_inliers = clustered_line_inliers;
    *line_center = end_points_line_center;

    if (disp_computation_time_) {
      std::chrono::high_resolution_clock::time_point t_cluster_end = std::chrono::high_resolution_clock::now();
      auto cluster_time = std::chrono::duration_cast<std::chrono::microseconds>(t_cluster_end - t_strt).count();
      float cluster_time_ms = static_cast<float>(cluster_time) * 0.001;
      std::cout << "[Line] cluster time: "<< cluster_time_ms << std::endl;
    }

    t_strt = std::chrono::high_resolution_clock::now();
    // Find end points of the handrail and set the handrail_status
    FindCorner(clustered_line_inliers, plane_parameter, end_points_line_center, plane_vector_length);

    if (disp_computation_time_) {
      std::chrono::high_resolution_clock::time_point t_corner_end = std::chrono::high_resolution_clock::now();
      auto corner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_corner_end - t_strt).count();
      float corner_time_ms = static_cast<float>(corner_time) * 0.001;
      std::cout << "[Line] corner time: "<< corner_time_ms << std::endl;
    }
    dist_to_handrail_ = std::min(std::max((*line_center)(2), min_depth_dist_), max_depth_dist_);
    return true;
  }

  // Check x = p + dv point and its location!
  void FindCorner(const std::vector<int>& line_inliers, const Eigen::Vector4f& plane_parameter,
                                  const Eigen::Vector3f& line_center, const float& plane_vector_length) {
    // Detect the end points of the handrail
    std::vector<int> corner_points;
    int fst_end_pixel = line_inliers[0] / depth_width_;
    int scd_end_pixel = line_inliers[line_inliers.size() - 1] / depth_width_;
    int pixel_gap = static_cast<int>(2 * handrail_group_gap_ / y_scale_);
    int check_step = y_step_;
    int max_length_pixel = depth_height_;
    int vertical_width = depth_width_;
    int parallel_width = 1;
    if (vertical_handrail_detect_) {
      fst_end_pixel = line_inliers[0] % depth_width_;
      scd_end_pixel = line_inliers[line_inliers.size() - 1] % depth_width_;
      pixel_gap = static_cast<int>(handrail_group_gap_ / x_scale_);
      check_step = x_step_;
      vertical_width = 1;
      parallel_width = depth_width_;
      max_length_pixel = depth_width_;
    }
    int corner_cnt_thres = pixel_gap / check_step / 2;
    int check_ID_max = static_cast<int>(cloud_.points.size()) - 1;
    bool end_flags[2] = {false, false};
    float wall_gap = ave_handrail_wall_gap_ / 2;
    Eigen::Vector4f tmp_pnt;
    for (int k = 0; k < 2; ++k) {  // k = 0 for first end, k = 1 for second end
      std::vector<int> tmp_corner_points;
      int outlier_pixel = fst_end_pixel;
      int end_ID = line_inliers[0];  // first end ID of line liners
      if (k == 1) {
        outlier_pixel = max_length_pixel - scd_end_pixel;
        end_ID = line_inliers[line_inliers.size() - 1];  // second end ID of line liners
      }
      outlier_pixel = std::min(outlier_pixel, pixel_gap);
      int tot_cnt = 0;
      for (int i = check_step; i < outlier_pixel; i += check_step) {
        std::vector<int> id_vec;
        // Check left diagonal, center, right diagonal direction of each handrail end point
        for (int j = -1; j < 2; ++j) {
          int check_ID = end_ID - i * vertical_width + i * parallel_width * j;
          if (k == 1)
            check_ID = end_ID + i * vertical_width + i * parallel_width * j;
          check_ID = std::min(std::max(check_ID, 0), check_ID_max);
          tmp_pnt << cloud_.points[check_ID].x, cloud_.points[check_ID].y, cloud_.points[check_ID].z, 1;
          float measure_handrail_wall_gap = fabs(tmp_pnt.dot(plane_parameter)) / plane_vector_length;
          if (measure_handrail_wall_gap > wall_gap)
            break;
          id_vec.push_back(check_ID);
        }
        // all left diagonal, center, right diagonal should satisfy the dist condition
        if (id_vec.size() != 3)
          continue;

        ++tot_cnt;
        tmp_corner_points.insert(tmp_corner_points.end(), id_vec.begin(), id_vec.end());
        if (tot_cnt > corner_cnt_thres) {
          end_flags[k] = true;
          corner_points.insert(corner_points.end(), tmp_corner_points.begin(), tmp_corner_points.end());
          break;
        }
      }
    }
    PublishCloud(corner_points, cloud_, &cloud_pub_[13]);

    if (end_flags[0] && end_flags[1])
      curr_handrail_status_ = BOTH_ENDS;
    else if (end_flags[0])
      curr_handrail_status_ = FIRST_END;
    else if (end_flags[1])
      curr_handrail_status_ = SECOND_END;
    else
      curr_handrail_status_ = NO_END;
  }

  void GetBodyTargetPose(const Eigen::Vector3f& line_vector, const Eigen::Vector3f& line_center,
                                         const std::vector<geometry_msgs::Point>& end_point,
                                         const Eigen::Vector3f& plane_vector,
                                         Eigen::Vector3f* target_pos_err, Eigen::Affine3f* i2h) {
    Eigen::Matrix3f r_err;
    r_err.col(0) = plane_vector;
    r_err.col(1) = line_vector.cross(plane_vector);
    r_err.col(2) = line_vector;

    i2h->translation() = line_center;
    i2h->linear() = r_err;

     // Set up transform to get center of gripper, instead of robot
    Eigen::Affine3f g2r;
    Eigen::Vector3f g2r_translation;
    g2r_translation(0) = 0.0;  // The transforms already take the reach into account
    g2r_translation(1) = 0.0;
    g2r_translation(2) = 0.18056;
    g2r.translation() = g2r_translation;
    Eigen::Matrix3f g2r_rot = Eigen::Matrix3f::Identity(3, 3);
    g2r.linear() = g2r_rot;

    // Get a transformation of handrail in the robot frame
    r2h_center_ = r2i_ * (*i2h);

    float target_dist = fabs(r2i_.translation()(0)) + arm_length_;
    (*target_pos_err) = i2h->translation() + target_dist * plane_vector;

    Eigen::Affine3f esti_r2h_center = r2h_center_;  // Measured average location of line points
    Eigen::Vector3f esti_r2h_pos = r2h_center_.translation();
    if (end_point.size() != 0) {
      Eigen::Vector3f either_end;
      either_end << end_point[0].x, end_point[0].y, end_point[0].z;
      float target_axis_val = either_end(1);
      if (vertical_handrail_detect_)
        target_axis_val = either_end(0);
      if (target_axis_val > 0)
        esti_r2h_pos = g2r * r2i_ * (either_end - (0.5 * fix_handrail_length_ * line_vector));
      else
        esti_r2h_pos = g2r * r2i_ * (either_end + (0.5 * fix_handrail_length_ * line_vector));
    }

    esti_r2h_center.translation() = esti_r2h_pos;
    r2h_center_ = esti_r2h_center;
    (*i2h) = r2i_.inverse() * r2h_center_;
  }

  void GetFeaturePoints(const std::vector<int>& plane_inliers,
                                        const std::vector<int>& line_inliers) {
    int num_line_features = 4;
    int num_plane_features = 6;
    int tot_features = num_line_features + num_plane_features;

    std::vector<int> feature_idx;
    feature_idx.reserve(tot_features);
    std::vector<int> line_point_indices;
    std::vector<int> plane_point_indices;

    // Get feature points for each step from handrail
    int line_step = line_inliers.size() / num_line_features;
    int steps = 0;
    for (int i = 0; i < num_line_features && steps < static_cast<int>(line_inliers.size()); ++i) {
      feature_idx.push_back(line_inliers[steps]);
      line_point_indices.push_back(line_inliers[steps]);
      steps += line_step;
    }

    // Get feature points randomly from plane
    // Define minimum dist between each sampled points
    float fov_x = 62.0, fov_y = 45.0;
    float fov_x_half_tan = static_cast<float>(tan(fov_x * 0.5 * (M_PI / 180.0)));
    float fov_y_half_tan = static_cast<float>(tan(fov_y * 0.5 * (M_PI / 180.0)));
    float half_tan = std::min(fov_x_half_tan, fov_y_half_tan);
    float feature_dist_thres = (half_tan * cloud_.points[plane_inliers[plane_inliers.size() / 2]].z)
                               / sqrt(static_cast<float>((tot_features - num_line_features)));

    Eigen::Vector2f rp, tp;
    int rv, escape_cnt = 0, escape_cnt2 = 0;

    for (int i = num_line_features; i < tot_features; ++i) {
      bool g2g = false;
      escape_cnt2 = 0;

      while (!g2g) {
        // Compare dist between the randomly selected point and the previously selected points
        g2g = true;
        rv = rand_r(&seed_) % plane_inliers.size();

        if (!std::isnan(cloud_.points[plane_inliers[rv]].z)) {
          rp << cloud_.points[plane_inliers[rv]].x, cloud_.points[plane_inliers[rv]].y;
          for (auto const& itr : feature_idx) {
            tp << cloud_.points[itr].x, cloud_.points[itr].y;
            if ((rp - tp).norm() < feature_dist_thres) {
              g2g = false;
              ++escape_cnt2;  // count cnt2 when the selected point is close to any of previousl points
              break;
            }
          }
          if (g2g) {
            feature_idx.push_back(plane_inliers[rv]);
            plane_point_indices.push_back(plane_inliers[rv]);
            break;
          } else if (escape_cnt2 > static_cast<int>(feature_idx.size())) {
            ++escape_cnt;
            --i;
            break;
          }
        } else {
          std::cout << "Inf number in plane?!! \n";
          g2g = false;
        }
      }
      if (escape_cnt > tot_features)
        break;
    }

    // Make DepthLandmarks
    dl_.landmarks.resize(feature_idx.size());
    int lm_cnt = 0;
    for (auto const& itr : feature_idx) {
      dl_.landmarks[lm_cnt].u = cloud_.points[itr].x;
      dl_.landmarks[lm_cnt].v = cloud_.points[itr].y;
      dl_.landmarks[lm_cnt].w = cloud_.points[itr].z;
      ++lm_cnt;
    }

    for (int i = 0; i < line_point_indices.size(); ++i){
      const int line_point_index = line_point_indices[i];
      dl_.sensor_t_line_points.push_back(cloud_.points[line_point_index]);
    }
    for (int i = 0; i < plane_point_indices.size(); ++i){
      const int plane_point_index = plane_point_indices[i];
      dl_.sensor_t_plane_points.push_back(cloud_.points[plane_point_index]);
    }
  }

  bool GetXYScale(const std::vector<int>& plane_inliers) {
    if (cloud_.points.size() == 0)
      return false;

    int rand_cnt = 0, escape_cnt = 0;
    int prev_ID = 0;
    const int rand_cnt_max = 21;
    const int escape_cnt_max = rand_cnt_max * 10;
    std::array<float, rand_cnt_max> pixel_diff[2];
    std::array<float, 2> prev_point = {0, 0};
    while (rand_cnt < rand_cnt_max && escape_cnt < escape_cnt_max) {
      int rand_ID = rand_r(&seed_) % plane_inliers.size();
      rand_ID = plane_inliers[rand_ID];
      if (!(cloud_.points[rand_ID].z == 0 || std::isnan(cloud_.points[rand_ID].x)
            || std::isnan(cloud_.points[rand_ID].y) || std::isnan(cloud_.points[rand_ID].z))) {
        if (rand_cnt > 0) {
          int x_fst_ID = rand_ID % depth_width_;
          int x_scd_ID = prev_ID % depth_width_;
          int x_ID_diff = x_fst_ID - x_scd_ID;
          if (x_ID_diff == 0) {
            --rand_cnt;
          } else {
            int y_fst_ID = rand_ID / depth_width_;
            int y_scd_ID = prev_ID / depth_width_;
            int y_ID_diff = y_fst_ID - y_scd_ID;
            if (y_ID_diff == 0) {
              --rand_cnt;
            } else {
              pixel_diff[0][rand_cnt - 1] = fabs(static_cast<float>(cloud_.points[rand_ID].x - prev_point[0])
                                                 / x_ID_diff);
              pixel_diff[1][rand_cnt - 1] = fabs(static_cast<float>(cloud_.points[rand_ID].y - prev_point[1])
                                                 / y_ID_diff);
            }
          }
        }
        prev_ID = rand_ID;
        prev_point[0] = cloud_.points[rand_ID].x;
        prev_point[1] = cloud_.points[rand_ID].y;
        ++rand_cnt;
        ++escape_cnt;
      }
    }
    if (escape_cnt == escape_cnt_max) {
      return false;
    }

    std::sort(pixel_diff[0].begin(), pixel_diff[0].end());
    std::sort(pixel_diff[1].begin(), pixel_diff[1].end());
    float min_scale = 0.001;
    const int median_idx = rand_cnt_max / 2;
    x_scale_ = std::max(pixel_diff[0][median_idx], min_scale);
    y_scale_ = std::max(pixel_diff[1][median_idx], min_scale);

    x_step_ = std::max(static_cast<int>(min_measure_gap_ / x_scale_), 1);
    y_step_ = std::max(static_cast<int>(min_measure_gap_ / y_scale_), 1);

    return true;
  }

  void PublishCloud(const std::vector<int>& idxs, const sensor_msgs::PointCloud& cloud_data,
                                    ros::Publisher* cloud_publisher) {
    if (!disp_pcd_and_tf_)
      return;

    sensor_msgs::PointCloud disp_cloud;
    disp_cloud.points.clear();
    disp_cloud.header = cloud_data.header;
    disp_cloud.header.stamp = ros::Time::now();
    disp_cloud.points.reserve(idxs.size());
    for (auto const& itr : idxs) {
      disp_cloud.points.push_back(cloud_data.points[itr]);
    }
    cloud_publisher->publish(disp_cloud);
  }

  // Map from 3D point in the world to 2D point in the image plane
  void PublishDepthImage() {
    // Publish depth image
    int max_cell_dist = 255;
    float clipped_dist = 0;
    float depth_diff = max_depth_dist_ - min_depth_dist_;
    for (size_t i = 0; i < image_.data.size(); ++i) {
      int idx = (i % image_.height) * image_.width + i / image_.height;
      idx = ((idx / image_.width + 1) * image_.width - 1) - idx + (idx / image_.width) * image_.width;
      float measure_dist = static_cast<float>(cloud_.points[i].z);
      if (measure_dist == 0) {
        image_.data[idx] = 0;
      } else {
        clipped_dist = std::min(std::max(min_depth_dist_, measure_dist), max_depth_dist_)
                       - min_depth_dist_;
        image_.data[idx] = max_cell_dist * (1.0 - clipped_dist / depth_diff);
      }
    }
    image_pub_.publish(image_);
  }

  void PublishNoMarker() {
    Eigen::Vector3f zero_pos_err = Eigen::Vector3f::Zero();
    zero_pos_err << -100, -100, -100;
    Eigen::Affine3f zero_pose = Eigen::Affine3f::Identity();
    zero_pose.translation() = zero_pos_err;
    std::vector<geometry_msgs::Point> zero_geo_point;
    PublishMarker(zero_pos_err, zero_pose, zero_geo_point);
  }

  // Only for visualization in rviz (ros marker)
  void PublishMarker(const Eigen::Vector3f& target_pos_err, const Eigen::Affine3f& i2h,
                                     const std::vector<geometry_msgs::Point>& end_point) {
    // ROS_WARN("[DEBUG] PublishMarker called");
    float point_size = 0.005;
    visualization_msgs::Marker  points_mk;
    points_mk.header.frame_id = perch_image_frame_;
    points_mk.header.stamp = ros::Time::now();
    points_mk.action = visualization_msgs::Marker::MODIFY;
    points_mk.type = visualization_msgs::Marker::SPHERE_LIST;
    points_mk.scale.x = point_size;
    points_mk.scale.y = point_size;
    points_mk.scale.z = point_size;
    points_mk.color.a = 1.0;

    size_t marker_id = 0;
    geometry_msgs::Point inp_point;
    // [0] handrail_end_points
    points_mk.ns = "handrail_end_points";
    points_mk.color.g = 1.0;
    points_mk.scale.x *= 6.0;
    points_mk.scale.y = points_mk.scale.x;
    points_mk.scale.z = points_mk.scale.x;
    for (auto const& itr : end_point) {
      inp_point.x = itr.x;
      inp_point.y = itr.y;
      inp_point.z = itr.z;
      points_mk.points.push_back(inp_point);
    }
    marker_pub_.publish(points_mk);
    ++marker_id;

    // [1] handrail cylinder
    visualization_msgs::Marker cylinder_mk;
    cylinder_mk.header.frame_id = perch_image_frame_;
    cylinder_mk.header.stamp = ros::Time::now();
    cylinder_mk.ns = "handrail";
    cylinder_mk.action = visualization_msgs::Marker::MODIFY;
    cylinder_mk.type = visualization_msgs::Marker::CYLINDER;
    cylinder_mk.id = marker_id;
    cylinder_mk.scale.x = handrail_width_;
    cylinder_mk.scale.y = cylinder_mk.scale.x;
    cylinder_mk.scale.z = fix_handrail_length_;
    cylinder_mk.color.g = 1.0;
    cylinder_mk.color.a = 0.8;

    Eigen::Quaternionf q(i2h.linear());
    cylinder_mk.pose.position.x = i2h.translation()(0);
    cylinder_mk.pose.position.y = i2h.translation()(1);
    cylinder_mk.pose.position.z = i2h.translation()(2);
    cylinder_mk.pose.orientation.x = q.x();
    cylinder_mk.pose.orientation.y = q.y();
    cylinder_mk.pose.orientation.z = q.z();
    cylinder_mk.pose.orientation.w = q.w();
    marker_pub_.publish(cylinder_mk);

    // [2] target robot pose
    float cubic_size = 0.35;
    visualization_msgs::Marker target_pose_mk;
    target_pose_mk.header.frame_id = perch_image_frame_;
    target_pose_mk.header.stamp = ros::Time::now();
    target_pose_mk.ns = "target_pose";
    target_pose_mk.action = visualization_msgs::Marker::MODIFY;
    target_pose_mk.type = visualization_msgs::Marker::CUBE;
    target_pose_mk.id = marker_id;
    target_pose_mk.scale.x = cubic_size;
    target_pose_mk.scale.y = cubic_size;
    target_pose_mk.scale.z = cubic_size;
    target_pose_mk.color.r = 1.0;
    target_pose_mk.color.b = 1.0;
    target_pose_mk.color.a = 0.3;
    target_pose_mk.pose.position.x = target_pos_err(0);
    target_pose_mk.pose.position.y = target_pos_err(1);
    target_pose_mk.pose.position.z = target_pos_err(2);
    target_pose_mk.pose.orientation.x = q.x();
    target_pose_mk.pose.orientation.y = q.y();
    target_pose_mk.pose.orientation.z = q.z();
    target_pose_mk.pose.orientation.w = q.w();
    marker_pub_.publish(target_pose_mk);
  }

  void PrintHandrailStatus(const handrailStatus& prev_status,
      const handrailStatus& curr_status) {
    std::array<handrailStatus, 2> tmp_status = {prev_status, curr_status};
    int status_cnt = 0;
    for (auto const& itr_status : tmp_status) {
      switch (itr_status) {
      case NOT_FOUND:
        std::cout << "NOT_FOUND";
        break;
      case BOTH_ENDS:
        std::cout << "BOTH_ENDS";
        break;
      case FIRST_END:
        std::cout << "FIRST_END";
        break;
      case SECOND_END:
        std::cout << "SECOND_END";
        break;
      case NO_END:
        std::cout << "NO_END";
        break;
      }
      ++status_cnt;
      if (status_cnt < static_cast<int>(tmp_status.size()))
        std::cout << " -> ";
    }
    std::cout << std::endl;
  }

  void ConvertRotMatToEuler(const Eigen::Matrix3f& r_mat, Eigen::Vector3f* euler) {
    tf2::Matrix3x3 rot_m;
    for (int i = 0; i < 3; ++i)
      for (int j = 0;  j < 3; ++j)
        rot_m[i][j] = r_mat(i, j);
    double r, p, y, rtod = 180 / M_PI;
    rot_m.getRPY(r, p, y);
    r *= rtod;
    p *= rtod;
    y *= rtod;
    (*euler)(0) = static_cast<float>(r);
    (*euler)(1) = static_cast<float>(p);
    (*euler)(2) = static_cast<float>(y);
  }


 private:
  config_reader::ConfigReader config_;
  ros::Timer config_timer_;

  /// Start of handrail_detect.config parameters
  // Display
  bool disp_computation_time_;
  bool disp_marker_;
  bool disp_pcd_and_tf_;
  bool disp_depth_image_;

  // Handrail description
  float close_dist_to_handrail_;
  float handrail_wall_min_gap_;
  unsigned int seed_;
  float handrail_width_;
  float fix_handrail_length_;
  float handrail_group_gap_;
  float pcd_plane_rate_;
  float width_filter_rate_;
  float arm_length_;

  // Downsampling parameters
  float min_measure_gap_;
  float min_depth_dist_;
  float max_depth_dist_;

  // RANSAC related parameters
  int RANSAC_line_iteration_;
  int RANSAC_plane_iteration_;
  float RANSAC_line_thres_;
  float RANSAC_plane_thres_;

  // Frame name
  std::string body_frame_;
  std::string handrail_frame_;
  std::string perch_image_frame_;

  // robot to image frame
  Eigen::Affine3f r2i_;
  /// End of handrail_detect.config parameters

  // Point cloud
  sensor_msgs::PointCloud cloud_;

  // Depth landmark msg
  ff_msgs::DepthLandmarks dl_;

  // Publisher and subscriber
  ros::NodeHandle *nh_;
  ros::Publisher marker_pub_;
  ros::Publisher landmark_pub_;
  ros::Publisher registration_pub_;
  ros::Publisher image_pub_;
  std::array<ros::Publisher, 14> cloud_pub_;
  ros::ServiceServer enable_srv_;
  ros::Subscriber depth_sub_;

  // Handrail status
  handrailStatus curr_handrail_status_;

  // Depth image
  sensor_msgs::Image image_;

  bool vertical_handrail_detect_;
  float x_scale_;
  float y_scale_;
  float handrail_length_;
  float ave_handrail_wall_gap_;
  float dist_to_handrail_;

  int x_step_;
  int y_step_;
  int depth_width_;
  int depth_height_;

  int frame_count_;
  int handrail_status_cnt_;
  int max_num_handrails_;

  Eigen::Affine3f r2h_center_;

  // Handrail Detection Outlier Rejection
  int rolling_window_cnt_ = 0;
  bool start_ = false;
  double outlier_tolerance_;
  Eigen::Matrix<float, 3, 5> rolling_window_pos_ = Eigen::Matrix<float, 3, 5>::Zero();
  int reject_cnt_ = 0;
};

PLUGINLIB_EXPORT_CLASS(handrail_detect::HandrailDetect, nodelet::Nodelet);

}  // namespace handrail_detect
