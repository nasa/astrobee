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

// ROS includes
#include <ros/ros.h>

// TF2
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// FSW includes
#include <config_reader/config_reader.h>

// FSW messages
#include <ff_msgs/DepthLandmarks.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/SetBool.h>

// Camera model
#include <camera/camera_model.h>
#include <camera/camera_params.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STL includes
#include <string>

namespace gazebo {

class GazeboSensorPluginHandrailDetect : public FreeFlyerSensorPlugin {
 public:
  // Plugin constructor
  GazeboSensorPluginHandrailDetect() :
    FreeFlyerSensorPlugin("handrail_detect", "perch_cam", true),
      active_(true) {}

  // Plugin destructor
  ~GazeboSensorPluginHandrailDetect() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginHandrailDetect requires a parent depth sensor.\n";
      return;
    }
    // Build the nav cam Camera Model
    config_.AddFile("cameras.config");
    config_.AddFile("simulation/handrail_detect.config");
    if (!config_.ReadFiles()) {
        ROS_ERROR("Failed to read config files.");
        return;
    }

    if (!config_.GetReal("rate", &rate_))
      ROS_FATAL("Could not read the rate parameter.");

    if (!config_.GetReal("delay_camera", &delay_camera_))
      ROS_FATAL("Could not read the delay_camera parameter.");

    if (!config_.GetReal("delay_features", &delay_features_))
      ROS_FATAL("Could not read the delay_features parameter.");

    if (!config_.GetReal("near_clip", &near_clip_))
      ROS_FATAL("Could not read the near_clip parameter.");

    if (!config_.GetReal("far_clip", &far_clip_))
      ROS_FATAL("Could not read the far_clip parameter.");

    if (!config_.GetUInt("num_samp", &num_samp_))
      NODELET_ERROR("Could not read the num_samp parameter.");

    if (!config_.GetUInt("num_features", &num_features_))
      ROS_FATAL("Could not read the num_features parameter.");

    // Create a publisher for the registration messages
    pub_reg_ = nh->advertise<ff_msgs::CameraRegistration>(
      TOPIC_LOCALIZATION_HR_REGISTRATION, 1);

    // Create a publisher for the feature messages
    pub_feat_ = nh->advertise<ff_msgs::DepthLandmarks>(
      TOPIC_LOCALIZATION_HR_FEATURES, 1);

    // Create a shape for collision testing
    GetWorld()->GetPhysicsEngine()->InitForThread();
    shape_ = boost::dynamic_pointer_cast<physics::RayShape>(GetWorld()
        ->GetPhysicsEngine()->CreateShape("ray", physics::CollisionPtr()));

    // Only do this once
    msg_feat_.header.frame_id = std::string(FRAME_NAME_WORLD);
    msg_reg_.header.frame_id = std::string(FRAME_NAME_WORLD);
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(ros::NodeHandle *nh) {
    // Sercide for enabling mapped landmarks
    srv_enable_ = nh->advertiseService(SERVICE_LOCALIZATION_HR_ENABLE,
      &GazeboSensorPluginHandrailDetect::EnableService, this);

    // Timer triggers registration
    timer_registration_ = nh->createTimer(ros::Duration(ros::Rate(rate_)),
      &GazeboSensorPluginHandrailDetect::SendRegistration, this, false, true);

    // Timer triggers features
    timer_features_ = nh->createTimer(ros::Duration(0.8 / rate_),
      &GazeboSensorPluginHandrailDetect::SendFeatures, this, true, false);
  }

  // Enable or disable the feature timer
  bool EnableService(ff_msgs::SetBool::Request & req,
                     ff_msgs::SetBool::Response & res) {
    active_ = req.enable;
    res.success = true;
    return true;
  }


  // Send a registration pulse
  void SendRegistration(ros::TimerEvent const& event) {
    if (!active_) return;

    // Add a short delay between the features and new registration pulse
    timer_features_.stop();
    timer_features_.start();

    // Send the registration pulse
    msg_reg_.header.stamp = ros::Time::now() + ros::Duration(delay_camera_);
    msg_reg_.camera_id++;
    pub_reg_.publish(msg_reg_);
    ros::spinOnce();

    // Copy over the camera id to the feature message
    msg_feat_.camera_id = msg_reg_.camera_id;
    msg_feat_.landmarks.clear();

    // Handle the transform for all sensor types
    Eigen::Affine3d wTb = (
        Eigen::Translation3d(
          GetModel()->GetWorldPose().pos.x,
          GetModel()->GetWorldPose().pos.y,
          GetModel()->GetWorldPose().pos.z) *
        Eigen::Quaterniond(
          GetModel()->GetWorldPose().rot.w,
          GetModel()->GetWorldPose().rot.x,
          GetModel()->GetWorldPose().rot.y,
          GetModel()->GetWorldPose().rot.z));
    Eigen::Affine3d bTc = (
        Eigen::Translation3d(
          sensor_->Pose().Pos().X(),
          sensor_->Pose().Pos().Y(),
          sensor_->Pose().Pos().Z()) *
        Eigen::Quaterniond(
          sensor_->Pose().Rot().W(),
          sensor_->Pose().Rot().X(),
          sensor_->Pose().Rot().Y(),
          sensor_->Pose().Rot().Z()));
    Eigen::Affine3d wTc = wTb * bTc;

    // Initialize the camera paremeters
    static camera::CameraParameters cam_params(&config_, "perch_cam");
    static camera::CameraModel camera(Eigen::Vector3d(0, 0, 0),
      Eigen::Matrix3d::Identity(), cam_params);

    // Look for all models with the name "handrail". This will find every
    // handrail of every size that exists in the gazebo simulation
    physics::ModelPtr closest_model = nullptr;
    double closest_distance = 0.0;
    Eigen::Affine3d closest_pose = Eigen::Affine3d::Identity();
    for (size_t i = 0; i < GetWorld()->GetModelCount(); i++) {
      // Find only models with "handrail" in their name
      physics::ModelPtr model = GetWorld()->GetModel(i);
      // Determine the handrail based on the name
      if (model->GetName().find("handrail") == std::string::npos)
        continue;
      // Get the handrail to world transform
      Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
      q.x() = model->GetWorldPose().rot.x;
      q.y() = model->GetWorldPose().rot.y;
      q.z() = model->GetWorldPose().rot.z;
      q.w() = model->GetWorldPose().rot.w;
      Eigen::Affine3d wTh = Eigen::Affine3d::Identity();
      wTh.translation()[0] = model->GetWorldPose().pos.x;
      wTh.translation()[1] = model->GetWorldPose().pos.y;
      wTh.translation()[2] = model->GetWorldPose().pos.z;
      wTh.linear() = q.toRotationMatrix();
      // Handrail to camera frame
      Eigen::Affine3d cTh =  wTc.inverse() * wTh;
      // Check if the handrail center is in view. A consequence of this is that
      // the center of the handrail must be in view in order to be detected.
      if (!camera.IsInFov(cTh.translation()))
        continue;
      // Get the distance between the robot and
      double distance = cTh.translation().norm();
      // It is in theory possible to hav multiple handrails detected, so we
      // will only use the closest handrail, as measured by Euclidean distance.
      if (closest_model == nullptr || distance < closest_distance) {
        closest_model = model;
        closest_distance = distance;
        closest_pose = cTh;
      }
    }

    // Check if we managed to find a handrail
    if (closest_model == nullptr) {
      ROS_DEBUG_STREAM("No handrail in view");
      return;
    } else {
      ROS_DEBUG_STREAM("Closest handrail: " << closest_model->GetName());
    }

    // Update the handrail transform for the rest of the system
    static tf2_ros::StaticTransformBroadcaster bc;
    Eigen::Affine3d wTh = wTc * closest_pose;
    Eigen::Quaterniond wTc_q(wTh.rotation());
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "world";
    tf.child_frame_id = "handrail/body";
    tf.transform.translation.x = wTh.translation()[0];
    tf.transform.translation.y = wTh.translation()[1];
    tf.transform.translation.z = wTh.translation()[2];
    tf.transform.rotation.x = wTc_q.x();
    tf.transform.rotation.y = wTc_q.y();
    tf.transform.rotation.z = wTc_q.z();
    tf.transform.rotation.w = wTc_q.w();
    bc.sendTransform(tf);

    // Assemble the feature message. The local_pose of the handrail is the
    // pose of the center of the handrail "h" in the camera frame "c".
    msg_feat_.local_pose.position.x = closest_pose.translation().x();
    msg_feat_.local_pose.position.y = closest_pose.translation().y();
    msg_feat_.local_pose.position.z = closest_pose.translation().z();
    Eigen::Quaterniond q(closest_pose.rotation());
    msg_feat_.local_pose.orientation.w = q.w();
    msg_feat_.local_pose.orientation.x = q.x();
    msg_feat_.local_pose.orientation.y = q.y();
    msg_feat_.local_pose.orientation.z = q.z();
    msg_feat_.end_seen = true;
    msg_feat_.update_global_pose = true;

    // Initialize and lock the physics engine for this scope
    {
      GetWorld()->GetPhysicsEngine()->InitForThread();
      boost::unique_lock<boost::recursive_mutex> lock(*(
        GetWorld()->GetPhysicsEngine()->GetPhysicsUpdateMutex()));

      // Create a new ray that passes through the image plane
      size_t i = 0;
      for (; i < num_samp_ && msg_feat_.landmarks.size() < num_features_; i++) {
        // Get the image coordinate to sample
        Eigen::Vector2i img(
          rand() % (2 * camera.GetParameters().GetDistortedSize()[0])
            - camera.GetParameters().GetDistortedSize()[0],
          rand() % (2 * camera.GetParameters().GetDistortedSize()[1])
            - camera.GetParameters().GetDistortedSize()[1]);

        // Get a ray through a random image coordinate
        Eigen::Vector3d ray = camera.Ray(img[0], img[1]);

        // Get the camera coordinate of the ray near and far clips
        Eigen::Vector3d n_w = wTc * (near_clip_ * ray);
        Eigen::Vector3d f_w = wTc * (far_clip_ * ray);

        // Collision detection
        double dist;
        std::string entity;

        // Set the start and end points of the ray
        shape_->SetPoints(
          ignition::math::Vector3d(n_w.x(), n_w.y(), n_w.z()),
          ignition::math::Vector3d(f_w.x(), f_w.y(), f_w.z()));
        shape_->GetIntersection(dist, entity);

        // If we don't have an entity then we didnt collide
        if (entity.empty())
          continue;

        // Calculate the point
        Eigen::Vector3d p_w = n_w + dist * (f_w - n_w).normalized();
        Eigen::Vector3d p_c = wTc.inverse() * p_w;
        Eigen::Vector2d p_i = camera.ImageCoordinates(p_c);

        // Create the landmark message
        ff_msgs::DepthLandmark landmark;
        landmark.u = static_cast<double>(p_i[0]);
        landmark.v = static_cast<double>(p_i[1]);
        landmark.w = p_c.norm();
        msg_feat_.landmarks.push_back(landmark);
      }
    }
  }

  // Send off the features
  void SendFeatures(ros::TimerEvent const& event) {
    if (!active_) return;
    msg_feat_.header.stamp = ros::Time::now();
    pub_feat_.publish(msg_feat_);
  }

 private:
  config_reader::ConfigReader config_;
  ros::Publisher pub_reg_, pub_feat_;
  ros::ServiceServer srv_enable_;
  ros::Timer timer_registration_, timer_features_;
  std::shared_ptr<sensors::DepthCameraSensor> sensor_;
  gazebo::physics::RayShapePtr shape_;
  bool active_;
  ff_msgs::DepthLandmarks msg_feat_;
  ff_msgs::CameraRegistration msg_reg_;
  double rate_;
  double delay_camera_;
  double delay_features_;
  double near_clip_;
  double far_clip_;
  unsigned int num_features_;
  unsigned int num_samp_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginHandrailDetect)

}   // namespace gazebo
