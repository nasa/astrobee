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

#include <vive/vive_visualization.h>

#include <math.h>

/**
 * \ingroup tools
 */
namespace vive {

  Visualization::Visualization() {
    tf_listener_ = new tf2_ros::TransformListener(buffer_);
  }

  Visualization::~Visualization() {
    delete tf_listener_;
  }

  void Visualization::Initialize(Tracker const& tracker, int id) {
    tracker_stamped_.tracker = tracker;
    tracker_stamped_.id = id;
    imu_data_.status = false;
    // Initialize pose tracker
    pose_tracker_.first = Eigen::Vector3d(0.0, 0.0, 1.0);
    pose_tracker_.second = Eigen::Matrix3d::Identity();
    return;
  }

  // void Visualization::Set(geometry_msgs::TransformStamped & tf) {
  //   pose_tracker_.first = Eigen::Vector3d(
  //     tf.transform.translation.x,
  //     tf.transform.translation.y,
  //     tf.transform.translation.z);
  //   pose_tracker_.second = Eigen::Quaterniond(
  //     tf.transform.rotation.w,
  //     tf.transform.rotation.x,
  //     tf.transform.rotation.y,
  //     tf.transform.rotation.z).toRotationMatrix();
  //   return;
  // }

  void Visualization::AddImu(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_data_.imu.accel.x = msg->linear_acceleration.x;
    imu_data_.imu.accel.y = msg->linear_acceleration.y;
    imu_data_.imu.accel.z = msg->linear_acceleration.z;
    imu_data_.imu.gyro.x = msg->angular_velocity.x;
    imu_data_.imu.gyro.x = msg->angular_velocity.x;
    imu_data_.imu.gyro.x = msg->angular_velocity.x;
    imu_data_.stamp = msg->header.stamp;
    imu_data_.status = true;
    return;
  }

  void Visualization::AddLight(const ff_msgs::ViveLight::ConstPtr& msg) {
    if (light_data_.find(msg->lighthouse) == light_data_.end()) {
      lighthouse_id_[msg->lighthouse] = light_data_.size()-1;
    }
    if (msg->axis == HORIZONTAL) {
      light_data_[msg->lighthouse].horizontal_lights.clear();
    } else if (msg->axis == VERTICAL) {
      light_data_[msg->lighthouse].vertical_lights.clear();
    }
    for (std::vector<ff_msgs::ViveLightSample>::const_iterator li_it = msg->samples.cbegin();
      li_it != msg->samples.cend(); li_it++) {
      Light light;
      light.sensor_id = li_it->sensor;
      light.timecode = li_it->timecode;
      light.angle = li_it->angle;
      light.length = li_it->length;
      if (msg->axis == HORIZONTAL) {
        light_data_[msg->lighthouse].horizontal_lights[li_it->sensor] = light;
      } else if (msg->axis == VERTICAL) {
        light_data_[msg->lighthouse].vertical_lights[li_it->sensor] = light;
      }
    }
    light_data_[msg->lighthouse].status = true;
    return;
  }

  bool Visualization::GetImu(visualization_msgs::Marker * arrow) {
    if (!imu_data_.status) return false;
    (*arrow).header.frame_id = tracker_stamped_.tracker.serial;
    (*arrow).header.stamp = ros::Time::now();
    (*arrow).id = 3000 * tracker_stamped_.id;
    (*arrow).ns = tracker_stamped_.tracker.serial;
    (*arrow).action = 0;
    geometry_msgs::Point origin;
    geometry_msgs::Point head;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    head.x = imu_data_.imu.accel.x / 20;
    head.y = imu_data_.imu.accel.y / 20;
    head.z = imu_data_.imu.accel.z / 20;
    (*arrow).points.push_back(origin);
    (*arrow).points.push_back(head);
    (*arrow).scale.x = 0.02;
    (*arrow).scale.y = 0.05;
    (*arrow).scale.z = 0.1;
    (*arrow).color.a = 1.0;
    (*arrow).color.r = 0.0;
    (*arrow).color.g = 0.0;
    (*arrow).color.b = 1.0;
    imu_data_.status = false;
    return true;
  }

  bool Visualization::GetLight(visualization_msgs::MarkerArray * directions) {
    if (light_data_.size() == 0) return false;
    for (LightDataVisual::iterator lh_it = light_data_.begin();
      lh_it!= light_data_.end(); lh_it++) {
      if (lh_it->second.horizontal_lights.size() == 0
        || lh_it->second.vertical_lights.size() == 0)
        continue;
      visualization_msgs::Marker direction;
      try {
        geometry_msgs::TransformStamped tf = buffer_.lookupTransform(lh_it->first,
          tracker_stamped_.tracker.serial,
          ros::Time(0));
        pose_tracker_.first = Eigen::Vector3d(tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z);
        pose_tracker_.second = Eigen::Quaterniond(tf.transform.rotation.w,
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z);
      } catch (tf2::TransformException &ex) {}

      for (LightMap::iterator li_it = lh_it->second.horizontal_lights.begin();
        li_it != lh_it->second.horizontal_lights.end(); li_it++) {
        if (lh_it->second.vertical_lights.find(li_it->first)
          == lh_it->second.vertical_lights.end())
          continue;
        if ( lh_it->second.vertical_lights[li_it->first].angle > 1.047
          || lh_it->second.vertical_lights[li_it->first].angle < -1.047
          || li_it->second.angle > 1.047
          || li_it->second.angle < -1.047)
          continue;
        Eigen::Vector3d sensor_position = Eigen::Vector3d(
          tracker_stamped_.tracker.sensors[li_it->first].position.x,
          tracker_stamped_.tracker.sensors[li_it->first].position.y,
          tracker_stamped_.tracker.sensors[li_it->first].position.z);
        geometry_msgs::Point origin;
        geometry_msgs::Point head;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = 0.0;
        Eigen::Vector3d dir_vector = Eigen::Vector3d(
          tan(lh_it->second.vertical_lights[li_it->first].angle),
          tan(-li_it->second.angle),
          1.0);
        dir_vector.normalize();
        dir_vector = dir_vector *
          (pose_tracker_.second * sensor_position + pose_tracker_.first).norm();
        head.x = dir_vector(0);
        head.y = dir_vector(1);
        head.z = dir_vector(2);
        direction.points.push_back(origin);
        direction.points.push_back(head);
      }
      direction.id = 2000 * tracker_stamped_.id + 100 * lighthouse_id_[lh_it->first];
      direction.action = visualization_msgs::Marker::ADD;
      direction.ns = tracker_stamped_.tracker.serial;
      direction.scale.x = 0.005;
      direction.color.a = 1.0;
      direction.color.r = 1.0;
      direction.color.g = 0.0;
      direction.color.b = 0.0;
      direction.header.frame_id = lh_it->first;
      direction.header.stamp = ros::Time::now();
      direction.type = visualization_msgs::Marker::LINE_LIST;
      (*directions).markers.push_back(direction);
      lh_it->second.status = false;
    }
    return true;
  }

  bool Visualization::GetSensors(visualization_msgs::MarkerArray * sensors) {
    for (std::map<uint8_t, Sensor>::iterator sn_it = tracker_stamped_.tracker.sensors.begin();
      sn_it != tracker_stamped_.tracker.sensors.end(); sn_it++) {
      visualization_msgs::Marker sensor;
      sensor.header.frame_id = tracker_stamped_.tracker.serial;
      sensor.header.stamp = ros::Time::now();
      sensor.id = tracker_stamped_.id * 1000 + static_cast<int>(unsigned(sn_it->first));
      sensor.ns = tracker_stamped_.tracker.serial;
      sensor.type = visualization_msgs::Marker::SPHERE;
      sensor.action = visualization_msgs::Marker::ADD;
      sensor.pose.position.x = sn_it->second.position.x;
      sensor.pose.position.y = sn_it->second.position.y;
      sensor.pose.position.z = sn_it->second.position.z;
      sensor.pose.orientation.x = 0.0;
      sensor.pose.orientation.y = 0.0;
      sensor.pose.orientation.z = 0.0;
      sensor.pose.orientation.w = 1.0;
      sensor.scale.x = 0.005;
      sensor.scale.y = 0.005;
      sensor.scale.z = 0.005;
      sensor.color.a = 1.0;
      sensor.color.r = 1.0;
      sensor.color.g = 1.0;
      sensor.color.b = 0.0;
      (*sensors).markers.push_back(sensor);
    }
    return true;
  }

}  // namespace vive
