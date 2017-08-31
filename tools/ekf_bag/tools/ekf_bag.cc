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

#include <common/init.h>
#include <common/utils.h>
#include <config_reader/config_reader.h>
#include <gnc_ros_wrapper/ekf.h>
#include <sparse_mapping/sparse_map.h>
#include <localization_node/localization.h>
#include <lk_optical_flow/lk_optical_flow.h>
#include <image_transport/image_transport.h>
#include <ff_util/ff_names.h>
#include <camera/camera_params.h>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

DEFINE_double(sparse_map_delay, 0.3, "The delay to process a sparse mapping image.");
DEFINE_double(of_delay, 0.05, "The delay to process an OF update.");
DEFINE_string(save_output, "", "Specify a file to save the results to.");

bool start_time_set = false;
ros::Time start_time;

Eigen::Vector3f QuatToEuler(const geometry_msgs::Quaternion & q) {
  Eigen::Vector3f euler;
  float q2q2 = q.y * q.y;
  euler.x() = atan2(2 * (q.x * q.w + q.y * q.z), 1 - 2 * (q.x * q.x + q2q2));
  float arg = std::max(-1.0, std::min(1.0, 2 * (q.y * q.w - q.x * q.z)));
  euler.y() = asin(arg);
  euler.z() = atan2(2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q2q2 + q.z * q.z));
  return euler;
}

Eigen::Vector3f QuatToEuler(const Eigen::Quaternionf & q) {
  Eigen::Vector3f euler;
  float q2q2 = q.y() * q.y();
  euler.x() = atan2(2 * (q.x() * q.w() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q2q2));
  float arg = std::max(-1.0f, std::min(1.0f, 2 * (q.y() * q.w() - q.x() * q.z())));
  euler.y() = asin(arg);
  euler.z() = atan2(2 * (q.x() * q.y() + q.z() * q.w()), 1 - 2 * (q2q2 + q.z() * q.z()));
  return euler;
}

void WriteResults(FILE* f, const ff_msgs::EkfState & s) {
  if (f == NULL)
    return;
  if (!start_time_set) {
    start_time = s.header.stamp;
    start_time_set = true;
  }
  fprintf(f, "EKF %g ", (s.header.stamp - start_time).toSec());
  fprintf(f, "%g %g %g ", s.pose.position.x, s.pose.position.y, s.pose.position.z);
  Eigen::Vector3f euler = QuatToEuler(s.pose.orientation);
  fprintf(f, "%g %g %g ", euler.x(), euler.y(), euler.z());
  fprintf(f, "%g %g %g ", s.velocity.x, s.velocity.y, s.velocity.z);
  fprintf(f, "%g %g %g ", s.omega.x, s.omega.y, s.omega.z);
  fprintf(f, "%g %g %g ", s.accel.x, s.accel.y, s.accel.z);
  fprintf(f, "%g %g %g ", s.accel_bias.x, s.accel_bias.y, s.accel_bias.z);
  fprintf(f, "%g %g %g ", s.gyro_bias.x, s.gyro_bias.y, s.gyro_bias.z);
  fprintf(f, "%d ", s.confidence);
  fprintf(f, "%d ", s.status);
  fprintf(f, "%d %d", s.ml_count, s.of_count);
  for (int i = 0; i < 15; i++)
    fprintf(f, " %g", s.cov_diag[i]);
  for (int i = 0; i < 50; i++)
    fprintf(f, " %g", s.ml_mahal_dists[i]);
  fprintf(f, "\n");
}

void WriteOFRegister(FILE* f, const ff_msgs::Feature2dArray & of, const camera::CameraParameters & params) {
  if (f == NULL || !start_time_set)
    return;
  fprintf(f, "OF %g ", (of.header.stamp - start_time).toSec());
  fprintf(f, "%d ", static_cast<int>(of.feature_array.size()));
  for (unsigned int i = 0; i < of.feature_array.size(); i++) {
    Eigen::Vector2d input(of.feature_array[i].x, of.feature_array[i].y);
    Eigen::Vector2d output;
    params.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>(input, &output);
    fprintf(f, "%d %g %g ", of.feature_array[i].id, output.x(), output.y());
  }
  fprintf(f, "\n");
}

void WriteVLRegister(FILE* f, const gnc_ros_wrapper::Ekf & ekf, const ff_msgs::VisualLandmarks & vl,
    const camera::CameraParameters & params) {
  if (f == NULL || !start_time_set)
    return;
  fprintf(f, "VL %g ", (vl.header.stamp - start_time).toSec());
  fprintf(f, "%d ", static_cast<int>(vl.landmarks.size()));
  Eigen::Vector3f trans = ekf.GetNavCamToBody().translation().cast<float>();
  Eigen::Quaternionf q1(0, trans.x(), trans.y(), trans.z());
  Eigen::Quaternionf q2(vl.pose.orientation.w, vl.pose.orientation.x, vl.pose.orientation.y, vl.pose.orientation.z);
  Eigen::Quaternionf cam_to_body_q(ekf.GetNavCamToBody().rotation().cast<float>());
  cam_to_body_q.w() = -cam_to_body_q.w();
  q2 = q2 * cam_to_body_q;
  Eigen::Quaternionf t = (q2 * q1) * q2.conjugate();
  Eigen::Vector3f r(vl.pose.position.x, vl.pose.position.y, vl.pose.position.z);
  r -= Eigen::Vector3f(t.x(), t.y(), t.z());
  fprintf(f, "%g %g %g ", r.x(), r.y(), r.z());
  Eigen::Vector3f euler = QuatToEuler(q2);
  fprintf(f, "%g %g %g ", euler.x(), euler.y(), euler.z());
  for (unsigned int i = 0; i < vl.landmarks.size(); i++) {
    Eigen::Vector2d input(vl.landmarks[i].u, vl.landmarks[i].v);
    Eigen::Vector2d output;
    params.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>(input, &output);
    fprintf(f, "%g %g %g %g %g ", output.x(), output.y(), vl.landmarks[i].x, vl.landmarks[i].y, vl.landmarks[i].z);
  }
  fprintf(f, "\n");
}

void WriteGroundTruth(FILE* f, const geometry_msgs::PoseStamped & gt) {
  if (f == NULL || !start_time_set)
    return;
  fprintf(f, "GT %g ", (gt.header.stamp - start_time).toSec());
  fprintf(f, "%g %g %g ", gt.pose.position.x, gt.pose.position.y, gt.pose.position.z);
  Eigen::Vector3f euler = QuatToEuler(gt.pose.orientation);
  fprintf(f, "%g %g %g\n", euler.x(), euler.y(), euler.z());
}

void EstimateBias(const rosbag::Bag & bag, gnc_ros_wrapper::Ekf* ekf) {
  std::vector<std::string> topics;
  topics.push_back(std::string("/") + TOPIC_HARDWARE_IMU);

  // get the start time
  rosbag::View temp(bag, rosbag::TopicQuery(topics));
  ros::Time start = temp.getBeginTime();

  // look in only the first few seconds
  rosbag::View early_imu(bag, rosbag::TopicQuery(topics), start, start + ros::Duration(5.0));

  int count = 0;
  Eigen::Vector3f gyro(0, 0, 0), accel(0, 0, 0);
  for (rosbag::MessageInstance const m : early_imu) {
    if (m.isType<sensor_msgs::Imu>()) {
      sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
      accel.x() += imu->linear_acceleration.x;
      accel.y() += imu->linear_acceleration.y;
      accel.z() += imu->linear_acceleration.z;
      gyro.x() += imu->angular_velocity.x;
      gyro.y() += imu->angular_velocity.y;
      gyro.z() += imu->angular_velocity.z;
      count++;
    }
  }
  accel = accel / count;
  gyro = gyro / count;
  ekf->SetBias(gyro, accel);
  printf("Set bias from %d initial observations.\n", early_imu.size());
}

void ReadParams(lk_optical_flow::LKOpticalFlow* of, localization_node::Localizer* loc, gnc_ros_wrapper::Ekf* ekf) {
  config_reader::ConfigReader config;
  config.AddFile("gnc.config");
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  config.AddFile("localization.config");
  config.AddFile("optical_flow.config");
  if (!config.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }

  ekf->ReadParams(&config);
  of->ReadParams(&config);
  loc->ReadParams(&config);
}

int main(int argc, char ** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 3) {
    LOG(INFO) << "Usage: " << argv[0] << " map.map bag.bag";
    exit(0);
  }

  rosbag::Bag bag;
  bag.open(argv[2], rosbag::bagmode::Read);

  sparse_mapping::SparseMap map(argv[1], true);
  lk_optical_flow::LKOpticalFlow of;
  localization_node::Localizer loc(&map);
  cmc_msg cmc;
  gnc_ros_wrapper::Ekf ekf(&cmc);

  ReadParams(&of, &loc, &ekf);

  FILE* f = NULL;
  if (FLAGS_save_output != "") {
    f = fopen(FLAGS_save_output.c_str(), "w");
  }

  EstimateBias(bag, &ekf);

  std::vector<std::string> topics;
  topics.push_back(std::string("/") + TOPIC_HARDWARE_IMU);
  topics.push_back(std::string("/") + TOPIC_HARDWARE_NAV_CAM);
  topics.push_back(std::string("/") + TOPIC_LOCALIZATION_TRUTH);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool processing_of = false, processing_sparse_map = false;
  int of_id = 0, vl_id = 0;
  ros::Time of_send_time, vl_send_time;
  ff_msgs::Feature2dArray of_features;
  ff_msgs::VisualLandmarks vl_features;

  int progress = 0;
  geometry_msgs::Quaternion quat;  // Potentially use ground truth quaternion for MGTF gravity correction
  for (rosbag::MessageInstance const m : view) {
    progress++;
    // actually send the visual messages
    if (processing_of && m.getTime() >= of_send_time) {
      ekf.OpticalFlowUpdate(of_features);
      processing_of = false;
    }
    if (processing_sparse_map && m.getTime() >= vl_send_time) {
      ekf.SparseMapUpdate(vl_features);
      processing_sparse_map = false;
    }

    if (m.isType<sensor_msgs::Image>()) {
      sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      if (!processing_of) {
        // send of registration
        ff_msgs::CameraRegistration r;
        r.header = std_msgs::Header();
        r.header.stamp = m.getTime();
        r.camera_id = ++of_id;
        ekf.OpticalFlowRegister(r);
        // do the processing now, but send it later
        of_features.feature_array.clear();
        of.OpticalFlow(image_msg, &of_features);
        of_features.camera_id = of_id;
        processing_of = true;
        of_send_time = m.getTime() + ros::Duration(FLAGS_of_delay);

        WriteOFRegister(f, of_features, map.GetCameraParameters());
      }
      // now do sparse map
      if (!processing_sparse_map) {
        // send registration
        ff_msgs::CameraRegistration r;
        r.header = std_msgs::Header();
        r.header.stamp = m.getTime();
        r.camera_id = ++vl_id;
        ekf.SparseMapRegister(r);
        // do processing now, but send it later
        cv_bridge::CvImageConstPtr image;
        try {
          image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          break;
        }
        vl_features.landmarks.clear();
        loc.Localize(image, &vl_features);
        vl_features.camera_id = vl_id;
        processing_sparse_map = true;
        vl_send_time = m.getTime() + ros::Duration(FLAGS_sparse_map_delay);

        WriteVLRegister(f, ekf, vl_features, map.GetCameraParameters());
      }
      common::PrintProgressBar(stdout, static_cast<float>(progress) / view.size());
    } else if (m.isType<sensor_msgs::Imu>()) {
      sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      ekf.PrepareStep(*imu_msg.get(), quat);  // Pass a quaternion in to do MGTF gravity correction if needed
      ff_msgs::EkfState state;
      int ret = ekf.Step(&state);
      if (!ret)
        continue;
      WriteResults(f, state);
    } else if (m.isType<geometry_msgs::PoseStamped>()) {
      geometry_msgs::PoseStampedConstPtr gt_msg = m.instantiate<geometry_msgs::PoseStamped>();
      quat = gt_msg->pose.orientation;  // Cache the ground truth for MGTF gravity correction
      WriteGroundTruth(f, *gt_msg.get());
    }
  }
  bag.close();
  if (f != NULL)
    fclose(f);
  printf("\n");
}

