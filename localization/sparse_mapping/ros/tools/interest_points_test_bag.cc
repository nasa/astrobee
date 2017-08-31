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

// this program takes matches interest points from the first image in the
// bag to all other interest points. It then prints out the number
// of interest points vs. the difference in position and orientation on all axes

#include <common/init.h>
#include <common/utils.h>
#include <config_reader/config_reader.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <interest_point/matching.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

Eigen::Vector3f QuatToEuler(const Eigen::Quaternionf & q) {
  Eigen::Vector3f euler;
  float q2q2 = q.y() * q.y();
  euler.x() = atan2(2 * (q.x() * q.w() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q2q2));
  float arg = std::max(-1.0f, std::min(1.0f, 2 * (q.y() * q.w() - q.x() * q.z())));
  euler.y() = asin(arg);
  euler.z() = atan2(2 * (q.x() * q.y() + q.z() * q.w()), 1 - 2 * (q2q2 + q.z() * q.z()));
  return euler;
}

void WriteResults(FILE* f, int matches, const Eigen::Vector3f & p, const Eigen::Quaternionf & q) {
  if (f == NULL)
    return;
  Eigen::Vector3f euler = QuatToEuler(q);
  fprintf(f, "%d ", matches);
  fprintf(f, "%g %g %g ", p.x(), p.y(), p.z());
  fprintf(f, "%g %g %g\n", euler.x(), euler.y(), euler.z());
}

void ReadParams(interest_point::FeatureDetector* detector) {
  config_reader::ConfigReader config;
  config.AddFile("localization.config");
  if (!config.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }

  int min_features, max_features, brisk_threshold, detection_retries;
  if (!config.GetInt("min_features", &min_features))
    ROS_FATAL("min_features not specified in localization.");
  if (!config.GetInt("max_features", &max_features))
    ROS_FATAL("max_features not specified in localization.");
  if (!config.GetInt("brisk_threshold", &brisk_threshold))
    ROS_FATAL("brisk_threshold not specified in localization.");
  if (!config.GetInt("detection_retries", &detection_retries))
    ROS_FATAL("detection_retries not specified in localization.");
  detector->Reset("ORGBRISK", min_features, max_features, brisk_threshold, detection_retries);
}

void DetectImageFeatures(interest_point::FeatureDetector & detector, sensor_msgs::ImageConstPtr & image_msg,
                         cv::Mat* description) {
  std::vector<cv::KeyPoint> keypoints;
  cv_bridge::CvImageConstPtr image;
  try {
    image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_FATAL("cv_bridge exception: %s", e.what());
    return;
  }
  detector.Detect(image->image, &keypoints, description);
}

void InterpolateGroundTruth(const geometry_msgs::PoseStampedConstPtr & last_gt,
                            const geometry_msgs::PoseStampedConstPtr & next_gt,
                            const ros::Time & last_gt_time,
                            const ros::Time & next_gt_time, const ros::Time & image_time,
                            Eigen::Quaternionf * q, Eigen::Vector3f * p) {
  Eigen::Quaternionf q_prev(last_gt->pose.orientation.w, last_gt->pose.orientation.x,
                            last_gt->pose.orientation.y, last_gt->pose.orientation.z);
  Eigen::Quaternionf q_next(next_gt->pose.orientation.w, next_gt->pose.orientation.x,
                            next_gt->pose.orientation.y, next_gt->pose.orientation.z);
  Eigen::Vector3f p_prev(last_gt->pose.position.x, last_gt->pose.position.y, last_gt->pose.position.z);
  Eigen::Vector3f p_next(next_gt->pose.position.x, next_gt->pose.position.y, next_gt->pose.position.z);
  float u = (image_time - last_gt_time).toSec() / (next_gt_time - last_gt_time).toSec();
  *q = q_prev.slerp(u, q_next);
  *p = p_prev + u * (p_next - p_prev);
}

int main(int argc, char ** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 2) {
    LOG(INFO) << "Usage: " << argv[0] << " bag.bag";
    exit(0);
  }

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  interest_point::FeatureDetector detector("ORGBRISK");
  ReadParams(&detector);

  FILE* f = NULL;
  std::string name = argv[1];
  name.erase(name.length() - 3);
  name = name + "txt";
  f = fopen(name.c_str(), "w");

  std::vector<std::string> image_topics, truth_topics;
  image_topics.push_back("/hw/cam_nav");
  truth_topics.push_back("/loc/ground_truth");
  rosbag::View image_view(bag, rosbag::TopicQuery(image_topics));
  rosbag::View truth_view(bag, rosbag::TopicQuery(truth_topics));

  rosbag::View::iterator it_gt = truth_view.begin();
  rosbag::View::iterator it_image = image_view.begin();

  geometry_msgs::PoseStampedConstPtr last_gt = (*it_gt).instantiate<geometry_msgs::PoseStamped>();
  ros::Time last_gt_time = (*it_gt).getTime();
  it_gt++;

  bool first = true;

  cv::Mat description_original;

  Eigen::Quaternionf start_q(1, 0, 0, 0);
  Eigen::Vector3f start_p(0, 0, 0);
  int progress = 0;
  while (it_gt != truth_view.end()) {
    geometry_msgs::PoseStampedConstPtr next_gt = (*it_gt).instantiate<geometry_msgs::PoseStamped>();
    ros::Time next_gt_time = (*it_gt).getTime();

    while (it_image != image_view.end() && (*it_image).getTime() < next_gt_time) {
      progress++;
      if (first && (*it_image).getTime() < last_gt_time) {
          it_image++;
          continue;
      }
      sensor_msgs::ImageConstPtr image_msg = (*it_image).instantiate<sensor_msgs::Image>();

      Eigen::Quaternionf q;
      Eigen::Vector3f p;
      InterpolateGroundTruth(last_gt, next_gt, last_gt_time, next_gt_time, (*it_image).getTime(), &q, &p);

      if (first) {
        DetectImageFeatures(detector, image_msg, &description_original);
        start_p = p;
        start_q = q;
        first = false;
      } else {
        cv::Mat description;
        DetectImageFeatures(detector, image_msg, &description);
        std::vector<cv::DMatch> matches;
        interest_point::FindMatches(description, description_original, &matches);
        WriteResults(f, matches.size(), p - start_p, q * start_q.conjugate());
      }

      common::PrintProgressBar(stdout, static_cast<float>(progress) / image_view.size());
      it_image++;
    }

    it_gt++;
    last_gt_time = next_gt_time;
    last_gt = next_gt;
  }
  bag.close();
  if (f != NULL)
    fclose(f);
  common::PrintProgressBar(stdout, 1.0);
  printf("\n");
}

