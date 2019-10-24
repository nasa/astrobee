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
#include <common/thread.h>
#include <common/utils.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sparse_map.pb.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

// Parse information obtained from a ROS bag file.  Take as inputs
// image time stamps (images.csv), ground truth camera positions and
// orientations (ground_truth.csv), and the images themselves.
// Interpolate the camera poses at image time stamps. Save the result
// in a sparse map having only images and cameras (no features).

// See localization/sparse_mapping/Readme.md for more info.

// The images.csv file is obtained by invoking
// rostopic echo -b <bag.bag>  -p /hw/cam_nav  --noarr > <bag.images.csv>
// The ground_truth.csv file is obtained as:
// rostopic echo -b <bag.bag>  -p /loc/ground_truth  > <bag.ground_truth.csv>

// outputs
DEFINE_string(output_map, "output.map",
              "Output file containing the control network with images and camera poses.");

DEFINE_string(image_file, "",
              "File containing the time stamps at which images were acquired.");

DEFINE_string(ground_truth_file, "",
              "File containing measured camera positions.");

DEFINE_string(image_set_dir, "",
              "The directory containing the full set of images extracted from the bag file.");

DEFINE_string(image_subset_dir, "",
              "Put in the map only the images from this subset of the images "
              "(if not specified, all the images will be put in the map).");

// Not used, but for completeness
DEFINE_string(camera_calibration, "",
              "The camera calibration file, in OpenCV's XML format.");
DEFINE_string(detector, "ORGBRISK",
              "Feature detector to use. Options are [FAST, STAR, SIFT, SURF, ORB, "
              "BRISK, ORGBRISK, MSER, GFTT, HARRIS, Dense].");

DEFINE_bool(save_trajectory, false,
            "If true, save the bot trajectory in the format understood by P2: X Y Z QX QY QZ QW 0 0 0 0 0 0).");
DEFINE_string(trajectory_file, "trajectory.csv",
              "Save the trajectory to this file.");

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  // Parse ground truth data
  std::map< std::string, std::vector<double> > ground_truth_data;
  sparse_mapping::ParseCSV(FLAGS_ground_truth_file, &ground_truth_data);

  // Parse image data
  std::map< std::string, std::vector<double> > image_data;
  sparse_mapping::ParseCSV(FLAGS_image_file, &image_data);

  std::vector<std::string> images;
  common::ListFiles(FLAGS_image_set_dir, "jpg", &images);
  if (images.empty())
    LOG(FATAL) << "No input images provided";

  std::vector<double> & image_time = image_data["field.header.stamp"];  // alias
  if (image_time.size() != images.size())
    LOG(FATAL) << "The number of images does not match the number of entries in "
               << FLAGS_image_file;

#if 0
  std::vector<double> const& abs_time = image_data["time"];  // alias
  // (This appears to be no longer necessary, now image_time is
  // absolute.)
  // In the image file, the time in field.header.stamp is
  // relative. Make it absolute by using the mean time in the 'time'
  // field.  field.header.stamp is acquisition time, while 'time' is
  // when data got received by the ros topic subscriber.
  double offset1 = abs_time[0], offset2 = image_time[0];
  double mean1 = 0, mean2 = 0;
  int num = abs_time.size();
  for (int i = 0; i < num; i++) {
    // Since these are big numbers, subtract an offset before finding the mean.
    mean1 += (abs_time[i]  - offset1);
    mean2 += (image_time[i] - offset2);
  }
  mean1 = offset1 + mean1/num;
  mean2 = offset2 + mean2/num;
  for (int i = 0; i < num; i++)
    image_time[i] = (image_time[i] - mean2) + mean1;
#endif

  std::vector<Eigen::Affine3d> cid_to_cam_t;  // interpolated measured pose
  std::vector<std::string> good_images;  // images at which we can interpolate the meas pose
  sparse_mapping::PoseInterpolation(images, image_time, ground_truth_data,
                                    &cid_to_cam_t, &good_images);

  // This is the place at which to use just a subset of the images if
  // desired.
  if (FLAGS_image_subset_dir != "") {
    std::vector<Eigen::Affine3d> cid_to_cam_t2;
    std::vector<std::string> good_images2;
    std::map<std::string, int> image2num;

    for (size_t cid = 0; cid < good_images.size(); cid++) {
      image2num[common::basename(good_images[cid])] = cid;
    }

    std::vector<std::string> sub_images;
    common::ListFiles(FLAGS_image_subset_dir, "jpg", &sub_images);
    for (size_t cid = 0; cid < sub_images.size(); cid++) {
      std::string sub_image = sub_images[cid];
      std::string base_image = common::basename(sub_image);
      auto it = image2num.find(base_image);
      if (it == image2num.end()) continue;
      good_images2.push_back(sub_image);
      cid_to_cam_t2.push_back(cid_to_cam_t[it->second]);
    }
    cid_to_cam_t = cid_to_cam_t2;
    good_images = good_images2;
  }

  LOG(INFO) << "Number of cameras: " << cid_to_cam_t.size() << std::endl;

  if (FLAGS_save_trajectory) {
    // Save the bot positions and orientations before switching from
    // bot to image coordinate system
    LOG(INFO) << "Writing: " << FLAGS_trajectory_file;
    std::ofstream tr(FLAGS_trajectory_file.c_str());
    tr.precision(18);
    for (size_t cid = 0; cid < cid_to_cam_t.size(); cid++) {
      // std::cout << cid_to_cam_t[cid].linear() << std::endl << std::endl;
      // Format: X Y Z QX QY QZ QW 0 0 0 0 0 0
      Eigen::Vector3d P = cid_to_cam_t[cid].translation();
      tr << P[0] << ' ' << P[1] << ' ' << P[2] << ' ';

      // Convert from body-to-world to world-to-body
      Eigen::Matrix3d TI = cid_to_cam_t[cid].linear().inverse();
      Eigen::Quaternion<double> Q(TI);
      tr << Q.x() << ' ' << Q.y() << ' ' << Q.z() << ' ' << Q.w()
         << " 0 0 0 0 0 0" << std::endl;
    }
    tr.close();
  }

  // Go from measuring robot position/orientation, to camera position/orientation.
  // Use info from communications/ff_frame_store/launch/ff_frame_store.launch
  Eigen::Affine3d CamToBot;
  CamToBot.translation()   <<  0.203,  -0.157,    -0.0303;   // P3
  CamToBot.linear()
    = Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5).toRotationMatrix();
  for (size_t cid = 0; cid < cid_to_cam_t.size(); cid++)
    cid_to_cam_t[cid] = cid_to_cam_t[cid]*CamToBot;

  // We know that our overhead camera position is inaccurate, by a lot
  // actually.  That results in inaccurate measurements for the bot
  // camera.  As a stop gap solution, we apply a correction here. This
  // will need to be revisited.
#if 0
  // This was necessary for P2. For P3 also need a scale change, still
  // to be done.
  Eigen::Vector3d corr; corr << -0.0034905, -0.0264956, -0.1626;
  for (size_t cid = 0; cid < cid_to_cam_t.size(); cid++)
    cid_to_cam_t[cid].translation() += corr;
#endif

  // What we have is the transform from camera to world.
  // Do instead the transform from world to camera.
  for (size_t cid = 0; cid < cid_to_cam_t.size(); cid++)
    cid_to_cam_t[cid] = cid_to_cam_t[cid].inverse();

  // Dummy parameters but which must be set
  cv::Mat image = cv::imread(good_images[0], CV_LOAD_IMAGE_GRAYSCALE);
  camera::CameraParameters params(FLAGS_camera_calibration);
  sparse_mapping::SparseMap map(cid_to_cam_t,
                                good_images, FLAGS_detector, params);

  map.Save(FLAGS_output_map);

#if 0
  // Debug code. Save the bot positions in the parsed map.
  std::string traj_file = "parsed_map.txt";
  LOG(INFO) << "Writing: " << traj_file << std::endl;
  std::ofstream it(traj_file.c_str());
  it.precision(18);
  for (size_t i = 0; i < (cid_to_cam_t).size(); i++) {
    it << (cid_to_cam_t)[i].inverse().translation().transpose() << std::endl;
  }
  it.close();
#endif

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
