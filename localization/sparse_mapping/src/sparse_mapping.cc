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

#include <sparse_mapping/sparse_mapping.h>
#include <camera/camera_model.h>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message_lite.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Get rid of warning beyond our control
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push
#include <openMVG/tracks/tracks.hpp>
#pragma GCC diagnostic pop

#include <ff_common/utils.h>

#include <fstream>
#include <iostream>
#include <set>

DEFINE_double(min_valid_angle, 1e-2,
              "If all rays converging to a triangulated point make angles "
              "less than this, in degrees, drop it.");

DEFINE_bool(verbose_parsing, false,
              "If true, be more verbose when parsing camera data.");

// Compute the n-weight slerp, analogous to the linear combination
// W[0]*Q[0] + ... + W[n-1]*Q[n-1]. This is experimental.
// We assume the sum of weights is 1.
Eigen::Quaternion<double> sparse_mapping::slerp_n(std::vector<double> const& W,
                                                  std::vector<Eigen::Quaternion<double> > const& Q) {
  if (W.size() != Q.size())
    LOG(FATAL) << "Expecting as many quaternions as weights.";

  if (Q.empty())
    LOG(FATAL) << "Expecting at least one quaternion and weight.";

  if (Q.size() == 1)
    return Q[0];

  if (Q.size() == 2) {
    if (!(std::abs(W[0] + W[1] - 1.0) < 1e-6 && W[0] >= 0 && W[1] >= 0))
      LOG(FATAL) << "Expecting the weights to be >= 0 and sum up to 1.";
    return Q[0].slerp(W[1], Q[1]);
  }

  // Call recursively this function with fewer terms
  double sum = W[0] + W[1];
  if (sum == 0) sum = 1.0;
  Eigen::Quaternion<double> q = Q[0].slerp(W[1]/sum, Q[1]);
  std::vector<double> W2 = W;
  std::vector<Eigen::Quaternion<double> > Q2 = Q;
  W2.erase(W2.begin());
  Q2.erase(Q2.begin());
  W2[0] = sum;
  Q2[0] = q;
  return sparse_mapping::slerp_n(W2, Q2);
}

namespace sparse_mapping {

  // Statistics for filtering
  struct FilterStats{
    int total;
    int small_angle;
    int behind_cam;
    int invalid_reproj;
    int big_reproj_err;
    int num_features;
    FilterStats():total(0), small_angle(0), behind_cam(0), invalid_reproj(0),
                  big_reproj_err(0), num_features(0) {}

    void PrintStats() {
      // Print the stats.
      LOG(INFO) << "Statistics of points to filter out.";
      LOG(INFO) << "Total: " << total;
      LOG(INFO) << "xyz points with small ray angles:     "
                << small_angle << " (" << (100.0*small_angle)/total << " %)";
      LOG(INFO) << "xyz points behind camera:             "
                << behind_cam << " (" << (100.0*behind_cam)/total << " %)";
      LOG(INFO) << "Reprojected outside of image:         "
                << invalid_reproj << " (" << (100.0*invalid_reproj)/total << " %)";
      LOG(INFO) << "Features with big reprojection error: "
                << big_reproj_err << " (" << (100.0*big_reproj_err)/num_features << " %)";
    }
  };


}  // namespace sparse_mapping

// Logic for implementing if two histogram equalization flags are compatible.
// This flag can be either 0 (false), 1 (true), or 2 (unknown). Be tolerant
// of unknown values, but intolerant when true and false are mixed.
void sparse_mapping::HistogramEqualizationCheck(int histogram_equalization1,
                                                int histogram_equalization2) {
  if ( (histogram_equalization1 == 0 && histogram_equalization2 == 1) ||
       (histogram_equalization1 == 1 && histogram_equalization2 == 0) )
    LOG(FATAL) << "Incompatible values of histogram equalization detected.";
}

bool sparse_mapping::IsBinaryDescriptor(std::string const& descriptor) {
  if (descriptor == "OPENSIFT" || descriptor == "SIFT" || descriptor == "SURF")
    return false;
  return true;
}

// Writes the NVM control network format.
void sparse_mapping::WriteNVM(std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                              std::vector<std::string> const& cid_to_filename,
                              std::vector<std::map<int, int> > const& pid_to_cid_fid,
                              std::vector<Eigen::Vector3d> const& pid_to_xyz,
                              std::vector<Eigen::Affine3d> const&
                              cid_to_cam_t_global,
                              double focal_length,
                              std::string const& output_filename) {
  std::fstream f(output_filename, std::ios::out);
  f << "NVM_V3\n";

  CHECK(cid_to_filename.size() == cid_to_keypoint_map.size())
    << "Unequal number of filenames and keypoints";
  CHECK(pid_to_cid_fid.size() == pid_to_xyz.size())
    << "Unequal number of pid_to_cid_fid and xyz measurements";
  CHECK(cid_to_filename.size() == cid_to_cam_t_global.size())
    << "Unequal number of filename and camera transforms";

  // Write camera information
  f << cid_to_filename.size() << std::endl;
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++) {
    // Decompose cam_t_global so that we can write it into a WXY
    // quaternion and an XYZ camera position
    Eigen::Quaterniond q(cid_to_cam_t_global[cid].rotation());
    Eigen::Vector3d t(cid_to_cam_t_global[cid].translation());
    Eigen::Vector3d camera_center =
      - cid_to_cam_t_global[cid].rotation().inverse() * t;

    // The NVM format is a little crazy. When using a quaternion, we
    // write the camera center instead of the t from camera_t_global.
    f << cid_to_filename[cid] << " " << focal_length
      << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
      << camera_center[0] << " " << camera_center[1] << " "
      << camera_center[2] << " " << "0.9 0\n";
  }

  // Write the number of points
  f << pid_to_cid_fid.size() << std::endl;

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    f << pid_to_xyz[pid][0] << " " << pid_to_xyz[pid][1] << " "
      << pid_to_xyz[pid][2] << " 0 0 0 "
      << pid_to_cid_fid[pid].size();

    CHECK(pid_to_cid_fid[pid].size() > 1)
      << "PID " << pid << " has " << pid_to_cid_fid[pid].size() << " measurements";

    for (std::map<int, int>::const_iterator it = pid_to_cid_fid[pid].begin();
         it != pid_to_cid_fid[pid].end(); it++) {
      f << " " << it->first << " " << it->second << " "
        << cid_to_keypoint_map[it->first].col(it->second)[0] << " "
        << cid_to_keypoint_map[it->first].col(it->second)[1];
    }
    f << std::endl;
  }

  // Close the file
  f.flush();
  f.close();
}

// Reads the NVM control network format.
void sparse_mapping::ReadNVM(std::string const& input_filename,
                             std::vector<Eigen::Matrix2Xd > * cid_to_keypoint_map,
                             std::vector<std::string> * cid_to_filename,
                             std::vector<std::map<int, int> > * pid_to_cid_fid,
                             std::vector<Eigen::Vector3d> * pid_to_xyz,
                             std::vector<Eigen::Affine3d> *
                             cid_to_cam_t_global) {
  std::ifstream f(input_filename, std::ios::in);
  std::string token;
  std::getline(f, token);

  // Assert that we start with our NVM token
  if (token.compare(0, 6, "NVM_V3") != 0) {
    LOG(FATAL) << "File doesn't start with NVM token";
  }

  // Read the number of cameras
  ptrdiff_t number_of_cid;
  f >> number_of_cid;
  if (number_of_cid < 1) {
    LOG(FATAL) << "NVM file is missing cameras";
  }

  // Resize all our structures to support the number of cameras we now expect
  cid_to_keypoint_map->resize(number_of_cid);
  cid_to_filename->resize(number_of_cid);
  cid_to_cam_t_global->resize(number_of_cid);
  for (ptrdiff_t cid = 0; cid < number_of_cid; cid++) {
    // Clear keypoints from map. We'll read these in shortly
    cid_to_keypoint_map->at(cid).resize(Eigen::NoChange_t(), 2);

    // Read the line that contains camera information
    double focal, dist1, dist2;
    Eigen::Quaterniond q;
    Eigen::Vector3d c;
    f >> token >> focal;
    f >> q.w() >> q.x() >> q.y() >> q.z();
    f >> c[0] >> c[1] >> c[2] >> dist1 >> dist2;
    cid_to_filename->at(cid) = token;

    // Solve for t, which is part of the affine transform
    Eigen::Matrix3d r = q.matrix();
    cid_to_cam_t_global->at(cid).linear() = r;
    cid_to_cam_t_global->at(cid).translation() = -r * c;
  }

  // Read the number of points
  ptrdiff_t number_of_pid;
  f >> number_of_pid;
  if (number_of_pid < 1) {
    LOG(FATAL) << "The NVM file has no triangulated points.";
  }

  // Read the point
  pid_to_cid_fid->resize(number_of_pid);
  pid_to_xyz->resize(number_of_pid);
  Eigen::Vector3d xyz;
  Eigen::Vector3i color;
  Eigen::Vector2d pt;
  ptrdiff_t cid, fid;
  for (ptrdiff_t pid = 0; pid < number_of_pid; pid++) {
    pid_to_cid_fid->at(pid).clear();

    ptrdiff_t number_of_measures;
    f >> xyz[0] >> xyz[1] >> xyz[2] >>
      color[0] >> color[1] >> color[2] >> number_of_measures;
    pid_to_xyz->at(pid) = xyz;
    for (ptrdiff_t m = 0; m < number_of_measures; m++) {
      f >> cid >> fid >> pt[0] >> pt[1];

      pid_to_cid_fid->at(pid)[cid] = fid;

      if (cid_to_keypoint_map->at(cid).cols() <= fid) {
        cid_to_keypoint_map->at(cid).conservativeResize(Eigen::NoChange_t(), fid + 1);
      }
      cid_to_keypoint_map->at(cid).col(fid) = pt;
    }

    if (!f.good())
      LOG(FATAL) << "Unable to correctly read PID: " << pid;
  }
}

std::string sparse_mapping::ImageToFeatureFile(std::string const& image_file,
                                               std::string const& detector_name) {
  return std::string(image_file) + ".yaml.gz";
}

std::string sparse_mapping::DBImagesFile(std::string const& db_name) {
  return db_name + ".txt";
}

std::string sparse_mapping::MatchesFile(std::string const& map_file) {
  return map_file + ".matches.txt";
}

std::string sparse_mapping::EssentialFile(std::string const& map_file) {
  return map_file + ".essential.csv";
}

int sparse_mapping::ReadFeaturesSIFT(std::string const& filename,
                                     cv::Mat * descriptors,
                                     std::vector<cv::KeyPoint> * keypoints) {
  // Read SIFT keypoints and descriptors as written by Lowe's sift tool and opensift.

  std::ifstream f(filename);
  if (!f.good()) {
    LOG(ERROR) << "Could not read: " << filename;
    return 0;
  }

  std::string line;
  if (!std::getline(f, line)) {
    LOG(ERROR) << "Invalid file: " << filename;
    return 0;
  }

  int i, num, len;

  if (sscanf(line.c_str(), "%d %d", &num, &len) != 2) {
    LOG(ERROR) << "Invalid file: " << filename;
    return 0;
  }

  if (len != 128) {
    printf("Keypoint descriptor length invalid (should be 128).");
    return 0;
  }

  *descriptors = cv::Mat(num, 128, CV_32F);
  keypoints->resize(num);

  int16_t pbuf[128];
  for (i = 0; i < num; i++) {
    // Allocate memory for the keypoint.
    float x, y, scale, ori;

    if (!std::getline(f, line)) {
      LOG(ERROR) << "Invalid file: " << filename;
      return 0;
    }

    if (sscanf(line.c_str(), "%f %f %f %f\n", &y, &x, &scale, &ori) != 4) {
      printf("Invalid keypoint file format.");
      return 0;
    }

    (*keypoints)[i] = cv::KeyPoint(x, y, scale, ori);

    int16_t * p = pbuf;
    for (int iter = 0; iter < 7; iter++) {
      if (!std::getline(f, line)) {
        LOG(ERROR) << "Invalid file: " << filename;
        return 0;
      }

      if (iter < 6) {
        sscanf(line.c_str(),
               "%hu %hu %hu %hu %hu %hu %hu %hu %hu %hu "
               "%hu %hu %hu %hu %hu %hu %hu %hu %hu %hu",
               p+0, p+1, p+2, p+3, p+4, p+5, p+6, p+7, p+8, p+9,
               p+10, p+11, p+12, p+13, p+14,
               p+15, p+16, p+17, p+18, p+19);

        p += 20;
      } else {
        sscanf(line.c_str(),
               "%hu %hu %hu %hu %hu %hu %hu %hu",
               p+0, p+1, p+2, p+3, p+4, p+5, p+6, p+7);
        p += 8;
      }
    }

    for (int c = 0; c < 128; c++)
      (*descriptors).at<float>(i, c) = pbuf[c];
  }

  return num;
}

void sparse_mapping::WriteFeatures(std::string const& detector_name,
                                   std::vector<cv::KeyPoint> const& keypoints,
                                   cv::Mat const& descriptors,
                                   std::string const& output_filename) {
  LOG(INFO) << "Writing: " << output_filename;
  cv::FileStorage fs(output_filename,
                     cv::FileStorage::WRITE);
  cv::write(fs, "keypoints", keypoints);
  cv::write(fs, "descriptions", descriptors);
}

bool sparse_mapping::ReadFeatures(std::string const& input_filename,
                                  std::string const& detector_name,
                                  std::vector<cv::KeyPoint> * keypoints,
                                  cv::Mat * descriptors) {
  LOG(INFO) << "Reading: " << input_filename;

  // Test the file
  std::ifstream f(input_filename);
  if (!f.good()) {
    LOG(FATAL) << "Could not read: " << input_filename;
    return false;
  }

  // Read the yaml.gz file
  cv::FileStorage fs(input_filename, cv::FileStorage::READ);
  cv::FileNode fn = fs["keypoints"];
  cv::read(fn, *keypoints);
  fs["descriptions"] >> *descriptors;

  return true;
}

Eigen::Vector3d
sparse_mapping::TriangulatePoint(Eigen::Vector3d const& unnormalized_pt1,
                                 Eigen::Vector3d const& unnormalized_pt2,
                                 Eigen::Matrix3d const& cam2_r_cam1,
                                 Eigen::Vector3d const& cam2_t_cam1,
                                 double * error) {
  // The second camera's center in the coordinate system of the first
  // camera.
  Eigen::Vector3d p2 = -cam2_r_cam1.transpose() * cam2_t_cam1;

  // Calculate the two unit pointing vectors in the domain of cam1
  Eigen::Vector3d unit1 = unnormalized_pt1.normalized();
  Eigen::Vector3d unit2 = cam2_r_cam1.transpose() * unnormalized_pt2.normalized();

  Eigen::Vector3d v12 = unit1.cross(unit2);
  Eigen::Vector3d v1 = v12.cross(unit1);
  Eigen::Vector3d v2 = v12.cross(unit2);

  Eigen::Vector3d closestPoint1 = v2.dot(p2) / v2.dot(unit1) * unit1;
  Eigen::Vector3d closestPoint2 = p2 + v1.dot(-p2) / v1.dot(unit2) * unit2;
  *error = (closestPoint2 - closestPoint1).norm();

  return 0.5 * (closestPoint2 + closestPoint1);
}

void sparse_mapping::DecomposeFMatIntoEMat(Eigen::Matrix3d const& fundamental,
                                           Eigen::Matrix3d const& intrinsics,
                                           Eigen::Matrix3d * essential) {
  Eigen::Matrix<double, 3, 3> messy_essential =
    intrinsics.transpose() * fundamental * intrinsics;
  Eigen::JacobiSVD<Eigen::Matrix3d>
    svd(messy_essential, Eigen::ComputeFullU | Eigen::ComputeFullV);
  *essential =
    svd.matrixU() * Eigen::Vector3d(1, 1, 0).asDiagonal() * svd.matrixV().transpose();
}

void sparse_mapping::DecomposeEMatIntoRT(Eigen::Matrix3d const& essential,
                                         Eigen::Matrix2Xd const& unnormalized_pts1,
                                         Eigen::Matrix2Xd const& unnormalized_pts2,
                                         std::vector<cv::DMatch> const& matches,
                                         double focal_length1,  // Camera 1
                                         double focal_length2,  // Camera 2
                                         Eigen::Matrix3d * cam2_r_cam1,
                                         Eigen::Vector3d * cam2_t_cam1) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(essential,
                                        Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d w;
  w << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix3d R[2];
  Eigen::Vector3d t[2];
  R[0] = svd.matrixU() * w * svd.matrixV().transpose();
  R[1] = svd.matrixU() * w.transpose() * svd.matrixV().transpose();
  R[0] /= R[0].determinant();
  R[1] /= R[1].determinant();
  t[0] = svd.matrixU().col(2);
  t[1] = -t[0];

  // Test all possible combinations to determine which combination
  // is correct. (Or most correct in this case. Our fundamental
  // matrix that was fitted still has measurements with 5 px error.)
  int histogram[2][2] = {{0, 0}, {0, 0}};
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (std::vector<cv::DMatch>::value_type const& match : matches) {
        double error;

        Eigen::Vector2d px1 = unnormalized_pts1.col(match.queryIdx);
        Eigen::Vector2d px2 = unnormalized_pts2.col(match.trainIdx);

        Eigen::Vector3d pt =
          sparse_mapping::TriangulatePoint
          (Eigen::Vector3d(px1[0], px1[1], focal_length1),
           Eigen::Vector3d(px2[0], px2[1], focal_length2),
           R[i], t[j], &error);

        // Point described in the other camera's space
        Eigen::Vector3d pt_2 = R[i] * pt + t[j];

        if (pt[2] > 0 && pt_2[2] > 0) {
          // In front of the second camera. This must be a valid
          // solution.
          histogram[i][j]++;
        }
      }
    }
  }

  // Select the best solution for R and t
  int best_sol = std::max(std::max(histogram[0][0], histogram[0][1]),
                          std::max(histogram[1][0], histogram[1][1]));
  if (best_sol == histogram[0][0]) {
  } else if (best_sol == histogram[0][1]) {
    t[0] = t[1];
  } else if (best_sol == histogram[1][0]) {
    R[0] = R[1];
  } else if (best_sol == histogram[1][1]) {
    R[0] = R[1];
    t[0] = t[1];
  }

  // Write the output
  *cam2_r_cam1 = R[0];
  *cam2_t_cam1 = t[0];
}

std::string sparse_mapping::CvMatTypeStr(cv::Mat const& Mat) {
  int type = Mat.type();
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  std::string r;
  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void sparse_mapping::ListToListMap(std::vector<std::string> const& big_list,
                                   std::vector<std::string> const& small_list,
                                   std::map<int, int> * map) {
  // Given a big list, and a smaller subset of it, for each index i in
  // the small list find the index j in the big list so that
  // small_list[i] equals big_list[j].  Define the map as map[j] = i.
  (*map).clear();

  std::map<std::string, int> str2int;
  for (size_t i = 0; i < big_list.size(); i++)
    str2int[big_list[i]] = i;

  for (size_t i = 0; i < small_list.size(); i++) {
    std::map<std::string, int>::iterator it = str2int.find(small_list[i]);
    if (it == str2int.end())
      LOG(FATAL) << "Could not query image: " << small_list[i];

    (*map)[it->second] = i;
  }
}

void sparse_mapping::MergePids(int repeat_index, int num_unique,
                               std::vector<std::map<int, int> > * pid_to_cid_fid) {
  // Consider a set of images, and the corresponding tracks, stored in
  // (*pid_to_cid_fid). By design, we have num_unique images,
  // and after these, the images up to index repeat_index are repeated
  // (we do this repetition to help us close the loop). So, our sequence
  // is in fact,
  // image[0], ..., image[repeat_index], ..., image[num_unique-1], ...
  // image[0], ..., image[repeat_index].

  // So, some tracks show up twice (sometimes the second instance is
  // longer than the first, since it sees images with indices <=
  // num_unique - 1). Merge the repeated tracks, by replacing each cid
  // >=num_unique in the tracks with cid%num_unique, and wipe the now
  // redundant tracks.

  int num_to_wipe = 0;

  std::set<int> pids_to_wipe;
  for (int cid1 = 0; cid1 <= repeat_index; cid1++) {
    // Images cid1 and cid2 are the same, so in the final
    // merge tracks cid2 will become cid1
    int cid2 = cid1 + num_unique;

    // For given cid1 and cid2, find all pids containing these cids.
    // Index by fid, which is the same for both images.
    std::map<int, int> fid_to_pid1, fid_to_pid2;

    for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
      // Ignore already merged and redundant pids which we will wipe
      if (pids_to_wipe.find(pid) != pids_to_wipe.end())
        continue;

      std::map<int, int> & cid_fid = (*pid_to_cid_fid)[pid];

      std::map<int, int>::iterator it1 = cid_fid.find(cid1);
      if (it1 != cid_fid.end())  fid_to_pid1[it1->second] = pid;

      std::map<int, int>::iterator it2 = cid_fid.find(cid2);
      if (it2 != cid_fid.end())  fid_to_pid2[it2->second] = pid;
    }

    // Merge the tracks having the same fids
    for (std::map<int, int>::iterator fid_it1 = fid_to_pid1.begin();
         fid_it1 != fid_to_pid1.end(); fid_it1++) {
      std::map<int, int>::iterator it2 = fid_to_pid2.find(fid_it1->first);
      if (it2 == fid_to_pid2.end()) continue;

      // The indices of the pids to merge
      int pid1 = fid_it1->second, pid2 = it2->second;

      // The below is quite unlikely
      if (pid1 == pid2) continue;

      // Ignore already merged and redundant pids which we will wipe
      if (pids_to_wipe.find(pid1) != pids_to_wipe.end()) continue;
      if (pids_to_wipe.find(pid2) != pids_to_wipe.end()) continue;

      // Important, we are aliasing below
      std::map<int, int> & cid_fid1 = (*pid_to_cid_fid)[pid1];
      std::map<int, int> & cid_fid2 = (*pid_to_cid_fid)[pid2];

      for (std::map<int, int>::iterator it = cid_fid2.begin();
           it != cid_fid2.end() ; it++) {
        int good_cid = it->first;
        if (good_cid >= num_unique) good_cid = good_cid % num_unique;

        // Merge feature into first pid
        cid_fid1[good_cid] = it->second;
      }

      pids_to_wipe.insert(pid2);
      num_to_wipe++;
    }
  }

  // Any leftover cid >= num_unique must be shifted by num_unique
  for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
    // Ignore already merged and redundant pids which we will wipe
    if (pids_to_wipe.find(pid) != pids_to_wipe.end())
      continue;

    std::map<int, int> & cid_fid = (*pid_to_cid_fid)[pid];
    std::map<int, int>   cid_fid2;
    bool need_to_shift = false;
    for (std::map<int, int>::iterator it = cid_fid.begin();
         it != cid_fid.end() ; it++) {
      if (it->first >= num_unique) {
        need_to_shift = true;
      }

      int cid = it->first % num_unique;
      cid_fid2[cid] = it->second;
    }
    if (need_to_shift) {
      // Overwrite with the pid with the shifted cid
      (*pid_to_cid_fid)[pid] = cid_fid2;
    }
    // some pids only link to themselves, delete
    if ((*pid_to_cid_fid)[pid].size() < 2)
      pids_to_wipe.insert(pid);
  }

  // Wipe the pids which were merged
  std::vector<std::map<int, int> > pid_to_cid_fid2;
  for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
    if (pids_to_wipe.find(pid) != pids_to_wipe.end()) continue;
    pid_to_cid_fid2.push_back((*pid_to_cid_fid)[pid]);
  }
  LOG(INFO) << "Number of pids before and after loop closure: "
            << (*pid_to_cid_fid).size() << ' ' << pid_to_cid_fid2.size();
  (*pid_to_cid_fid) = pid_to_cid_fid2;

  LOG(INFO) << "Number of removed pids: " << num_to_wipe;

  // Sanity check, there must be no cids >= num_unique by now
  for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
    std::map<int, int> & cid_fid = (*pid_to_cid_fid)[pid];
    for (std::map<int, int>::iterator it = cid_fid.begin(); it != cid_fid.end(); it++) {
      if (it->first >= num_unique)
        LOG(FATAL) << "Must have fixed all cids by now.";
    }
  }
}

void sparse_mapping::PrintPidStats(std::vector<std::map<int, int> > const& pid_to_cid_fid) {
  std::map<int, int> cid_to_pid;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    std::map<int, int> const& cid_fid = pid_to_cid_fid[pid];
    for (std::map<int, int>::const_iterator it = cid_fid.begin(); it != cid_fid.end();
         it++) {
      cid_to_pid[it->first]++;
    }
  }
  LOG(INFO) << "cid and number of pids having fids in that cid";
  for (std::map<int, int>::iterator it = cid_to_pid.begin();
       it != cid_to_pid.end(); it++) {
    LOG(INFO) << "cid_fid " << it->first << ' ' << it->second;
  }
}

// Extract control points and the images they correspond 2 from
// a hugin project file
void sparse_mapping::ParseHuginControlPoints(std::string const& hugin_file,
                                             std::vector<std::string> * images,
                                             Eigen::MatrixXd * points) {
  // Initialize the outputs
  (*images).clear();
  *points = Eigen::MatrixXd(6, 1);

  std::ifstream hf(hugin_file.c_str());
  if (!hf.good())
    LOG(FATAL) << "ParseHuginControlPoints(): Could not open hugin file: " << hugin_file;

  int num_points = 0;
  std::string line;
  while (getline(hf, line)) {
    // Parse for images
    if (line.find("i ") == 0) {
      size_t it = line.find("n\"");
      if (it == std::string::npos)
        LOG(FATAL) << "ParseHuginControlPoints(): Invalid line: " << line;
      it += 2;
      std::string image;
      while (it < line.size() && line[it] != '"') {
        image += line[it];
        it++;
      }
      (*images).push_back(image);
    }

    // Parse control points
    if (line.find("c ") == 0) {
      // First wipe all letters
      std::string orig_line = line;
      char * ptr = const_cast<char*>(line.c_str());
      for (size_t i = 0; i < line.size(); i++) {
        // Wipe some extra chars
        if ( (ptr[i] >= 'a' && ptr[i] <= 'z') ||
             (ptr[i] >= 'A' && ptr[i] <= 'Z') )
          ptr[i] = ' ';
      }

      // Out of a line like:
      // c n0 N1 x367 y240 X144.183010710425 Y243.04008545843 t0
      // we store the numbers, 0, 1, 367, 240, 144.183010710425 243.04008545843
      // as a column.
      // The stand for left image index, right image index,
      // left image x, left image y, right image x, right image y.
      double a, b, c, d, e, f;
      if (sscanf(ptr, "%lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f) != 6)
        LOG(FATAL) << "ParseHuginControlPoints(): Could not scan line: " << line;

      // The left and right images must be different
      if (a == b)
        LOG(FATAL) << "The left and right images must be distinct. "
                   << "Offending line in " << hugin_file << " is:\n"
                   << orig_line << "\n";

      num_points++;
      (*points).conservativeResize(Eigen::NoChange_t(), num_points);
      (*points).col(num_points-1) << a, b, c, d, e, f;
    }
  }
}

namespace {
  // A little helper function
  bool is_blank(std::string const& line) {
    return (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos);
  }
}

// Parse a file having on each line xyz coordinates
void sparse_mapping::ParseXYZ(std::string const& xyz_file,
                              Eigen::MatrixXd * xyz) {
  // Initialize the outputs
  *xyz = Eigen::MatrixXd(3, 1);

  std::ifstream hf(xyz_file.c_str());
  if (!hf.good())
    LOG(FATAL) << "ParseXYZ(): Could not open hugin file: " << xyz_file;

  int num_points = 0;
  std::string line;
  while (getline(hf, line)) {
    // Ignore lines starting with comments and empty lines
    if (line.find("#") == 0 || is_blank(line)) continue;

    // Apparently sometimes empty lines show up as if of length 1
    if (line.size() == 1)
      continue;

    // Replace commas with spaces
    char * ptr = const_cast<char*>(line.c_str());
    for (size_t c = 0; c < line.size(); c++)
      if (ptr[c] == ',') ptr[c] = ' ';
    double x, y, z;
    if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
      LOG(FATAL) << "ParseXYZ(): Could not scan line: '" << line << "'\n";

    num_points++;
    (*xyz).conservativeResize(Eigen::NoChange_t(), num_points);
    (*xyz).col(num_points-1) << x, y, z;
  }
}

// Parse a CSV file, with the first line having column names. Return
// the results as columns in an std::map, with the column name being the
// key. We assume all values are numbers (non-numbers are set to 0).
void sparse_mapping::ParseCSV(std::string const& csv_file,
                              std::map< std::string, std::vector<double> > *cols) {
  // Initialize the output
  (*cols).clear();

  std::ifstream cf(csv_file.c_str());
  if (!cf.good())
    LOG(FATAL) << "ParseCSV(): Could not open file: " << csv_file;

  int line_pos = -1;
  std::vector<std::string> col_names;
  std::string line;
  while (getline(cf, line)) {
    line_pos++;

    // Replace leading % or # in the first line.
    // Ignore other lines starting with this field
    char * ptr = const_cast<char*>(line.c_str());
    if (line.find("#") == 0 || line.find("%") == 0) {
      if (line_pos == 0)
        ptr[0] = ' ';
      else
        continue;
    }

    // Skip empty lines
    if (is_blank(line)) continue;

    // Replace commas with spaces
    for (size_t c = 0; c < line.size(); c++)
      if (ptr[c] == ',') ptr[c] = ' ';

    // Parse the fields in the line
    std::istringstream is(line);
    std::string val;
    int col_pos = -1;
    while (is >> val) {
      col_pos++;

      // Initialize columns
      if (line_pos == 0) {
        col_names.push_back(val);
        (*cols)[val] = std::vector<double>();
        continue;
      }

      // Convert value to double (invalid values will be 0).
      double v = atof(val.c_str());
      if (col_pos < static_cast<int>(col_names.size()))
        (*cols)[col_names[col_pos]].push_back(v);
    }
  }

  return;
}

// save size before protbuf, to save multiple protobufs in one file
bool sparse_mapping::WriteProtobufTo(const google::protobuf::MessageLite& message,
                     google::protobuf::io::ZeroCopyOutputStream* rawOutput) {
  google::protobuf::io::CodedOutputStream output(rawOutput);
  // Write the size.
  int size = message.ByteSize();
  output.WriteVarint32(size);

  uint8_t* buffer = output.GetDirectBufferForNBytesAndAdvance(size);
  if (buffer != NULL) {
    // Optimization:  The message fits in one buffer, so use the faster
    // direct-to-array serialization path.
    message.SerializeWithCachedSizesToArray(buffer);
  } else {
    // Slightly-slower path when the message is multiple buffers.
    message.SerializeWithCachedSizes(&output);
    if (output.HadError())
      return false;
  }

  return true;
}

bool sparse_mapping::WriteFileTo(const char* filename,
                     google::protobuf::io::ZeroCopyOutputStream* rawOutput) {
  google::protobuf::io::CodedOutputStream output(rawOutput);

  FILE *f = fopen(filename, "r");
  if (f == NULL)
    return false;
  if (fseek(f, 0L, SEEK_END) != 0)
    return false;
  uint64_t size = ftell(f);
  rewind(f);

  // Write the size.
  output.WriteVarint32(size);

  char buffer[4096];
  uint64_t count = 0;
  while (true) {
    size_t len = fread(buffer, sizeof(char), 4096, f);
    count += len;
    output.WriteRaw(buffer, len);
    if (len != 4096)
      break;
  }
  fclose(f);
  if (count != size) {
    LOG(ERROR) << "Failed to write file to protobuf output stream.";
    return false;
  }

  return true;
}

// read size before protbuf, to save multiple protobufs in one file
bool sparse_mapping::ReadProtobufFrom(google::protobuf::io::ZeroCopyInputStream* rawInput,
                      google::protobuf::MessageLite* message) {
  google::protobuf::io::CodedInputStream input(rawInput);

  // Read the size.
  uint32_t size;
  if (!input.ReadVarint32(&size)) return false;

  // Tell the stream not to read beyond that size.
  auto limit = input.PushLimit(size);

  // Parse the message.
  if (!message->MergePartialFromCodedStream(&input)) return false;
  if (!input.ConsumedEntireMessage()) return false;

  // Release the limit.
  input.PopLimit(limit);

  return true;
}

// read size before protbuf, to save multiple protobufs in one file
bool sparse_mapping::ReadFileFrom(google::protobuf::io::ZeroCopyInputStream* rawInput, const char* filename) {
  FILE * f = fopen(filename, "w");
  if (f == NULL)
    return false;
  google::protobuf::io::CodedInputStream input(rawInput);

  // Read the size.
  uint32_t size;
  if (!input.ReadVarint32(&size)) return false;

  // Tell the stream not to read beyond that size.
  auto limit = input.PushLimit(size);

  char buffer[4096];
  unsigned int count = 0;
  while (count < size) {
    unsigned int bytes = std::min(4096u, size - count);
    if (!input.ReadRaw(buffer, bytes)) {
      LOG(ERROR) << "Failed to read file from protobuf.";
      fclose(f);
      return false;
    }
    if (fwrite(buffer, sizeof(char), bytes, f) != bytes) {
      LOG(ERROR) << "Failed to write file from protobuf.";
      fclose(f);
      return false;
    }
    count += bytes;
  }

  // Release the limit.
  input.PopLimit(limit);
  fclose(f);

  return true;
}


// Apply a given transform to the specified xyz points, and adjust accordingly the cameras
// for consistency. We assume that the transform is of the form
// A(x) = scale * rotation * x + translation
void sparse_mapping::TransformCamerasAndPoints(Eigen::Affine3d const& A,
                                               std::vector<Eigen::Affine3d> *cid_to_cam_t,
                                               std::vector<Eigen::Vector3d> *xyz) {
  for (size_t pid = 0; pid < (*xyz).size(); pid++)
    (*xyz)[pid] = A * (*xyz)[pid];

  // Inverse of rotation component
  double scale = pow(A.linear().determinant(), 1.0/3.0);
  Eigen::MatrixXd Ainv = (A.linear()/scale).inverse();

  for (size_t cid = 0; cid < (*cid_to_cam_t).size(); cid++) {
    (*cid_to_cam_t)[cid].linear() = (*cid_to_cam_t)[cid].linear()*Ainv;
    (*cid_to_cam_t)[cid].translation() = scale*(*cid_to_cam_t)[cid].translation() -
      (*cid_to_cam_t)[cid].linear()*A.translation();
  }
}

// Get the median error value, and multiply it by factor.
double sparse_mapping::GetErrThresh(std::vector<double> const& errors, double factor) {
  std::vector<double> sorted_errors = errors;
  std::sort(sorted_errors.begin(), sorted_errors.end());

  int len = sorted_errors.size();
  if (len == 0) return 0;

  // The case when there are too few errors
  if (len <= 2) return factor*sorted_errors[len-1];

    return factor*sorted_errors[len/2];
}

// Find the maximum angle between n rays intersecting at given
// point. Must compute the camera centers in the global coordinate
// system before calling this function.
double sparse_mapping::ComputeRaysAngle(int pid,
                                        std::vector<std::map<int, int> > const& pid_to_cid_fid,
                                        std::vector<Eigen::Vector3d> const & cam_ctrs,
                                        std::vector<Eigen::Vector3d> const& pid_to_xyz) {
  double max_angle = 0;
  std::map<int, int> const& track = pid_to_cid_fid[pid];
  for (std::map<int, int>::const_iterator it1 = track.begin();
       it1 != track.end(); it1++) {
    int cid1 = it1->first;
    for (std::map<int, int>::const_iterator it2 = it1;
         it2 != track.end(); it2++) {
      if (it1 == it2) continue;

      int cid2 = it2->first;
      Eigen::Vector3d X1 = cam_ctrs[cid1] - pid_to_xyz[pid];
      Eigen::Vector3d X2 = cam_ctrs[cid2] - pid_to_xyz[pid];
      double l1 = X1.norm(), l2 = X2.norm();
      if (l1 == 0 || l2 == 0)
        continue;

      double dot = X1.dot(X2)/l1/l2;
      dot = std::min(dot, 1.0);
      dot = std::max(-1.0, dot);
      double angle = (180.0/M_PI)*acos(dot);
      max_angle = std::max(angle, max_angle);
    }
  }
  return max_angle;
}

void sparse_mapping::FilterPID(double reproj_thresh,
                               camera::CameraParameters const& camera_params,
                               std::vector<Eigen::Affine3d > const& cid_to_cam_t_global,
                               std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                               std::vector<std::map<int, int> > * pid_to_cid_fid,
                               std::vector<Eigen::Vector3d> * pid_to_xyz,
                               bool print_stats, double multiple_of_median) {
  // Remove points that don't project at valid camera pixels,
  // points behind the camera, and matches having large reprojection error.

  // Reprojection error at each match point.
  std::vector<double> errors;

  int num_cams = cid_to_cam_t_global.size();
  std::vector<Eigen::Vector3d> cam_ctrs(num_cams);
  for (int cid = 0; cid < num_cams; cid++) {
    cam_ctrs[cid] = cid_to_cam_t_global[cid].inverse().translation();
  }

  // Init the stats
  sparse_mapping::FilterStats s;
  s.total = (*pid_to_xyz).size();

  std::vector<bool> is_bad((*pid_to_xyz).size(), false);
  Eigen::Vector2d half_size = camera_params.GetUndistortedHalfSize();
  for (size_t pid = 0; pid < (*pid_to_xyz).size(); pid++) {
    bool small_angle = false, behind_cam = false, invalid_reproj = false;

    double max_angle
      = sparse_mapping::ComputeRaysAngle(pid, *pid_to_cid_fid,
                                         cam_ctrs,  *pid_to_xyz);
    if (max_angle < FLAGS_min_valid_angle) {
      small_angle = true;
      is_bad[pid] = true;
    }

    for (std::pair<int, int> cid_fid : (*pid_to_cid_fid)[pid]) {
      Eigen::Vector2d pix = (cid_to_cam_t_global[cid_fid.first] *
                             (*pid_to_xyz)[pid]).hnormalized() * camera_params.GetFocalLength();
      errors.push_back((cid_to_keypoint_map[cid_fid.first].col(cid_fid.second) - pix).norm());
      // Mark points which don't project at valid camera pixels
      // TODO(zmoratto) : This can probably be done with a Eigen Array reduction
      if (pix[0] < -half_size[0] || pix[0] >= half_size[0] || pix[1] < -half_size[1] || pix[1] >= half_size[1]) {
        invalid_reproj = true;
        is_bad[pid] = true;
      }

      // Mark points that are behind the camera
      Eigen::Vector3d P = cid_to_cam_t_global[cid_fid.first] * (*pid_to_xyz)[pid];
      if (P[2] <= 0) {
        behind_cam = true;
        is_bad[pid] = true;
      }
    }
    s.small_angle    += static_cast<int>(small_angle);
    s.behind_cam     += static_cast<int>(behind_cam);
    s.invalid_reproj += static_cast<int>(invalid_reproj);
  }

  for (size_t pid = (*pid_to_xyz).size() - 1; pid < (*pid_to_xyz).size(); pid--) {
    if (is_bad[pid]) {
      std::vector<std::map<int, int> >::iterator cid_fid_it = (*pid_to_cid_fid).begin();
      std::vector<Eigen::Vector3d>::iterator xyz_it = (*pid_to_xyz).begin();
      std::advance(cid_fid_it, pid);
      std::advance(xyz_it, pid);
      (*pid_to_cid_fid).erase(cid_fid_it);
      (*pid_to_xyz).erase(xyz_it);
    }
  }

  // Wipe all features who are further than the reprojection of the
  // corresponding 3D point than given threshold.
  double thresh = std::max(GetErrThresh(errors, multiple_of_median), reproj_thresh);
  LOG(INFO) << "Filtering features with reprojection error higher than: "
            << thresh << " pixels";
  for (size_t pid = (*pid_to_xyz).size() - 1; pid < (*pid_to_xyz).size(); pid--) {
    std::map<int, int> & cid_fid = (*pid_to_cid_fid)[pid];
    std::map<int, int>::iterator itr = cid_fid.begin();
    while (itr != cid_fid.end()) {
      s.num_features++;
      Eigen::Vector2d pix = (cid_to_cam_t_global[itr->first] *
                             (*pid_to_xyz)[pid]).hnormalized() * camera_params.GetFocalLength();
      double err
        = (cid_to_keypoint_map[itr->first].col(itr->second) - pix).norm();

      if (err >= thresh) {
        std::map<int, int>::iterator toErase = itr;
        ++itr;
        cid_fid.erase(toErase);
        s.big_reproj_err++;
      } else {
        ++itr;
      }
    }

    // Wipe a 3D point altogether if it corresponds to less than 2 matches.
    int total = (*pid_to_cid_fid)[pid].size();
    if (total < 2) {
      std::vector<std::map<int, int> >::iterator cid_fid_it
        = (*pid_to_cid_fid).begin();
      std::vector<Eigen::Vector3d>::iterator xyz_it = (*pid_to_xyz).begin();
      std::advance(cid_fid_it, pid);
      std::advance(xyz_it, pid);
      (*pid_to_cid_fid).erase(cid_fid_it);
      (*pid_to_xyz).erase(xyz_it);
    }
  }

  if (print_stats)
    s.PrintStats();
}

// Write the BAL format.
bool sparse_mapping::WriteBAL(const std::string& filename,
                              camera::CameraParameters const& camera_params,
                              std::vector<std::map<int, int> > const& pid_to_cid_fid,
                              std::vector<Eigen::Vector3d> const& pid_to_xyz,
                              std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
                              std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map) {
  std::ofstream os;
  os.open(filename.c_str());
  os.precision(20);
  if (!os.is_open()) {
    LOG(ERROR) << "WriteBAL: cannot open the file.";
    return false;
  }

  LOG(INFO) << "Writing: " << filename << std::endl;

  // Write the number of camera poses and 3D points
  size_t nrObservations = 0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++)
    nrObservations += pid_to_cid_fid[pid].size();

  // Write observations
  os << cid_to_cam_t_global.size() << " " << pid_to_cid_fid.size() << " "
     << nrObservations << std::endl;
  os << std::endl;

  for (size_t pid = 0; pid < pid_to_xyz.size(); pid++) {
    std::map<int, int> const& track = pid_to_cid_fid[pid];

    for (std::map<int, int>::const_iterator it = track.begin(); it != track.end(); it++) {
      int cid = it->first;
      int fid = it->second;

      Eigen::Vector2d pt = cid_to_keypoint_map[cid].col(fid);
      os << cid /*camera id*/<< " " << pid /*point id*/<< " "
         << -pt[0] << ' ' << pt[1] << std::endl;  // need minus due to bal conventions
    }
  }
  os << std::endl;

  // One more place at which to deal with bal conversion. The camera plane
  // is on the negative z axis.
  Eigen::Matrix3d T; T << -1, 0, 0, 0, 1, 0, 0, 0, -1;

  // Write cameras
  double k1 = 0, k2 = 0;  // no distortion
  for (size_t i = 0; i < cid_to_cam_t_global.size(); i++) {
    Eigen::Vector3d vec;
    camera::RotationToRodrigues(T*(cid_to_cam_t_global.at(i).linear()),
                                &vec);

    os << vec.transpose() << std::endl;
    os << (T*(cid_to_cam_t_global.at(i).translation())).transpose() << std::endl;
    os << camera_params.GetFocalLength() << std::endl;
    os << k1 << std::endl;
    os << k2 << std::endl;
    os << std::endl;
  }

  // Write the points
  for (size_t j = 0; j < pid_to_xyz.size(); j++) {
    Eigen::Vector3d point = pid_to_xyz[j];
    os << point[0] << std::endl;
    os << point[1] << std::endl;
    os << point[2] << std::endl;
    os << std::endl;
  }

  os.close();
  return true;
}

// Given a data sequence having camera pose information for a set of
// timestamps, interpolate those poses at the timestamps given in
// out_time. We assume timestamps are always in increasing values.
void sparse_mapping::PoseInterpolation(std::vector<std::string> const& images,
                                       std::vector<double> const& out_time,
                                       std::map< std::string, std::vector<double> >
                                       const& data,
                                       std::vector<Eigen::Affine3d> * cid_to_cam_t,
                                       std::vector<std::string> * good_images) {
  if (images.size() != out_time.size())
    LOG(FATAL) << "Number of images is inconsistent with the number of "
               << "timestamps at which to find the camera poses";

  // Pull the timestamps and pose information from the CSV file.
  typedef std::vector<double> dvec;
  dvec const& time = data.find("field.header.stamp")->second;
  dvec const& posx = data.find("field.pose.position.x")->second;
  dvec const& posy = data.find("field.pose.position.y")->second;
  dvec const& posz = data.find("field.pose.position.z")->second;
  dvec const& qx   = data.find("field.pose.orientation.x")->second;
  dvec const& qy   = data.find("field.pose.orientation.y")->second;
  dvec const& qz   = data.find("field.pose.orientation.z")->second;
  dvec const& qw   = data.find("field.pose.orientation.w")->second;

  // Ensure all columns were parsed correctly.
  size_t n = time.size();
  if (n != posx.size() || n != posy.size() || n != posz.size() ||
      n != qx.size() || n != qy.size() || n != qz.size() || n != qw.size())
    LOG(FATAL) << "Could not parse all ground truth fields.\n";

  if (FLAGS_verbose_parsing) {
    // Useful debug code. Save the bot trajectory as measured,
    // before we interpolated it at times the images were aquired.
    std::string uninterp_traj = "uninterp_trajectory.txt";
    LOG(INFO) << "Writing: " << uninterp_traj << std::endl;
    std::ofstream ut(uninterp_traj.c_str());
    ut.precision(18);
    for (size_t i = 0; i < n; i++) {
      ut << posx[i] << ' ' << posy[i] << ' ' << posz[i] << std::endl;
    }
    ut.close();
  }

  int num_in = time.size();
  int num_out = out_time.size();
  (*cid_to_cam_t).reserve(num_out); (*cid_to_cam_t).clear();
  (*good_images).reserve(num_out);  (*good_images).clear();

  // Bracket the output time. We count on the fact that the arrays
  // time and time_out are both increasing.
  int prev_beg = -1, prev_end = -1;
  int pos = 0;
  for (int cid = 0; cid < num_out; cid++) {
    bool success = false;
    // Increase pos until time[pos] <= out_time[cid] <= time[pos+1]
    while (1) {
      if (pos + 1 >= num_in) break;  // Out of range

      if (time[pos] > out_time[cid]) {
        // No point in going further, the values in time from
        // now on will always be larger than out_time[cid].
        success = false;
        break;
      }

      if (time[pos] <= out_time[cid] && out_time[cid] <= time[pos+1]) {
        success = true;
        break;
      }
      pos++;
    }

    if (!success) {
      // Try to see if we can bracket out_time[cid]
      // between the next values of the time array.
      continue;
    }

    // Insist that each cid be bracketed by new pos and pos+1.
    // Otherwise we have the unfortunate situation that we use
    // the same very sparse pos values to interpolate successive
    // cids, when the measurements are very sparse, with the result
    // that the interpolated values are very inaccurate. We'd rather
    // not interpolate at all then.
    if (prev_beg != -1 && prev_end != -1) {
      if (prev_beg == pos || prev_end == pos+1) {
        success = false;
      }
    }
    prev_beg = pos;
    prev_end = pos+1;
    if (!success) continue;

    double t = 0.0;
    if (time[pos] != time[pos+1])
      t = (out_time[cid] - time[pos])/(time[pos+1] - time[pos]);
    Eigen::Vector3d va(posx[pos],   posy[pos],   posz[pos]);
    Eigen::Vector3d vb(posx[pos+1], posy[pos+1], posz[pos+1]);
    Eigen::Quaternion<double> qa(qw[pos], qx[pos], qy[pos], qz[pos]);
    Eigen::Quaternion<double> qb(qw[pos+1], qx[pos+1], qy[pos+1], qz[pos+1]);

    Eigen::Affine3d T;
    T.translation() = (1.0-t)*va + t*vb;
    T.linear() = qa.slerp(t, qb).toRotationMatrix();
    (*cid_to_cam_t).push_back(T);
    (*good_images).push_back(images[cid]);
  }

  if (FLAGS_verbose_parsing) {
    // Useful debug code. Save the camera positions after interpolating
    // at image timestamps.
    std::string interp_traj = "interp_trajectory.txt";
    LOG(INFO) << "Writing: " << interp_traj << std::endl;
    std::ofstream it(interp_traj.c_str());
    it.precision(18);
    for (size_t i = 0; i < (*cid_to_cam_t).size(); i++) {
      it << (*cid_to_cam_t)[i].translation().transpose() << std::endl;
    }
    it.close();
  }

  return;
}
