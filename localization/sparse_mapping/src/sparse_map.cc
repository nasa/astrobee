/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http:  //  www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <sparse_mapping/sparse_map.h>
#include <camera/camera_params.h>
#include <ff_common/thread.h>
#include <ff_common/utils.h>
#include <interest_point/matching.h>
#include <sparse_mapping/reprojection.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/tensor.h>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>

#include <sparse_map.pb.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>

#include<boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <fstream>
#include <queue>
#include <set>
#include <thread>
#include <limits>

DEFINE_int32(num_similar, 20,
             "Use in localization this many images which "
             "are most similar to the image to localize.");
DEFINE_int32(num_ransac_iterations, 1000,
             "Use in localization this many ransac iterations.");
DEFINE_int32(ransac_inlier_tolerance, 3,
             "Use in localization this inlier tolerance.");
DEFINE_int32(early_break_landmarks, 100,
             "Break early when we have this many landmarks during localization.");
DEFINE_bool(histogram_equalization, false,
            "If true, equalize the histogram for images to improve robustness to illumination conditions.");
DEFINE_int32(num_extra_localization_db_images, 0,
             "Match this many extra images from the Vocab DB, only keep num_similar.");
DEFINE_bool(verbose_localization, false,
            "If true, list the images most similar to the one being localized.");

namespace sparse_mapping {

SparseMap::SparseMap(const std::vector<std::string> & filenames,
                     const std::string & detector,
                     const camera::CameraParameters & params):
        cid_to_filename_(filenames),
        detector_(detector), camera_params_(params),
        num_similar_(FLAGS_num_similar),
        num_ransac_iterations_(FLAGS_num_ransac_iterations),
        ransac_inlier_tolerance_(FLAGS_ransac_inlier_tolerance),
        early_break_landmarks_(FLAGS_early_break_landmarks),
        histogram_equalization_(FLAGS_histogram_equalization) {
  cid_to_descriptor_map_.resize(cid_to_filename_.size());
  // TODO(bcoltin): only record scale and orientation for opensift?
  cid_to_keypoint_map_.resize(cid_to_filename_.size());
}

SparseMap::SparseMap(const std::string & protobuf_file, bool localization) :
  camera_params_(Eigen::Vector2i(-1, -1), Eigen::Vector2d::Constant(-1),
                 Eigen::Vector2d(-1, -1)),
  num_similar_(FLAGS_num_similar),
  num_ransac_iterations_(FLAGS_num_ransac_iterations),
  ransac_inlier_tolerance_(FLAGS_ransac_inlier_tolerance),
  early_break_landmarks_(FLAGS_early_break_landmarks),
  histogram_equalization_(FLAGS_histogram_equalization) {
  // The above camera params used bad values because we are expected to reload
  // later.
  Load(protobuf_file, localization);
}

// Form a sparse map with given cameras/images, and no features
SparseMap::SparseMap(const std::vector<Eigen::Affine3d>& cid_to_cam_t,
                     const std::vector<std::string> & filenames,
                     const std::string & detector,
                     const camera::CameraParameters & params):
  detector_(detector), camera_params_(params),
  num_similar_(FLAGS_num_similar),
  num_ransac_iterations_(FLAGS_num_ransac_iterations),
  ransac_inlier_tolerance_(FLAGS_ransac_inlier_tolerance),
  early_break_landmarks_(FLAGS_early_break_landmarks),
  histogram_equalization_(FLAGS_histogram_equalization) {
  if (filenames.size() != cid_to_cam_t.size())
    LOG(FATAL) << "Expecting as many images as cameras";

  // Don't include images for which we have no camera information
  for (size_t cid = 0; cid < cid_to_cam_t.size(); cid++) {
    if (cid_to_cam_t[cid].linear() == Eigen::Matrix3d::Zero())
      continue;
    cid_to_cam_t_global_.push_back(cid_to_cam_t[cid]);
    cid_to_filename_.push_back(filenames[cid]);
  }

  int num_cams = cid_to_filename_.size();

  // Initialize other data expected in the map
  cid_to_keypoint_map_.resize(num_cams);
  cid_to_descriptor_map_.resize(num_cams);
}

// Form a sparse map by reading a text file from disk. This is for comparing
// bundler, nvm or theia maps.
SparseMap::SparseMap(bool bundler_format, std::string const& filename, std::vector<std::string> const& all_image_files)
    : camera_params_(Eigen::Vector2i(640, 480), Eigen::Vector2d::Constant(300),
                     Eigen::Vector2d(320, 240)),  // these are placeholders and must be changed
      num_similar_(FLAGS_num_similar),
      num_ransac_iterations_(FLAGS_num_ransac_iterations),
      ransac_inlier_tolerance_(FLAGS_ransac_inlier_tolerance),
      early_break_landmarks_(FLAGS_early_break_landmarks),
      histogram_equalization_(FLAGS_histogram_equalization) {
  std::string ext = ff_common::file_extension(filename);
  boost::to_lower(ext);

  if (ext == "nvm") {
    std::cout << "NVM format detected." << std::endl;

    sparse_mapping::ReadNVM(filename, &cid_to_keypoint_map_, &cid_to_filename_, &pid_to_cid_fid_, &pid_to_xyz_,
                            &cid_to_cam_t_global_);

    // Descriptors are not saved, so let them be empty
    cid_to_descriptor_map_.resize(cid_to_keypoint_map_.size());

    // When the NVM file is created by Theia, it saves the images
    // without a path, in random order, and it may not have used up
    // all the images, so need to adjust for that.

    std::map<std::string, std::string> base_to_full_path;
    std::map<int, std::string> orig_order;
    for (size_t it = 0; it < all_image_files.size(); it++) {
      std::string image = all_image_files[it];
      std::string base = boost::filesystem::path(image).filename().string();
      if (base_to_full_path.find(base) != base_to_full_path.end())
        LOG(FATAL) << "Duplicate image: " << base << std::endl;
      base_to_full_path[base] = image;
      orig_order[it] = base;
    }

    // Find the permutation which will tell how to reorder the images
    // in the nvm file to be in the original order.  This must happen
    // before we change cid_to_filename_ below.
    std::map<int, int> old_cid_to_new_cid;
    std::map<std::string, int> base2cid;
    for (size_t it = 0; it < cid_to_filename_.size(); it++) base2cid[cid_to_filename_[it]] = it;
    int new_cid = 0;
    for (auto order_it = orig_order.begin(); order_it != orig_order.end() ; order_it++) {
      auto base_it = base2cid.find(order_it->second);
      if (base_it == base2cid.end()) continue;  // Not all input images may be present in the map

      int old_cid = base_it->second;
      old_cid_to_new_cid[old_cid] = new_cid;
      new_cid++;
    }

    // Map the theia images to the actual image paths
    for (size_t it = 0; it < cid_to_filename_.size(); it++) {
      std::string base = cid_to_filename_[it];
      auto map_it = base_to_full_path.find(base);
      if (map_it == base_to_full_path.end())
        LOG(FATAL) << "The input file list is missing the nvm map image: " << base << std::endl;
      cid_to_filename_[it] = map_it->second;
    }

    // Apply the permutation
    reorderMap(old_cid_to_new_cid);

  } else if (bundler_format) {
    std::cout << "Bundler format detected." << std::endl;

    int num_cams = 0;

    std::ifstream is(filename.c_str());
    std::string line;
    std::getline(is, line);  // empty line
    is >> num_cams;
    cid_to_filename_ = all_image_files;
    cid_to_cam_t_global_.resize(num_cams);
    cid_to_filename_.resize(num_cams);

    for (int i = 0; i < num_cams; i++) {
      std::string line;
      std::getline(is, line);  // empty line
      std::getline(is, line);  // focal length, etc.

      // Rotation
      Eigen::Matrix3d T;
      for (int row = 0; row < T.rows(); row++) {
        for (int col = 0; col < T.cols(); col++) {
          is >> T(row, col);
        }
      }
      // This is needed for when bundler fails
      if (T.determinant() < 1e-8)
        T = Eigen::Matrix3d::Identity();

      // Translation
      Eigen::Vector3d P;
      for (int row = 0; row < P.size(); row++)
        is >> P[row];

      cid_to_cam_t_global_[i].linear() = T;  // not sure
      cid_to_cam_t_global_[i].translation() = P;
    }

    // Initialize other data expected in the map
    cid_to_keypoint_map_.resize(num_cams);
    cid_to_descriptor_map_.resize(num_cams);
  }

  // Initialize this convenient mapping
  InitializeCidFidToPid();
}

// Detect features in given images
void SparseMap::DetectFeatures() {
  ff_common::ThreadPool pool;
  bool multithreaded = true;
  size_t num_files = cid_to_filename_.size();
  for (size_t cid = 0; cid < num_files; cid++) {
    ff_common::PrintProgressBar(stdout, static_cast<float>(cid) / static_cast<float>(num_files - 1));

    pool.AddTask(&SparseMap::DetectFeaturesFromFile, this,
                 std::ref(cid_to_filename_[cid]),
                 multithreaded,
                 &cid_to_descriptor_map_[cid],
                 &cid_to_keypoint_map_[cid]);
  }
  pool.Join();

  // Create temporary pid_to_cid_fid_, it will contain all the raw
  // features we found so far, without matches (matching and outlier
  // removal will later reduce the number of features, so this is
  // useful for comparison).
  pid_to_cid_fid_.clear();
  for (size_t cid = 0; cid < cid_to_filename_.size(); cid++) {
    for (int fid = 0; fid < cid_to_keypoint_map_[cid].cols(); fid++) {
      std::map<int, int> cid_fid;
      cid_fid[cid] = fid;
      pid_to_cid_fid_.push_back(cid_fid);
    }
  }
  // Allocate space for landmarks
  pid_to_xyz_.resize(pid_to_cid_fid_.size());

  InitializeCidFidToPid();
}

void SparseMap::Load(const std::string & protobuf_file, bool localization) {
  sparse_mapping_protobuf::Map map;
  int input_fd = open(protobuf_file.c_str(), O_RDONLY);
  if (input_fd < 0)
    LOG(FATAL) << "Failed to open map file: " << protobuf_file;

  google::protobuf::io::ZeroCopyInputStream* input =
    new google::protobuf::io::FileInputStream(input_fd);
  if (!ReadProtobufFrom(input, &map)) {
    LOG(FATAL) << "Failed to parse map file.";
  }

  detector_.Reset(map.detector_name());

  // Check that the maps is correctly formed
  assert(map.camera().focal_length_size() == 2);
  assert(map.camera().optical_offset_size() == 2);
  assert(map.camera().distorted_image_size_size() == 2);
  assert(map.camera().undistorted_image_size_size() == 2);
  typedef Eigen::Vector2d V2d;
  typedef Eigen::Vector2i V2i;
  camera_params_.SetFocalLength(V2d(map.camera().focal_length(0),
                                    map.camera().focal_length(1)));
  camera_params_.SetOpticalOffset(V2d(map.camera().optical_offset(0),
                                      map.camera().optical_offset(1)));
  camera_params_.SetDistortedSize(V2i(map.camera().distorted_image_size(0),
                                      map.camera().distorted_image_size(1)));
  camera_params_.SetUndistortedSize(V2i(map.camera().undistorted_image_size(0),
                                        map.camera().undistorted_image_size(1)));
  Eigen::VectorXd distortion(map.camera().distortion_size());
  for (int i = 0; i < map.camera().distortion_size(); i++) {
    distortion[i] = map.camera().distortion(i);
  }
  camera_params_.SetDistortion(distortion);

  int num_frames = map.num_frames();
  int num_landmarks = map.num_landmarks();

  cid_to_filename_.resize(num_frames);
  cid_to_descriptor_map_.resize(num_frames);
  if (!localization) {
    cid_to_keypoint_map_.resize(num_frames);
    cid_to_cam_t_global_.resize(num_frames);
  }

  // load each frame
  for (int cid = 0; cid < num_frames; cid++) {
    sparse_mapping_protobuf::Frame frame;
    if (!ReadProtobufFrom(input, &frame)) {
      LOG(FATAL) << "Failed to parse frame.";
    }
    if (frame.has_name())
      cid_to_filename_[cid] = frame.name();
    else
      cid_to_filename_[cid] = "";

    // load keypoints
    if (!localization)
      cid_to_keypoint_map_[cid].resize(Eigen::NoChange_t(), frame.feature_size());

    // Poke the first frame's first descriptor to see how long the
    // descriptor is.
    if (frame.feature_size()) {
      size_t descriptor_length = frame.feature(0).description().size() /
        cv::getElemSize(map.descriptor_depth());
      cid_to_descriptor_map_[cid].create(frame.feature_size(),  // rows
                                         descriptor_length,     // columns
                                         map.descriptor_depth());
    } else {
      cid_to_descriptor_map_[cid].create(0, 0, map.descriptor_depth());
    }

    for (int fid = 0; fid < frame.feature_size(); fid++) {
      sparse_mapping_protobuf::Feature feature = frame.feature(fid);

      // Copy the features
      if (!localization)
        cid_to_keypoint_map_[cid].col(fid) << feature.x(), feature.y();

      // Copy the descriptors
      memcpy(cid_to_descriptor_map_[cid].ptr<uint8_t>(fid),  // Destination
             feature.description().data(),                   // Source
             feature.description().size());                  // Length
    }

    // Load pose
    if (frame.has_pose() && !localization) {
      sparse_mapping_protobuf::Affine3d pose = frame.pose();
      cid_to_cam_t_global_[cid].translation()
        << pose.t0(), pose.t1(), pose.t2();

      cid_to_cam_t_global_[cid].linear() <<
        pose.r00(), pose.r01(), pose.r02(),
        pose.r10(), pose.r11(), pose.r12(),
        pose.r20(), pose.r21(), pose.r22();
    }
  }

  // if not, only feature detection step has been run... or something is wrong
  if (num_landmarks > 0) {
    pid_to_xyz_.resize(num_landmarks);

    if (!localization) {
      pid_to_cid_fid_.resize(num_landmarks);
    } else {
      // Create directly cid_fid_to_pid
      cid_fid_to_pid_.clear();
      cid_fid_to_pid_.resize(cid_to_filename_.size(), std::map<int, int>());
    }

    for (int i = 0; i < num_landmarks; i++) {
      sparse_mapping_protobuf::Landmark l;
      if (!ReadProtobufFrom(input, &l)) {
        LOG(FATAL) << "Failed to parse landmark.";
      }
      Eigen::Vector3d pos(l.loc().x(), l.loc().y(), l.loc().z());
      pid_to_xyz_[i] = pos;
      for (int j = 0; j < l.match_size(); j++) {
        sparse_mapping_protobuf::Matching m = l.match(j);
        if (!localization)
          pid_to_cid_fid_[i][m.camera_id()] = m.feature_id();
        else
          cid_fid_to_pid_[m.camera_id()][m.feature_id()] = i;
      }
    }

    // If in localization mode, we already initialized cid_fid_to_pid_ right above.
    if (!localization)
      InitializeCidFidToPid();

  } else {
    LOG(WARNING) << "There appear to be no landmarks in map file.";
  }

  if (map.has_vocab_db())
    vocab_db_.LoadProtobuf(input, map.vocab_db());

  histogram_equalization_ = map.histogram_equalization();

  assert(histogram_equalization_ == 0 ||
         histogram_equalization_ == 1 ||
         histogram_equalization_ == 2);

  // For backward compatibility with old maps, allow a map to have its
  // histogram_equalization flag unspecified, but it is best to avoid
  // that situation, and rebuild the map if necessary.
  if (histogram_equalization_ == 2)
    std::cout << "Warning: Unknown value of histogram_equalization! "
              << "It is strongly suggested to rebuild this map to avoid "
              << "poor quality results." << std::endl;

  delete input;
  close(input_fd);
}

void SparseMap::SetDetectorParams(int min_features, int max_features, int retries,
                                  double min_thresh, double default_thresh, double max_thresh) {
  mutex_detector_.lock();
  detector_.Reset(detector_.GetDetectorName(), min_features, max_features, retries,
                  min_thresh, default_thresh, max_thresh);
  mutex_detector_.unlock();
}

void SparseMap::Save(const std::string & protobuf_file) const {
  // For backward compatibility with old maps, allow a map to have its
  // histogram_equalization flag unspecified, but it is best to avoid
  // that situation, and rebuild the map if necessary.
  if (histogram_equalization_ == 2)
    std::cout << "Warning: Unknown value of histogram_equalization! "
              << "It is strongly suggested to rebuild this map to avoid "
              << "poor quality results." << std::endl;

  sparse_mapping_protobuf::Map map;
  map.set_detector_name(detector_.GetDetectorName());
  if (!cid_to_descriptor_map_.empty())
    map.set_descriptor_depth(cid_to_descriptor_map_[0].depth());
  else
    map.set_descriptor_depth(0);

  sparse_mapping_protobuf::CameraModel* camera = map.mutable_camera();
  camera->add_focal_length(camera_params_.GetFocalVector()[0]);
  camera->add_focal_length(camera_params_.GetFocalVector()[1]);
  camera->add_optical_offset(camera_params_.GetOpticalOffset()[0]);
  camera->add_optical_offset(camera_params_.GetOpticalOffset()[1]);
  camera->add_distorted_image_size(camera_params_.GetDistortedSize()[0]);
  camera->add_distorted_image_size(camera_params_.GetDistortedSize()[1]);
  camera->add_undistorted_image_size(camera_params_.GetUndistortedSize()[0]);
  camera->add_undistorted_image_size(camera_params_.GetUndistortedSize()[1]);
  for (int i = 0; i < camera_params_.GetDistortion().size(); i++) {
    camera->add_distortion(camera_params_.GetDistortion()[i]);
  }

  CHECK(cid_to_filename_.size() == cid_to_keypoint_map_.size())
    << "Number of CIDs in filenames and keypoint map do not match";
  CHECK(cid_to_filename_.size() == cid_to_descriptor_map_.size())
    << "Number of CIDs in filenames and descriptor map do not match";

  map.set_num_frames(cid_to_filename_.size());
  map.set_num_landmarks(pid_to_xyz_.size());

  if (vocab_db_.binary_db != NULL)
    map.set_vocab_db(sparse_mapping_protobuf::Map::BINARYDB);

  map.set_histogram_equalization(histogram_equalization_);

  LOG(INFO) << "Writing: " << protobuf_file;
  int output_fd = open(protobuf_file.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (output_fd < 0) {
    LOG(FATAL) << "Failed to open protobuf writing file.";
  }
  google::protobuf::io::ZeroCopyOutputStream* output = new google::protobuf::io::FileOutputStream(output_fd);
  if (!WriteProtobufTo(map, output)) {
    LOG(FATAL) << "Failed to write map to file.";
  }

  // write the frames
  for (size_t cid = 0; cid < cid_to_filename_.size(); cid++) {
    sparse_mapping_protobuf::Frame frame;

    // set the filename if existing
    if (!cid_to_filename_[cid].empty()) {
      frame.set_name(cid_to_filename_[cid]);
    }

    // set the features, required
    for (int fid = 0; fid < cid_to_keypoint_map_[cid].cols(); fid++) {
      sparse_mapping_protobuf::Feature* f = frame.add_feature();
      f->set_x(cid_to_keypoint_map_[cid].col(fid).x());
      f->set_y(cid_to_keypoint_map_[cid].col(fid).y());
      f->set_description(cid_to_descriptor_map_[cid].ptr<uint8_t>(fid),
                         cid_to_descriptor_map_[cid].elemSize() *
                         cid_to_descriptor_map_[cid].cols);
    }

    // set the camera pose if available.
    if (cid < cid_to_cam_t_global_.size()) {
      sparse_mapping_protobuf::Affine3d* a = frame.mutable_pose();
      Eigen::Matrix4d c = cid_to_cam_t_global_[cid].matrix();
      a->set_r00(c(0, 0));
      a->set_r01(c(0, 1));
      a->set_r02(c(0, 2));
      a->set_r10(c(1, 0));
      a->set_r11(c(1, 1));
      a->set_r12(c(1, 2));
      a->set_r20(c(2, 0));
      a->set_r21(c(2, 1));
      a->set_r22(c(2, 2));
      a->set_t0(c(0, 3));
      a->set_t1(c(1, 3));
      a->set_t2(c(2, 3));
    }
    if (!WriteProtobufTo(frame, output)) {
      LOG(FATAL) << "Failed to write frame to file.";
    }
  }

  if (pid_to_xyz_.size() != pid_to_cid_fid_.size()) {
    LOG(FATAL) << "Book-keeping failure, expecting the following "
               << "arrays to have the same size:\n"
               << "pid_to_xyz_.size() = " << pid_to_xyz_.size() << "\n"
               << "pid_to_cid_fid_.size() = " << pid_to_cid_fid_.size();
  }

  for (size_t i = 0; i < pid_to_xyz_.size(); i++) {
    sparse_mapping_protobuf::Landmark l;
    l.mutable_loc()->set_x(pid_to_xyz_[i].x());
    l.mutable_loc()->set_y(pid_to_xyz_[i].y());
    l.mutable_loc()->set_z(pid_to_xyz_[i].z());
    for (std::map<int, int >::const_iterator it =
          pid_to_cid_fid_[i].begin(); it != pid_to_cid_fid_[i].end(); it++) {
      sparse_mapping_protobuf::Matching* m = l.add_match();
      m->set_camera_id(it->first);
      m->set_feature_id(it->second);
    }

    if (!WriteProtobufTo(l, output))
      LOG(FATAL) << "Failed to write landmark to file.";
  }

  if (vocab_db_.binary_db != NULL)
    vocab_db_.SaveProtobuf(output);

  delete output;
  close(output_fd);
}


// Non-member InitializeCidFidToPid() function, useful
// without a fully-formed map.
// From pid_to_cid_fid, create cid_fid_to_pid for lookup.
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid) {
  cid_fid_to_pid->clear();
  cid_fid_to_pid->resize(num_cid, std::map<int, int>());

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (std::pair<int, int> const& cid_fid : pid_to_cid_fid[pid]) {
      (*cid_fid_to_pid)[cid_fid.first][cid_fid.second] = pid;
    }
  }
}

void SparseMap::InitializeCidFidToPid() {
  sparse_mapping::InitializeCidFidToPid(cid_to_filename_.size(),
                                        pid_to_cid_fid_,
                                        &cid_fid_to_pid_);
}

void SparseMap::DetectFeaturesFromFile(std::string const& filename,
                                       bool multithreaded,
                                       cv::Mat* descriptors,
                                       Eigen::Matrix2Xd* keypoints) {
  cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  if (image.rows == 0 || image.cols == 0)
    LOG(FATAL) << "Found empty image in file: " << filename;

  DetectFeatures(image, multithreaded, descriptors, keypoints);
}

void SparseMap::DetectFeatures(const cv::Mat& image,
                               bool multithreaded,
                               cv::Mat* descriptors,
                               Eigen::Matrix2Xd* keypoints) {
  // If using histogram equalization, need an extra image to store it
  cv::Mat * image_ptr = const_cast<cv::Mat*>(&image);
  cv::Mat hist_image;
  if (histogram_equalization_) {
    cv::equalizeHist(image, hist_image);
    image_ptr = &hist_image;
  }

#if 0
  // This is useful for debugging
  std::cout << "Histogram equalization is " << histogram_equalization_ << std::endl;
  static int count = 10000;
  count++;
  std::ostringstream oss;
  oss << "image_" << count << ".jpg";
  std::string image_file = oss.str();
  std::cout << "Writing: " << image_file << std::endl;
  cv::imwrite(image_file, *image_ptr);
#endif

  std::vector<cv::KeyPoint> storage;
  mutex_detector_.lock();
  if (!multithreaded) {
    detector_.Detect(*image_ptr, &storage, descriptors);
  } else {
    // When using multiple threads, need an individual detector
    // instance, to avoid a crash. This is being used only in
    // map-building, rather than in localization which is more
    // performance-sensitive.
    int min_features, max_features, max_retries;
    double min_thresh, default_thresh, max_thresh;
    detector_.GetDetectorParams(min_features, max_features, max_retries,
                                min_thresh, default_thresh, max_thresh);
    interest_point::FeatureDetector local_detector(detector_.GetDetectorName(),
                                                   min_features, max_features, max_retries,
                                                   min_thresh, default_thresh, max_thresh);
    local_detector.Detect(*image_ptr, &storage, descriptors);
  }
  mutex_detector_.unlock();

  if (FLAGS_verbose_localization)
    std::cout << "Features detected " << storage.size() << std::endl;

  keypoints->resize(2, storage.size());
  Eigen::Vector2d output;

  for (size_t j = 0; j < storage.size(); j++) {
    camera_params_.Convert<camera::DISTORTED_C, camera::UNDISTORTED_C>
      (Eigen::Vector2d(storage[j].pt.x, storage[j].pt.y), &output);
    keypoints->col(j) = output;
  }
}

// A non-member Localize() function that can be invoked for a non-fully
// formed map.
bool Localize(cv::Mat const& test_descriptors,
              Eigen::Matrix2Xd const& test_keypoints,
              camera::CameraParameters const& camera_params,
              camera::CameraModel* pose,
              std::vector<Eigen::Vector3d>* inlier_landmarks,
              std::vector<Eigen::Vector2d>* inlier_observations,
              int num_cid,
              std::string const& detector_name,
              sparse_mapping::VocabDB * vocab_db,
              int num_similar,
              std::vector<std::string> const& cid_to_filename,
              std::vector<cv::Mat> const& cid_to_descriptor_map,
              std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
              std::vector<std::map<int, int> > const& cid_fid_to_pid,
              std::vector<Eigen::Vector3d> const& pid_to_xyz,
              int num_ransac_iterations, int ransac_inlier_tolerance,
              int early_break_landmarks, int histogram_equalization,
              std::vector<int> * cid_list) {
  std::vector<int> indices;
  // Query the vocab tree.
  if (cid_list == NULL)
    sparse_mapping::QueryDB(detector_name,
                            vocab_db,
                            // Notice that we request more similar
                            // images than what we need. We'll prune
                            // them below.
                            num_similar + FLAGS_num_extra_localization_db_images,
                            test_descriptors,
                            &indices);
  else
    indices = *cid_list;
  if (indices.empty()) {
    LOG(WARNING) << "Localizing against all keyframes as the vocab database is missing.";
    // Use all images, as no tree is available.
    for (int cid = 0; cid < num_cid; cid++)
      indices.push_back(cid);
  }

  // To turn on verbose localization for debugging
  // google::SetCommandLineOption("verbose_localization", "true");

  // Find matches to each image in map. Do this in two passes. First,
  // find matches to all map images, then keep only num_similar
  // best matched images, then localize against those.

  // We will not localize using all images having matches, there are too
  // many false positives that way. Instead, limit ourselves to the images
  // which have most observations in common with the current one.
  std::vector<int> similarity_rank(indices.size(), 0);
  std::vector<std::vector<cv::DMatch> > all_matches(indices.size());
  int total = 0;
  // TODO(oalexan1): Use multiple threads here?
  for (size_t i = 0; i < indices.size(); i++) {
    int cid = indices[i];
    interest_point::FindMatches(test_descriptors,
                                cid_to_descriptor_map[cid],
                                &all_matches[i]);

    for (size_t j = 0; j < all_matches[i].size(); j++) {
      if (cid_fid_to_pid[cid].count(all_matches[i][j].trainIdx) == 0)
        continue;
      similarity_rank[i]++;
    }
    if (FLAGS_verbose_localization)
      std::cout << "Overall matches and validated matches to: "
                << cid_to_filename[cid] << ": "
                << all_matches[i].size() << " "
                << similarity_rank[i] << "\n";
    total += similarity_rank[i];
    if (total >= early_break_landmarks)
      break;
  }

  std::vector<Eigen::Vector2d> observations;
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<int> highly_ranked = ff_common::rv_order(similarity_rank);
  int end = std::min(static_cast<int>(highly_ranked.size()), num_similar);
  std::set<int> seen_landmarks;
  if (FLAGS_verbose_localization)
    std::cout << "Similar images: ";
  for (int i = 0; i < end; i++) {
    int cid = indices[highly_ranked[i]];
    std::vector<cv::DMatch>* matches = &all_matches[highly_ranked[i]];
    int num_matches = 0;
    for (size_t j = 0; j < matches->size(); j++) {
      if (cid_fid_to_pid[cid].count(matches->at(j).trainIdx) == 0)
        continue;
      const int landmark_id = cid_fid_to_pid.at(cid).at(matches->at(j).trainIdx);
      if (seen_landmarks.count(landmark_id) > 0)
        continue;
      Eigen::Vector2d obs(test_keypoints.col(matches->at(j).queryIdx)[0],
                          test_keypoints.col(matches->at(j).queryIdx)[1]);
      observations.push_back(obs);
      landmarks.push_back(pid_to_xyz[landmark_id]);
      seen_landmarks.insert(landmark_id);
      num_matches++;
    }
    if (FLAGS_verbose_localization && num_matches > 0)
      std::cout << " " << cid_to_filename[cid];
  }
  if (FLAGS_verbose_localization) std::cout << std::endl;

  int ret = RansacEstimateCamera(landmarks, observations,
                                 num_ransac_iterations,
                                 ransac_inlier_tolerance, pose,
                                 inlier_landmarks, inlier_observations,
                                 FLAGS_verbose_localization);
  return (ret == 0);
}

bool SparseMap::Localize(std::string const& img_file,
                         camera::CameraModel* pose,
                         std::vector<Eigen::Vector3d>* inlier_landmarks,
                         std::vector<Eigen::Vector2d>* inlier_observations,
                         std::vector<int> * cid_list) {
  cv::Mat test_descriptors;
  Eigen::Matrix2Xd test_keypoints;
  bool multithreaded = false;
  DetectFeaturesFromFile(img_file, multithreaded, &test_descriptors, &test_keypoints);
  return sparse_mapping::Localize(test_descriptors, test_keypoints,
                                  std::cref(camera_params_),
                                  pose,
                                  inlier_landmarks, inlier_observations,
                                  cid_to_filename_.size(),
                                  detector_.GetDetectorName(),
                                  &vocab_db_,
                                  num_similar_,
                                  cid_to_filename_,
                                  cid_to_descriptor_map_,
                                  cid_to_keypoint_map_,
                                  cid_fid_to_pid_,
                                  pid_to_xyz_,
                                  num_ransac_iterations_,
                                  ransac_inlier_tolerance_,
                                  early_break_landmarks_,
                                  histogram_equalization_,
                                  cid_list);
}

// delete all the features that do not match to a landmark but are still around!
void SparseMap::PruneMap(void) {
#if 0
  // This is a good sanity check, print things before we start pruning
  for (unsigned int cid = 0; cid < cid_fid_to_pid_.size(); cid++) {
    for (int fid = 0; fid < cid_to_descriptor_map_[cid].rows; fid++) {
      if (cid_fid_to_pid_[cid].count(fid) == 0)
        continue;
      int pid = cid_fid_to_pid_[cid][fid];
      int rows = cid_to_keypoint_map_[cid].rows();  // must be equal to 2
      std::cout << "Value before: "
                << cid << ' ' << fid << ' ' << cid_to_descriptor_map_[cid].row(fid) << ' '
                << pid  << ' '
                <<  cid_to_keypoint_map_[cid].block(0, fid, rows, 1).transpose() << std::endl;
    }
  }
#endif

  for (unsigned int cid = 0; cid < cid_fid_to_pid_.size(); cid++) {
    std::vector<int> deleted_features;
    for (int fid = 0; fid < cid_to_descriptor_map_[cid].rows; fid++) {
      // delete if no matching landmark!
      if (cid_fid_to_pid_[cid].count(fid) == 0) {
        deleted_features.push_back(fid);
      }
    }
    if (deleted_features.size() == 0)
      continue;
    // create new descriptor map
    cv::Mat next_descriptor_map;
    next_descriptor_map.create(cid_to_descriptor_map_[cid].rows - deleted_features.size(),
                               cid_to_descriptor_map_[cid].cols, cid_to_descriptor_map_[cid].depth());
    int new_fid = 0;
    for (int fid = 0; fid < cid_to_descriptor_map_[cid].rows; fid++) {
      // delete if no matching landmark!
      if (cid_fid_to_pid_[cid].count(fid) == 0) {
        continue;
      } else {
        cid_to_descriptor_map_[cid].row(fid).copyTo(next_descriptor_map.row(new_fid));
        // fix indexing
        if (new_fid < fid) {
          int pid = cid_fid_to_pid_[cid][fid];
          // in localization mode this is empty
          if (pid_to_cid_fid_.size() > 0)
            pid_to_cid_fid_[pid][cid] = new_fid;
          cid_fid_to_pid_[cid][new_fid] = pid;
          cid_fid_to_pid_[cid].erase(fid);
        }
        new_fid++;
      }
    }
    cid_to_descriptor_map_[cid] = next_descriptor_map;

    // clean up other stuff
    for (int i = static_cast<int>(deleted_features.size() - 1); i >= 0; i--) {
      int fid = deleted_features[i];
      // these may not always exist if localizing
      if (cid_to_keypoint_map_.size() > 0) {
        int rows = cid_to_keypoint_map_[cid].rows();  // must be equal to 2
        int cols = cid_to_keypoint_map_[cid].cols();
        // TODO(oalexan1): Copying blocks like this repeatedly is
        // expensive.  It is simpler to just shift columns left one by
        // one, as done above.
        if (fid < cols - 1)
          cid_to_keypoint_map_[cid].block(0, fid, rows, cols - 1 - fid) =
            cid_to_keypoint_map_[cid].block(0, fid + 1, rows, cols - 1 - fid);
        cid_to_keypoint_map_[cid].conservativeResize(rows, cols - 1);
      }
    }
  }

  // This is not strictly necessary as all book-keeping was already done
  InitializeCidFidToPid();

#if 0
  // We must get everything same as before, except fid
  for (unsigned int cid = 0; cid < cid_fid_to_pid_.size(); cid++) {
    for (int fid = 0; fid < cid_to_descriptor_map_[cid].rows; fid++) {
      if (cid_fid_to_pid_[cid].count(fid) == 0)
        continue;
      int pid = cid_fid_to_pid_[cid][fid];
      int rows = cid_to_keypoint_map_[cid].rows();  // must be equal to 2
      std::cout << "Value after: "
                << cid << ' ' << fid << ' ' << cid_to_descriptor_map_[cid].row(fid) << ' '
                << pid  << ' '
                <<  cid_to_keypoint_map_[cid].block(0, fid, rows, 1).transpose() << std::endl;
    }
  }
#endif
}

// Reorder the images in the map and the rest of the data accordingly
void SparseMap::reorderMap(std::map<int, int> const& old_cid_to_new_cid) {
  int num_cid = cid_to_filename_.size();

  // Sanity checks
  if (old_cid_to_new_cid.size() != cid_to_filename_.size())
    LOG(FATAL) << "Wrong size of the permutation in SparseMap::reorderMap().";
  for (auto it = old_cid_to_new_cid.begin(); it != old_cid_to_new_cid.end(); it++) {
    int new_cid = it->second;
    if (new_cid >= num_cid) LOG(FATAL) << "Out of bounds in the permutation in SparseMap::reorderMap().";
  }

  // Wipe things that we won't reorder
  vocab_db_ = sparse_mapping::VocabDB();
  db_to_cid_map_.clear();
  cid_to_cid_.clear();
  user_cid_to_keypoint_map_.clear();
  user_pid_to_cid_fid_.clear();
  user_pid_to_xyz_.clear();
  cid_fid_to_pid_.clear();  // Will recreate this later

  // Must create temporary structures
  std::vector<std::string>        new_cid_to_filename(num_cid);
  std::vector<Eigen::Matrix2Xd>   new_cid_to_keypoint_map(num_cid);
  std::vector<Eigen::Affine3d>    new_cid_to_cam_t_global(num_cid);
  std::vector<cv::Mat>            new_cid_to_descriptor_map(num_cid);
  std::vector<std::map<int, int>> new_pid_to_cid_fid(pid_to_cid_fid_.size());

  // Note that pid_to_xyz_ is not changed by this reordering

  // Copy the data in new order
  for (int old_cid = 0; old_cid < num_cid; old_cid++) {
    auto it = old_cid_to_new_cid.find(old_cid);
    if (it == old_cid_to_new_cid.end())
      LOG(FATAL) << "Cannot find desired index in permutation in SparseMap::reorderMap().";

    int new_cid = it->second;

    new_cid_to_filename[new_cid] = cid_to_filename_[old_cid];
    new_cid_to_keypoint_map[new_cid] = cid_to_keypoint_map_[old_cid];
    new_cid_to_cam_t_global[new_cid] = cid_to_cam_t_global_[old_cid];
    new_cid_to_descriptor_map[new_cid] = cid_to_descriptor_map_[old_cid];
  }

  // pid_to_cid_fid needs special treatment
  for (size_t pid = 0; pid < pid_to_cid_fid_.size(); pid++) {
    auto const& cid_fid = pid_to_cid_fid_[pid];  // alias

    std::map<int, int> new_cid_fid;
    for (auto cid_fid_it = cid_fid.begin(); cid_fid_it != cid_fid.end(); cid_fid_it++) {
      int old_cid = cid_fid_it->first;
      auto cid_it = old_cid_to_new_cid.find(old_cid);
      if (cid_it == old_cid_to_new_cid.end())
        LOG(FATAL) << "Bookkeeping error in SparseMap::reorderMap()";

      int new_cid = cid_it->second;
      new_cid_fid[new_cid] = cid_fid_it->second;
    }

    new_pid_to_cid_fid[pid] = new_cid_fid;
  }

  // Swap in the new values
  cid_to_filename_.swap(new_cid_to_filename);
  cid_to_keypoint_map_.swap(new_cid_to_keypoint_map);
  cid_to_cam_t_global_.swap(new_cid_to_cam_t_global);
  cid_to_descriptor_map_.swap(new_cid_to_descriptor_map);
  pid_to_cid_fid_.swap(new_pid_to_cid_fid);

  // Recreate cid_fid_to_pid_ from pid_to_cid_fid_.
  InitializeCidFidToPid();
}

bool SparseMap::Localize(const cv::Mat & image, camera::CameraModel* pose,
                         std::vector<Eigen::Vector3d>* inlier_landmarks,
                         std::vector<Eigen::Vector2d>* inlier_observations,
                         std::vector<int> * cid_list) {
  bool multithreaded = false;
  cv::Mat test_descriptors;
  Eigen::Matrix2Xd test_keypoints;
  DetectFeatures(image, multithreaded, &test_descriptors, &test_keypoints);
  return sparse_mapping::Localize(test_descriptors, test_keypoints,
                                  std::cref(camera_params_),
                                  pose,
                                  inlier_landmarks, inlier_observations,
                                  cid_to_filename_.size(),
                                  detector_.GetDetectorName(),
                                  &vocab_db_,
                                  num_similar_,
                                  cid_to_filename_,
                                  cid_to_descriptor_map_,
                                  cid_to_keypoint_map_,
                                  cid_fid_to_pid_,
                                  pid_to_xyz_,
                                  num_ransac_iterations_,
                                  ransac_inlier_tolerance_,
                                  early_break_landmarks_,
                                  histogram_equalization_,
                                  cid_list);
}

bool SparseMap::Localize(const cv::Mat & test_descriptors, const Eigen::Matrix2Xd & test_keypoints,
                         camera::CameraModel* pose,
                         std::vector<Eigen::Vector3d>* inlier_landmarks,
                         std::vector<Eigen::Vector2d>* inlier_observations,
                         std::vector<int> * cid_list) {
  return sparse_mapping::Localize(test_descriptors, test_keypoints,
                                  std::cref(camera_params_),
                                  pose,
                                  inlier_landmarks, inlier_observations,
                                  cid_to_filename_.size(),
                                  detector_.GetDetectorName(),
                                  &vocab_db_,
                                  num_similar_,
                                  cid_to_filename_,
                                  cid_to_descriptor_map_,
                                  cid_to_keypoint_map_,
                                  cid_fid_to_pid_,
                                  pid_to_xyz_,
                                  num_ransac_iterations_,
                                  ransac_inlier_tolerance_,
                                  early_break_landmarks_,
                                  histogram_equalization_,
                                  cid_list);
}

}  // namespace sparse_mapping
