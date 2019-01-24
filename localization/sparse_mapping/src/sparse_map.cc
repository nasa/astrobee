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

#include <sparse_mapping/sparse_map.h>
#include <camera/camera_params.h>
#include <common/thread.h>
#include <common/utils.h>
#include <interest_point/matching.h>
#include <sparse_mapping/reprojection.h>
#include <sparse_mapping/sparse_mapping.h>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Geometry>

#include <sparse_map.pb.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>

#include <fstream>
#include <queue>
#include <set>
#include <thread>
#include <limits>

DEFINE_int32(num_similar, 20,
             "Use in localization this many images which "
             "are most similar to the image to localize.");
DEFINE_int32(num_ransac_iterations, 1000,
             "Use in localization this many images which "
             "are most similar to the image to localize.");
DEFINE_int32(ransac_inlier_tolerance, 3,
             "Use in localization this many images which "
             "are most similar to the image to localize.");
DEFINE_int32(early_break_landmarks, 100,
             "Break early when we have this many landmarks.");
DEFINE_int32(num_extra_localization_db_images, 0,
             "Match this many extra images from the Vocab DB, only keep num_similar.");
DEFINE_bool(verbose_localization, false,
            "If true, print more details on localization.");

namespace sparse_mapping {

SparseMap::SparseMap(const std::vector<std::string> & filenames,
                     const std::string & detector,
                     const camera::CameraParameters & params):
        cid_to_filename_(filenames),
        detector_(detector), camera_params_(params),
        num_similar_(FLAGS_num_similar),
        num_ransac_iterations_(FLAGS_num_ransac_iterations),
        ransac_inlier_tolerance_(FLAGS_ransac_inlier_tolerance) {
  cid_to_descriptor_map_.resize(cid_to_filename_.size());
  // TODO(bcoltin): only record scale and orientation for opensift?
  cid_to_keypoint_map_.resize(cid_to_filename_.size());
}

SparseMap::SparseMap(const std::string & protobuf_file, bool localization) :
  camera_params_(Eigen::Vector2i(-1, -1), Eigen::Vector2d::Constant(-1),
                 Eigen::Vector2d(-1, -1)),
  num_similar_(FLAGS_num_similar),
        num_ransac_iterations_(FLAGS_num_ransac_iterations),
        ransac_inlier_tolerance_(FLAGS_ransac_inlier_tolerance) {
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
        ransac_inlier_tolerance_(FLAGS_ransac_inlier_tolerance) {
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
// the bundler and theia maps with ours.
SparseMap::SparseMap(bool bundler_format, std::string const& filename,
                     std::vector<std::string> const& files) :
  camera_params_(Eigen::Vector2i(640, 480), Eigen::Vector2d::Constant(300),
                 Eigen::Vector2d(320, 240)),
  num_similar_(FLAGS_num_similar),
        num_ransac_iterations_(FLAGS_num_ransac_iterations),
        ransac_inlier_tolerance_(FLAGS_ransac_inlier_tolerance) {
  int num_cams;

  if (bundler_format) {
    std::ifstream is(filename.c_str());
    std::string line;
    std::getline(is, line);  // empty line
    is >> num_cams;
    cid_to_filename_ = files;
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

  } else {
    // Read the text dump of a theia map

    std::ifstream is(filename.c_str());
    is >> num_cams;
    cid_to_cam_t_global_.resize(num_cams);
    cid_to_filename_.resize(num_cams);

    for (int i = 0; i < num_cams; i++) {
      std::string line;
      std::getline(is, line);  // empty line
      std::getline(is, line);  // actual data
      cid_to_filename_[i] = line;

      Eigen::Matrix3d T;
      for (int row = 0; row < T.rows(); row++) {
        for (int col = 0; col < T.cols(); col++) {
          is >> T(row, col);
        }
      }

      Eigen::Vector3d P;
      for (int row = 0; row < P.size(); row++)
        is >> P[row];

      cid_to_cam_t_global_[i].linear() = T.transpose();  // not sure
      cid_to_cam_t_global_[i].translation() = P;
      cid_to_cam_t_global_[i] = cid_to_cam_t_global_[i].inverse();
    }

    int num_pts;
    is >> num_pts;
    pid_to_xyz_.resize(num_pts);
    pid_to_cid_fid_.resize(num_pts);
    for (int i = 0; i < num_pts; i++) {
      Eigen::Vector3d P;
      for (int row = 0; row < P.size(); row++) {
        is >> P[row];
      }
      pid_to_xyz_[i] = P;
    }
    is.close();
  }

  // Initialize other data expected in the map
  cid_to_keypoint_map_.resize(num_cams);
  cid_to_descriptor_map_.resize(num_cams);
}

// Detect features in given images
void SparseMap::DetectFeatures() {
  common::ThreadPool pool;
  bool multithreaded = true;
  size_t num_files = cid_to_filename_.size();
  for (size_t cid = 0; cid < num_files; cid++) {
    common::PrintProgressBar(stdout, static_cast<float>(cid) / static_cast<float>(num_files - 1));

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
      cid_to_descriptor_map_[cid].create(frame.feature_size(),     // rows
                                         descriptor_length,        // columns
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

  delete input;
  close(input_fd);
}

void SparseMap::SetDetectorParams(int min_features, int max_features, int retries) {
  detector_.Reset(detector_.GetDetectorName(), min_features, max_features, retries);
}

void SparseMap::Save(const std::string & protobuf_file) const {
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

  vocab_db_.SaveProtobuf(output);

  delete output;
  close(output_fd);
}


// Non-member InitializeCidFidToPid() function, useful
// without a fully-formed map.
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid) {
  // from pid_to_cid_fid, create cid_fid_to_pid for lookup
  // maybe we should store it this way in the first place?
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
  cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  DetectFeatures(image, multithreaded, descriptors, keypoints);
}

void SparseMap::DetectFeatures(const cv::Mat & image,
                               bool multithreaded,
                               cv::Mat* descriptors,
                               Eigen::Matrix2Xd* keypoints) {
  std::vector<cv::KeyPoint> storage;
  if (!multithreaded) {
    detector_.Detect(image, &storage, descriptors);
  } else {
    // When using multiple threads, need an individual detector
    // instance, to avoid a crash. This is being used only in
    // map-building, rather than in localization which is more
    // performance-sensitive.
    interest_point::FeatureDetector local_detector(detector_.GetDetectorName());
    local_detector.Detect(image, &storage, descriptors);
  }

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
              camera::CameraModel* pose,
              std::vector<Eigen::Vector3d>* inlier_landmarks,
              std::vector<Eigen::Vector2d>* inlier_observations,
              int num_cid,
              int max_cid_to_use,
              std::string const& detector_name,
              sparse_mapping::VocabDB * vocab_db,
              int num_similar,
              std::vector<std::string> const& cid_to_filename,
              std::vector<cv::Mat> const& cid_to_descriptor_map,
              std::vector<std::map<int, int> > const& cid_fid_to_pid,
              std::vector<Eigen::Vector3d> const& pid_to_xyz,
              int num_ransac_iterations, int ransac_inlier_tolerance) {
  // Query the vocab tree.
  std::vector<int> indices;
  sparse_mapping::QueryDB(detector_name,
                          vocab_db,
                          // Notice that we request more similar
                          // images than what we need. We'll prune
                          // them below.
                          num_similar + FLAGS_num_extra_localization_db_images,
                          test_descriptors,
                          &indices);
  if (indices.empty()) {
    LOG(WARNING) << "Localizing against all keyframes.";
    // Use all images, as no tree is available.
    for (int cid = 0; cid < num_cid; cid++)
      indices.push_back(cid);
  }

  // See if to use only a restricted set of cids.
  if (max_cid_to_use >= 0) {
    std::vector<int> indices2;
    for (size_t i = 0; i < indices.size(); i++) {
      if (indices[i] <= max_cid_to_use)
        indices2.push_back(indices[i]);
    }
    indices = indices2;
  }

  // Find matches to each image in map. Do this in two passes. First,
  // find matches to all map images, then keep only num_similar
  // best matched images, then localize against those.

  // TODO(bcoltin): don't include same landmark multiple times?
  // We will not localize using all images having matches, there are too
  // many false positives that way. Instead, limit ourselves to the images
  // which have most observations in common with the current one.
  // TODO(oalexan1): Perhaps we must not localize against images
  // which have too few observations in common with the current one
  // even after we reduce the number of images we localize against.
  std::vector<int> similarity_rank(indices.size(), 0);
  std::vector<std::vector<cv::DMatch> > all_matches(indices.size());
  int total = 0;
  for (size_t i = 0; i < indices.size(); i++) {
    int cid = indices[i];
    interest_point::FindMatches(test_descriptors,
                                cid_to_descriptor_map[cid],
                                &all_matches[i]);

    for (size_t j = 0; j < all_matches[i].size(); j++) {
      if (cid_fid_to_pid[cid].count(all_matches[i][j].trainIdx) == 0) {
        continue;
      }
      similarity_rank[i]++;
    }
    total += similarity_rank[i];
    if (total >= FLAGS_early_break_landmarks)
      break;
  }

  // TODO(oalexan1): Just finding matches among the images may not be
  // enough.  Here, just as in map-building, a geometric consistency
  // check may be necessary to remove spurious matches. I think I've
  // seen this be an issue but did not check it in detail.

  std::vector<Eigen::Vector2d> observations;
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<int> highly_ranked = common::rv_order(similarity_rank);
  int end = std::min(static_cast<int>(highly_ranked.size()), num_similar);
  for (int i = 0; i < end; i++) {
    int cid = indices[highly_ranked[i]];
    if (FLAGS_verbose_localization) std::cout << " " << cid_to_filename[cid];
    std::vector<cv::DMatch>* matches = &all_matches[highly_ranked[i]];
    for (size_t j = 0; j < matches->size(); j++) {
      if (cid_fid_to_pid[cid].count(matches->at(j).trainIdx) == 0)
        continue;
      Eigen::Vector2d obs(test_keypoints.col(matches->at(j).queryIdx)[0],
                          test_keypoints.col(matches->at(j).queryIdx)[1]);
      observations.push_back(obs);
      const int landmark_id = cid_fid_to_pid.at(cid).at(matches->at(j).trainIdx);
      landmarks.push_back(pid_to_xyz[landmark_id]);
    }
  }
  if (FLAGS_verbose_localization) std::cout << std::endl;

  int ret = RansacEstimateCamera(landmarks, observations,
        num_ransac_iterations,
        ransac_inlier_tolerance, pose,
        inlier_landmarks, inlier_observations);
  return (ret == 0);
}

bool SparseMap::Localize(std::string const& img_file,
                         camera::CameraModel* pose,
                         std::vector<Eigen::Vector3d>* inlier_landmarks,
                         std::vector<Eigen::Vector2d>* inlier_observations) {
  cv::Mat test_descriptors;
  Eigen::Matrix2Xd test_keypoints;
  int max_cid_to_use = -1;
  bool multithreaded = false;
  DetectFeaturesFromFile(img_file, multithreaded, &test_descriptors, &test_keypoints);
  return sparse_mapping::Localize(test_descriptors, test_keypoints, pose,
                                  inlier_landmarks, inlier_observations,
                                  cid_to_filename_.size(),
                                  max_cid_to_use,
                                  detector_.GetDetectorName(),
                                  &vocab_db_,
                                  num_similar_,
                                  cid_to_filename_,
                                  cid_to_descriptor_map_,
                                  cid_fid_to_pid_,
                                  pid_to_xyz_,
                                  num_ransac_iterations_,
                                  ransac_inlier_tolerance_);
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

bool SparseMap::Localize(const cv::Mat & image, camera::CameraModel* pose,
                         std::vector<Eigen::Vector3d>* inlier_landmarks,
                         std::vector<Eigen::Vector2d>* inlier_observations) {
  bool multithreaded = false;
  cv::Mat test_descriptors;
  Eigen::Matrix2Xd test_keypoints;
  DetectFeatures(image, multithreaded, &test_descriptors, &test_keypoints);
  int max_cid_to_use = -1;
  return sparse_mapping::Localize(test_descriptors, test_keypoints, pose,
                                  inlier_landmarks, inlier_observations,
                                  cid_to_filename_.size(),
                                  max_cid_to_use,
                                  detector_.GetDetectorName(),
                                  &vocab_db_,
                                  num_similar_,
                                  cid_to_filename_,
                                  cid_to_descriptor_map_,
                                  cid_fid_to_pid_,
                                  pid_to_xyz_,
                                  num_ransac_iterations_,
                                  ransac_inlier_tolerance_);
}

bool SparseMap::Localize(const cv::Mat & test_descriptors, const Eigen::Matrix2Xd & test_keypoints,
                         camera::CameraModel* pose,
                         std::vector<Eigen::Vector3d>* inlier_landmarks,
                         std::vector<Eigen::Vector2d>* inlier_observations) {
  int max_cid_to_use = -1;
  return sparse_mapping::Localize(test_descriptors, test_keypoints, pose,
                                  inlier_landmarks, inlier_observations,
                                  cid_to_filename_.size(),
                                  max_cid_to_use,
                                  detector_.GetDetectorName(),
                                  &vocab_db_,
                                  num_similar_,
                                  cid_to_filename_,
                                  cid_to_descriptor_map_,
                                  cid_fid_to_pid_,
                                  pid_to_xyz_,
                                  num_ransac_iterations_,
                                  ransac_inlier_tolerance_);
}

}  // namespace sparse_mapping
