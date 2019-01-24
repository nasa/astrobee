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
#include <config_reader/config_reader.h>
#include <camera/camera_params.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/reprojection.h>
#include <sparse_mapping/tensor.h>
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

// outputs
DEFINE_string(output_map, "output.map",
              "Output file containing the matches and control network.");

// parameters used in feature detection step only
DEFINE_int32(sample_rate, 1,
              "Add one of every n input frames to the map.");
DEFINE_bool(save_individual_maps, false,
            "If true, save separately the maps after detection, matching, track building, "
            "tensor initialization, and bundle adjustment.");

// output map parameters
DEFINE_string(detector, "SURF",
              "Feature detector to use. Options are [FAST, STAR, SIFT, SURF, ORB, "
              "BRISK, ORGBRISK, MSER, GFTT, HARRIS, Dense].");
DEFINE_string(rebuild_detector, "ORGBRISK",
              "Feature detector to use. Options are [FAST, STAR, SIFT, SURF, ORB, "
              "BRISK, ORGBRISK, MSER, GFTT, HARRIS, Dense].");

// control options
DEFINE_bool(feature_detection, false,
              "If true, perform compute features for input NVMs.");
DEFINE_bool(feature_matching, false,
              "If true, perform generate map from feature matching step.");
DEFINE_bool(track_building, false,
              "If true, perform generate map from feature matching step.");
DEFINE_bool(incremental_ba, false,
              "If true, perform incremental bundle adjustment.");
DEFINE_bool(loop_closure, false,
              "If true, take a map where images start repeating, and close the loop.");
DEFINE_bool(tensor_initialization, false,
              "If true, perform update output_nvm with tensor initialization.");
DEFINE_bool(bundle_adjustment, false,
              "If true, perform update output_nvm with bundle adjustment.");
DEFINE_bool(skip_pruning, false,
              "If true, do not prune maps, as pruned maps cannot be merged.");
DEFINE_bool(rebuild, false,
              "If true, rebuild the map with BRISK features.");
DEFINE_bool(rebuild_replace_camera, false,
              "If true, during rebuilding replace the camera with the one from ASTROBEE_ROBOT.");
DEFINE_bool(vocab_db, false,
              "If true, build the map with a vocabulary database.");
DEFINE_bool(registration, false,
            "Register the map to world coordinates(requires control points and their xyz coordinates). "
            "This new data is used to redo the bundle adjustment");
DEFINE_bool(verification, false,
            "Verify how an already registered map performs on an independently "
            "acquired set of control points and 3D measurements.");
DEFINE_bool(registration_skip_bundle_adjustment, false,
            "Skip bundle adjustment during the registration step.");
DEFINE_bool(info, false,
              "If true, just print some information on the existing map.");
DEFINE_int32(num_repeat_images, 0,
             "How many images from the beginning of the sequence to repeat at the end "
             "of the sequence, to help with loop closure (assuming first and last images "
             "are similar).");
DEFINE_bool(fix_cameras, false,
            "Keep the cameras fixed during bundle adjustment.");
DEFINE_bool(rebuild_refloat_cameras, false,
            "If true, optimize the cameras as well as part of rebuilding. Usually that is "
            "avoided when rebuilding with ORGBRISK, but could be useful with SURF.");

DEFINE_int32(db_restarts, 1, "Number of restarts when building the tree.");
DEFINE_int32(db_depth, 0, "Depth of the tree to build. Default: 4");
DEFINE_int32(db_branching_factor, 0, "Branching factor of the tree to build. "
             "Default: 10");

void DetectAllFeatures(int argc, char** argv) {
  // Check for user mistakes
  if (argc <= 1) {
    LOG(INFO) << "Usage: " << argv[0] << " <images>";
    exit(1);
    return;
  }

  LOG(INFO) << "Detecting features.";

  // load the first file to determine the resolution
  cv::Mat image = cv::imread(std::string(argv[1]), CV_LOAD_IMAGE_GRAYSCALE);

  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  if (!config.ReadFiles()) {
    LOG(ERROR) << "Failed to read config files.";
    exit(1);
    return;
  }
  camera::CameraParameters cam_params(&config, "nav_cam");

  // Useful for over-riding any config files when debugging
  // camera::CameraParameters cam_params(Eigen::Vector2i(776, 517),
  //                                    Eigen::Vector2d::Constant(610.502),
  //                                    Eigen::Vector2d(776/2.0, 517/2.0));

  // Remove some images from list based on sample rate
  int count = 0;
  int index = 1;
  for (int i = 1; i < argc; i++) {
    count++;
    if (count != FLAGS_sample_rate)
      continue;
    count = 0;
    argv[index] = argv[i];
    index++;
  }
  argc = index;

  std::vector<std::string> files(argc - 1);
  for (int i = 0; i < argc - 1; i++) {
    files[i] = std::string(argv[i + 1]);
  }

  // This is so that we can do loop closure later
  FLAGS_num_repeat_images = std::min(FLAGS_num_repeat_images, static_cast<int>(files.size()));
  for (int i = 0; i < FLAGS_num_repeat_images; i++) files.push_back(files[i]);

  // This will invoke a detection process
  sparse_mapping::SparseMap map(files, FLAGS_detector, cam_params);
  map.SetDetectorParams(1000, 20000, 3);
  map.DetectFeatures();

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".detect.map");
}

void MatchFeatures() {
  LOG(INFO) << "Matching features.";

  sparse_mapping::SparseMap map(FLAGS_output_map);
  sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile(FLAGS_output_map),
                                sparse_mapping::MatchesFile(FLAGS_output_map), &map);
  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".match.map");
}

void BuildTracks() {
  LOG(INFO) << "Building tracks.";

  sparse_mapping::SparseMap map(FLAGS_output_map);
  bool rm_invalid_xyz = false;  // we don't have valid cameras, so can't rm xyz
  sparse_mapping::BuildTracks(rm_invalid_xyz,
                              sparse_mapping::MatchesFile(FLAGS_output_map),
                              &map);
  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".track.map");
}

void IncrementalBA() {
  LOG(INFO) << "Beginning incremental bundle adjustment.";

  sparse_mapping::SparseMap map(FLAGS_output_map);

  sparse_mapping::IncrementalBA(sparse_mapping::EssentialFile(FLAGS_output_map), &map);

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".incremental.map");
}

void CloseLoop() {
  LOG(INFO) << "Beginning loop closure.";

  sparse_mapping::SparseMap map(FLAGS_output_map);

  sparse_mapping::CloseLoop(&map);
  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".closed.map");
}

void BundleAdjust() {
  LOG(INFO) << "Performing bundle adjustment.";
  sparse_mapping::SparseMap map(FLAGS_output_map);

  bool fix_cameras = FLAGS_fix_cameras;
  sparse_mapping::BundleAdjust(fix_cameras, &map);
  if (!FLAGS_skip_pruning) {
    LOG(INFO) << "Pruning map.\n";
    map.PruneMap();
  }

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".bundle.map");
}

// rebuilds with a different descriptor and detector
void Rebuild() {
  LOG(INFO) << "Rebuilding map with " << FLAGS_rebuild_detector << " detector.";
  sparse_mapping::SparseMap original(FLAGS_output_map);

  camera::CameraParameters params = original.GetCameraParameters();
  if (FLAGS_rebuild_replace_camera) {
    char * bot_ptr = getenv("ASTROBEE_ROBOT");
    if (bot_ptr == NULL)
      LOG(FATAL) << "Must set ASTROBEE_ROBOT.";
    LOG(INFO) << "Using camera for robot: " << bot_ptr << ".";
    config_reader::ConfigReader config;
    config.AddFile("cameras.config");
    if (!config.ReadFiles()) {
      LOG(ERROR) << "Failed to read config files.";
      exit(1);
      return;
    }
    params = camera::CameraParameters(&config, "nav_cam");
  }

  std::vector<std::string> files(original.GetNumFrames());
  for (size_t i = 0; i < original.GetNumFrames(); i++) {
    files[i] = original.GetFrameFilename(i);
  }

  LOG(INFO) << "Detecting features.";
  sparse_mapping::SparseMap map(files, FLAGS_rebuild_detector, params);
  map.SetDetectorParams(100, 20000, 3);
  map.DetectFeatures();

  LOG(INFO) << "Matching features.";
  // Borrow from the original map which images should be matched with which.
  map.cid_to_cid_.clear();
  for (size_t p = 0; p < original.pid_to_cid_fid_.size(); p++) {
    std::map<int, int> const &track = original.pid_to_cid_fid_[p];
    for (std::map<int, int>::const_iterator it1 = track.begin();
         it1 != track.end() ; it1++) {
      for (std::map<int, int>::const_iterator it2 = it1;
           it2 != track.end() ; it2++) {
        if (it1->first != it2->first) {
          // Never match an image with itself
          map.cid_to_cid_[it1->first].insert(it2->first);
        }
      }
    }
  }

  sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile(FLAGS_output_map),
                                sparse_mapping::MatchesFile(FLAGS_output_map), &map);
  for (unsigned int i = 0; i < original.GetNumFrames(); i++)
    map.SetFrameGlobalTransform(i, original.GetFrameGlobalTransform(i));

  LOG(INFO) << "Building tracks.";
  bool rm_invalid_xyz = true;  // by now cameras are good, so filter out bad stuff
  sparse_mapping::BuildTracks(rm_invalid_xyz,
                              sparse_mapping::MatchesFile(FLAGS_output_map),
                              &map);

  // It is essential that during re-building we do not vary the
  // cameras. Those were usually computed with a lot of SURF features,
  // while rebuilding is usually done with many fewer ORGBRISK
  // features.
  bool fix_cameras = !FLAGS_rebuild_refloat_cameras;
  if (fix_cameras)
    LOG(INFO) << "Performing bundle adjustment with fixed cameras.";
  else
    LOG(INFO) << "Performing bundle adjustment while floating cameras.";

  sparse_mapping::BundleAdjust(fix_cameras, &map);

  if (!FLAGS_skip_pruning) {
    LOG(INFO) << "Pruning map.\n";
    map.PruneMap();
  }

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + "." +
                                           FLAGS_rebuild_detector + ".map");
}

void VocabDB() {
  LOG(INFO) << "Building vocabulary database.";
  int depth, branching_factor;
  if (FLAGS_db_depth != 0)
    depth = FLAGS_db_depth;
  else
    depth = 6;
  if (FLAGS_db_branching_factor != 0)
    branching_factor = FLAGS_db_branching_factor;
  else
    branching_factor = 10;

  std::string detector;
  {
    // Temporarily load the map to guess the descriptor
    sparse_mapping::SparseMap m(FLAGS_output_map);
    detector = m.detector_.GetDetectorName();
  }

  sparse_mapping::BuildDB(FLAGS_output_map,
                          detector, depth, branching_factor,
                          FLAGS_db_restarts);
}

// Do either registration or verification
void RegistrationOrVerification(std::vector<std::string> const& data_files) {
  if (FLAGS_registration && FLAGS_verification)
    LOG(FATAL) << "Cannot perform both registration and verification in one step.";

  if (FLAGS_registration)
    LOG(INFO) << "Beginning registration to world coordinates.";

  sparse_mapping::SparseMap map(FLAGS_output_map);

  sparse_mapping::RegistrationOrVerification(data_files, FLAGS_verification, &map);

  if (FLAGS_verification)
    return;

  if (!FLAGS_registration_skip_bundle_adjustment) {
    LOG(INFO) << "Redoing bundle adjustment incorporating the user control points.";
    bool fix_cameras = false;
    BundleAdjust(fix_cameras, &map);
  }

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".registered.map");
}

void MapInfo() {
  sparse_mapping::SparseMap map(FLAGS_output_map);

  LOG(INFO) << "\t" << map.GetNumFrames() << " cameras and "
            << map.GetNumLandmarks()      << " points.";

  LOG(INFO) << "Images in the map: ";
  for (unsigned int cid = 0; cid < map.cid_to_filename_.size(); cid++) {
    LOG(INFO) << cid << " " << map.cid_to_filename_[cid];
  }
}


int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  // If the user selected no steps ... they selected all steps
  if (!FLAGS_feature_detection && !FLAGS_feature_matching &&
      !FLAGS_track_building && !FLAGS_incremental_ba &&
      !FLAGS_loop_closure && !FLAGS_tensor_initialization &&
      !FLAGS_bundle_adjustment && !FLAGS_rebuild &&
      !FLAGS_vocab_db &&
      !FLAGS_registration && !FLAGS_verification && !FLAGS_info) {
    FLAGS_feature_detection = true;
    FLAGS_feature_matching = true;
    FLAGS_track_building = true;
    FLAGS_incremental_ba = true;
    FLAGS_bundle_adjustment = true;
    FLAGS_rebuild = true;
    FLAGS_vocab_db = true;
  }

  if (FLAGS_feature_detection) {
    DetectAllFeatures(argc, argv);
  }
  if (FLAGS_feature_matching) {
    MatchFeatures();
  }
  if (FLAGS_track_building) {
    BuildTracks();
  }
  if (FLAGS_incremental_ba) {
    IncrementalBA();
  }
  if (FLAGS_loop_closure) {
    CloseLoop();
  }
  if (FLAGS_bundle_adjustment) {
    BundleAdjust();
  }
  if (FLAGS_rebuild) {
    Rebuild();
  }
  if (FLAGS_vocab_db) {
    VocabDB();
  }

  if (FLAGS_registration || FLAGS_verification) {
    std::vector<std::string> data_files;
    for (int arg = 1; arg < argc; arg++) {
      data_files.push_back(argv[arg]);
    }
    RegistrationOrVerification(data_files);
  }

  if (FLAGS_info) {
    MapInfo();
  }

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
