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
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/reprojection.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sparse_map.pb.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

// This is a test executable, a way of running localization for a map
// without invoking ROS. The ROS localization_node is the ultimate
// localization test.

// Given:
// 1. A reference measured map, obtained by running parse_cam on a
//    measured dataset.
// 2. A reference computed map, computed from the reference measured
//    map images using SfM, the executable build_map.
// 3. A "source" measured map, obtained by running parse_cam on a
//    second measured dataset.

// Do:

//  - Localize every image in the source measured map against the
//    computed reference.

//  - Compare the obtained computed location and orientation against
//    the actual ground truth values in the source measured map.

//  - Optionally first bring the reference computed map in the
//    coordinate system of the reference measured map using a rigid
//    transform, so that the camera centers of the two maps align in the
//    beset possible way.

//  - Optionally save the reference computed map after registering it to the
//    reference measured map.

DEFINE_string(reference_computed, "reference_computed.map",
              "Reference computed map to localize against.");
DEFINE_string(reference_measured, "reference_measured.map",
              "Reference measured map to localize against.");
DEFINE_string(source_measured, "source_measured.map",
              "Localize images in this map against the reference computed map, "
              "and compare with measured locations from this map.");
DEFINE_string(results_file, "",
              "Save the localization results to a file.");
DEFINE_bool(perform_registration, false,
            "Register the reference computed map to the reference measured map "
            "by finding the best rigid transform among them, before localizing. "
            "This step must not be necessary, as the correct way to do map "
            "registration is by using control points. This approach is for "
            "debugging only.");
DEFINE_bool(save_registered_computed_map, false,
            "Save the computed map after registering it to the measured map.");
DEFINE_bool(quit_after_registration, false,
            "Quit as soon as the computed map is registered to the measured map.");

void EvalDiff(sparse_mapping::SparseMap const& meas,
                                sparse_mapping::SparseMap * comp) {
  // Find the rigid transform that best aligns the computed to
  // measured map. For that, need to identify which images in the
  // measured map are also in the computed map, and vice-versa.
  // Apply that transform only if FLAGS_perform_registration is true.

  int num_comp = (*comp).GetNumFrames();
  int num_meas = meas.GetNumFrames();
  std::map< std::string, int > img2ind1, img2ind2;
  for (int cid = 0; cid < num_comp; cid++) {
    // only the base part of the filename needs to match
    std::string filename = (*comp).GetFrameFilename(cid);
    img2ind1[filename] = cid;
  }
  for (int cid = 0; cid < num_meas; cid++) {
    std::string filename = meas.GetFrameFilename(cid);
    img2ind2[filename] = cid;
  }

  int num_common = 0;
  for (std::map<std::string, int>::iterator it = img2ind1.begin();
       it != img2ind1.end() ; it++) {
    if (img2ind2.find(it->first) != img2ind2.end() )
      num_common++;
  }

  Eigen::Matrix3Xd comp_ctrs(3, num_common), meas_ctrs(3, num_common);
  int count = 0;
  std::map<int, std::string> index2file;

  for (std::map<std::string, int>::iterator it = img2ind1.begin();
       it != img2ind1.end() ; it++) {
    std::map<std::string, int>::iterator it2 = img2ind2.find(it->first);

    if (it2 == img2ind2.end() ) continue;
    if (it->first != it2->first) LOG(FATAL) << "Book-keeping failure.";
    comp_ctrs.col(count)
      = (*comp).GetFrameGlobalTransform(it->second).inverse().translation();
    meas_ctrs.col(count)
      = meas.GetFrameGlobalTransform(it2->second).inverse().translation();
    index2file[count] = it->first;
    count++;
  }

  Eigen::Affine3d world_transform;
  world_transform.linear() = Eigen::Matrix3d::Identity();
  world_transform.translation() = Eigen::Vector3d(0, 0, 0);
  if (FLAGS_perform_registration)
    sparse_mapping::Find3DAffineTransform(comp_ctrs, meas_ctrs, &world_transform);

  // Dump the difference between two maps to text file, to view it in Matlab
  std::string diff = FLAGS_reference_computed + ".diff.txt";
  LOG(INFO) << "Writing: " << diff;
  std::ofstream df(diff.c_str());
  df.precision(18);

  // Find the mean position error after registering the
  // computed map to the measured map.
  double mean_err = 0;
  std::vector<double> errors;
  for (int i = 0; i < meas_ctrs.cols(); i++) {
    double err = (world_transform*comp_ctrs.col(i) - meas_ctrs.col(i)).norm();
    mean_err += err;
    errors.push_back(err);
     df << meas_ctrs.col(i).transpose()  << " "
        << (world_transform*comp_ctrs.col(i)).transpose() << std::endl;
  }
  df.close();

  mean_err /= meas_ctrs.cols();
  std::cout.precision(18);
  std::cout << "Mean absolute error between (registered) computed and measured map: "
            << mean_err << " meters" << std::endl;

  double scale = pow(world_transform.linear().determinant(), 1.0/3.0);
  std::cout << "Registration transform (matrix and translation):\n"
            << world_transform.linear() << "\n"
            << world_transform.translation() <<"\n"
            << "scale: " << scale << std::endl;

#if 0
  // Useful debug info. Dump the computed and measured maps right before
  // registration.
  {
    std::string comp_traj = "comp_map.txt";
    LOG(INFO) << "Writing: " << comp_traj << std::endl;
    std::ofstream it(comp_traj.c_str());
    it.precision(18);

    for (size_t i = 0; i < (*comp).GetNumFrames(); i++) {
      it << (*comp).GetFrameGlobalTransform(i).inverse().translation().transpose()
         << std::endl;
    }
    it.close();
  }

  {
    std::string meas_traj = "meas_map.txt";
    LOG(INFO) << "Writing: " << meas_traj << std::endl;
    std::ofstream it(meas_traj.c_str());
    it.precision(18);

    for (size_t i = 0; i < (meas).GetNumFrames(); i++) {
      it << (meas).GetFrameGlobalTransform(i).inverse().translation().transpose()
         << std::endl;
    }
    it.close();
  }
#endif

  (*comp).ApplyTransform(world_transform);

  if (FLAGS_save_registered_computed_map) {
    std::string registered = FLAGS_reference_computed + ".registered.map";
    (*comp).Save(registered);
  }
}

// Avoid using this EvaluatePoint function, keeps on crashing
// for some reason.
void EvaluatePoint(const sparse_mapping::SparseMap & source_meas,
    sparse_mapping::SparseMap* ref_comp, int cid, FILE* results,
    pthread_mutex_t* results_mutex) {
  std::string img_file = source_meas.GetFrameFilename(cid);

  // localize frame
  camera::CameraModel camera(Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity(),
                                     ref_comp->GetCameraParameters());
  if (!ref_comp->Localize(img_file, &camera)) {
    // Localization failed
    pthread_mutex_lock(results_mutex);
    std::cout << "Errors for " << img_file << ": "
              << 1e+6 << " cm " << 1e+6 << " degrees" << std::endl;
    pthread_mutex_unlock(results_mutex);
    return;
  }

  camera::CameraModel measured(source_meas.GetFrameGlobalTransform(cid),
                               ref_comp->GetCameraParameters());
  Eigen::Vector3d expected_angle = measured.GetRotation() * Eigen::Vector3d::UnitX();
  Eigen::Vector3d estimated_angle =  camera.GetRotation() * Eigen::Vector3d::UnitX();
  double angle_err = acos(estimated_angle.dot(expected_angle)) * (180.0 / M_PI);

  pthread_mutex_lock(results_mutex);
  std::cout << "Measured position: " << measured.GetPosition().transpose() << std::endl;
  std::cout << "Computed position: " << camera.GetPosition().transpose() << std::endl;
  std::cout << "Errors for " << img_file << ": " << (camera.GetPosition() -
      measured.GetPosition()).norm() * 100 << " cm " << angle_err << " degrees" << std::endl;
  if (results != NULL) {
    Eigen::Vector3d a = measured.GetPosition();
    Eigen::Vector3d b = camera.GetPosition();
    fprintf(results, "%s %g %g %g %g %g %g %g %g\n", img_file.c_str(),
               (camera.GetPosition() - measured.GetPosition()).norm() * 100,
               angle_err, a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
    fflush(results);
  }
  pthread_mutex_unlock(results_mutex);
}

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  sparse_mapping::SparseMap ref_comp(FLAGS_reference_computed);

  sparse_mapping::SparseMap ref_meas(FLAGS_reference_measured);

  EvalDiff(ref_meas, &ref_comp);

  if (FLAGS_quit_after_registration)
    return 0;

  // FILE* results = NULL;
  // if (FLAGS_results_file.size() > 0) {
  //   results = fopen(FLAGS_results_file.c_str(), "w");
  // }

  Eigen::IOFormat CSVFormat(3, 0, ", ", ",   ");

  sparse_mapping::SparseMap source_meas(FLAGS_source_measured);

  // pthread_mutex_t write_mutex;
  // pthread_mutex_init(&write_mutex, NULL);
  // common::ThreadPool pool;
  for (size_t cid = 0; cid < source_meas.GetNumFrames(); cid++) {
    std::string img_file = source_meas.GetFrameFilename(cid);

    // localize frame
    camera::CameraModel camera(Eigen::Vector3d(), Eigen::Matrix3d::Identity(),
                               ref_comp.GetCameraParameters());
    if (!ref_comp.Localize(img_file, &camera)) {
      // Localization failed
      // pthread_mutex_lock(results_mutex);
      std::cout << "Errors for " << img_file << ": "
                << 1e+6 << " cm " << 1e+6 << " degrees" << std::endl;
      continue;
      // pthread_mutex_unlock(results_mutex);
      // return;
    }

    camera::CameraModel measured(source_meas.GetFrameGlobalTransform(cid),
                                 ref_comp.GetCameraParameters());
    Eigen::Vector3d expected_angle = measured.GetRotation() * Eigen::Vector3d::UnitX();
    Eigen::Vector3d estimated_angle =  camera.GetRotation() * Eigen::Vector3d::UnitX();
    double angle_err = acos(estimated_angle.dot(expected_angle)) * (180.0 / M_PI);

    // pthread_mutex_lock(results_mutex);
    std::cout << "Measured position: " << measured.GetPosition().transpose() << std::endl;
    std::cout << "Computed position: " << camera.GetPosition().transpose() << std::endl;
    std::cout << "Errors for " << img_file << ": "
              << (camera.GetPosition() - measured.GetPosition()).norm() * 100
              << " cm " << angle_err << " degrees" << std::endl;

    // The code below was causing a crash and was turned off.
    // if (results != NULL) {
    //   Eigen::Vector3d a = measured.GetPosition();
    //   Eigen::Vector3d b = camera.GetPosition();
    //   fprintf(results, "%s %g %g %g %g %g %g %g %g\n", img_file.c_str(),
    //           (camera.GetPosition() - measured.GetPosition()).norm() * 100,
    //           angle_err, a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
    //   fflush(results);
    //   }
    //   pthread_mutex_unlock(results_mutex);

    // LOG_EVERY_N(INFO, 100) << "\t" << 100 * static_cast<float>(cid) /
    //   static_cast<float>(source_meas.GetNumFrames())
    // << "% Progress";

    // pool.AddTask(&EvaluatePoint, std::ref(source_meas), std::ref(ref_comp), cid, results, &write_mutex);
  }
  // pool.Join();
  // pthread_mutex_destroy(&write_mutex);
  // fclose(results);

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
