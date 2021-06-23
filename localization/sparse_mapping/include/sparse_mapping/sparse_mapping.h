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

#ifndef SPARSE_MAPPING_SPARSE_MAPPING_H_
#define SPARSE_MAPPING_SPARSE_MAPPING_H_

#include <camera/camera_params.h>
#include <common/eigen_vectors.h>

#include <Eigen/Geometry>

#include <vector>
#include <map>
#include <string>

// Forward declare opencv keypoint
namespace cv {
class KeyPoint;
class Mat;
class DMatch;
}

namespace common {
  class CameraParameters;
}

namespace google {
  namespace protobuf {
    class MessageLite;
    namespace io {
      class ZeroCopyInputStream;
      class ZeroCopyOutputStream;
    }
  }
}

namespace sparse_mapping {


  // Terminology used in this code:

  //  CID = Camera ID. A unique ID for each camera
  //  PID = Point ID. A unique ID for each point. Can be observed by
  //        multiple cameras.
  //  FID = Feature ID. A unique ID for each pixel viewing of a
  //        PID. FID only make sense in the context of a CID. FID
  //        values are reused for each image.

  // cid_to_keypoint_map - Indexed first on CID and then on FID.
  // cid_to_filename - Indexed on CID.
  // pid_to_cid_fid - Index on PID. Then returns map index on CID that
  //    shows FID observing this PID.
  // pid_to_xyz - Index on PID. Gives the XYZ position of each point.
  // cid_to_camera_transform - Index on CID. Contains affine transform
  //    representing camera_t_global.

  Eigen::Quaternion<double> slerp_n(std::vector<double> const& W,
                                    std::vector<Eigen::Quaternion<double> > const& Q);

  bool IsBinaryDescriptor(std::string const& descriptor);

  // Logic for implementing if two histogram equalization flags are compatible
  void HistogramEqualizationCheck(int histogram_equalization1, int histogram_equalization2);

  // Writes the NVM control network format.
  void WriteNVM(std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                std::vector<std::string> const& cid_to_filename,
                std::vector<std::map<int, int> > const& pid_to_cid_fid,
                std::vector<Eigen::Vector3d> const& pid_to_xyz,
                std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
                double focal_length,
                std::string const& output_filename);
  // Reads the NVM control network format.
  void ReadNVM(std::string const& input_filename,
               std::vector<Eigen::Matrix2Xd > * cid_to_keypoint_map,
               std::vector<std::string> * cid_to_filename,
               std::vector<std::map<int, int> > * pid_to_cid_fid,
               std::vector<Eigen::Vector3d> * pid_to_xyz,
               std::vector<Eigen::Affine3d> * cid_to_cam_t_global);

  // Adds yaml.gz or .txt extension, depending on descriptor
  std::string ImageToFeatureFile(std::string const& image_file,
                                 std::string const& detector_name);

  // The name of the file storing the list of images
  std::string DBImagesFile(std::string const& db_name);

  // The name of the matches file
  std::string MatchesFile(std::string const& map_file);

  // The name of the essential file
  std::string EssentialFile(std::string const& map_file);

  // Write features yaml file
  void WriteFeatures(std::string const& detector_name,
                     std::vector<cv::KeyPoint> const& keypoints,
                     cv::Mat const& descriptors,
                     std::string const& output_filename);

  // Read features yaml file
  bool ReadFeatures(std::string const& input_filename,
                    std::string const& detector_name,
                    std::vector<cv::KeyPoint> * keypoints,
                    cv::Mat * descriptors);

  // Read SIFT features in Lowe's format
  int ReadFeaturesSIFT(std::string const& filename,
                       cv::Mat * descriptors,
                       std::vector<cv::KeyPoint> * keypoints);

  // Triangulate metric camera point
  //     unnormalized point means that the point is:
  //     [px (image loc) - cx (optical center), py - cy, f (f length in px)]
  Eigen::Vector3d
  TriangulatePoint(Eigen::Vector3d const& unnormalized_pt1,
                   Eigen::Vector3d const& unnormalized_pt2,
                   Eigen::Matrix3d const& cam2_r_cam1,
                   Eigen::Vector3d const& cam2_t_cam1,
                   double* error);

  // Decompose Fundamental Matrix into Essential Matrix given known
  // Intrinsics Matrix.
  void DecomposeFMatIntoEMat(Eigen::Matrix3d const& fundamental,
                             Eigen::Matrix3d const& intrinsics,
                             Eigen::Matrix3d * essential);

  // Decompose Essential Matrix into R and T
  void DecomposeEMatIntoRT(Eigen::Matrix3d const& essential,
                           Eigen::Matrix2Xd const& unnormalized_pts1,
                           Eigen::Matrix2Xd const& unnormalized_pts2,
                           std::vector<cv::DMatch> const& matches,
                           double focal_length1,  // Camera 1
                           double focal_length2,  // Camera 2
                           Eigen::Matrix3d * cam2_r_cam1,
                           Eigen::Vector3d * cam2_t_cam1);

  // Utility to return the type of entries in a given matrix
  std::string CvMatTypeStr(cv::Mat const& Mat);

  void ListToListMap(std::vector<std::string> const& big_list,
                     std::vector<std::string> const& small_list,
                     std::map<int, int> * map);

  void MergePids(int repeat_index, int num_unique,
                 std::vector<std::map<int, int> > * pid_to_cid_fid);

  void PrintPidStats(std::vector<std::map<int, int> > const& pid_to_cid_fid);

  // Extract control points and the images they correspond to from
  // a hugin project file
  void ParseHuginControlPoints(std::string const& hugin_file,
                               std::vector<std::string> * images,
                               Eigen::MatrixXd * points);

  // Parse a file having on each line xyz coordinates
  void ParseXYZ(std::string const& xyz_file, Eigen::MatrixXd * xyz);

  // Parse a CSV file, with the first line having column names. Return
  // the results as columns in an std::map, with the column name being
  // the key. We assume all values are numbers (non-numbers are set to
  // 0).
  void ParseCSV(std::string const& csv_file,
                std::map< std::string, std::vector<double> > *cols);

  // save size before protbuf, to save multiple protobufs in one file
  bool WriteProtobufTo(const google::protobuf::MessageLite& message,
                     google::protobuf::io::ZeroCopyOutputStream* rawOutput);

  // save size before file, then write file into rawOutput
  bool WriteFileTo(const char* filename,
                     google::protobuf::io::ZeroCopyOutputStream* rawOutput);

  // read size before protbuf, to save multiple protobufs in one file
  bool ReadProtobufFrom(google::protobuf::io::ZeroCopyInputStream* rawInput,
                      google::protobuf::MessageLite* message);

  // read a size before protobuf, then read a file and write it to filename
  bool ReadFileFrom(google::protobuf::io::ZeroCopyInputStream* rawInput, const char* filename);

  // Apply a given transform to the specified xyz points, and adjust
  // accordingly the cameras for consistency.
  void TransformCamerasAndPoints(Eigen::Affine3d const& A,
                                 std::vector<Eigen::Affine3d> *cid_to_cam_t,
                                 std::vector<Eigen::Vector3d> *xyz);

  // Get the error threshold based on a multiple of a percentile
  double GetErrThresh(const std::vector<double> & errors, double factor);

  // Find the maximum angle between n rays intersecting at given
  // point. Must compute the camera centers in the global coordinate
  // system before calling this function.
  double ComputeRaysAngle(int pid,
                          std::vector<std::map<int, int> > const& pid_to_cid_fid,
                          std::vector<Eigen::Vector3d> const & cam_ctrs,
                          std::vector<Eigen::Vector3d> const& pid_to_xyz);

  // Filter points by reprojection error and other criteria
  void FilterPID(double reproj_thresh,
                 camera::CameraParameters const& camera_params,
                 std::vector<Eigen::Affine3d > const& cid_to_cam_t_global,
                 std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                 std::vector<std::map<int, int> > * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz,
                 bool print_stats = true, double multiple_of_median = 3.0);

  // Write the BAL format.
  bool WriteBAL(const std::string& filename,
                camera::CameraParameters const& camera_params,
                std::vector<std::map<int, int> > const& pid_to_cid_fid,
                std::vector<Eigen::Vector3d> const& pid_to_xyz,
                std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
                std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map);

  // Given a data sequence having camera pose information for
  // a set of timestamps, interpolate those poses at the timestamps
  // given in out_time. We assume timestamps are always in increasing values.
  void PoseInterpolation(std::vector<std::string> const& images,
                         std::vector<double> const& out_time,
                         std::map< std::string, std::vector<double> >
                         const& data,
                         std::vector<Eigen::Affine3d> * cid_to_cam_t,
                         std::vector<std::string> * good_images);

}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAPPING_H_
