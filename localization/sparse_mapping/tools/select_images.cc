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
#include <camera/camera_params.h>
#include <sparse_mapping/sparse_map.h>
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

// From a given set of images, eliminate images such that the sequence
// is still contiguous, but has fewer images.

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc < 2) {
    LOG(ERROR) << "No input images specified.";
    exit(0);
  }

  interest_point::FeatureDetector detector;
  cv::Mat image1, image2, image_last, descriptors1, descriptors2, descriptors_last;
  std::vector<cv::KeyPoint> storage;

  detector.Reset("ORGBRISK", 100, 20000, 20, 3);

  printf("Removing duplicate images...\n");
  common::PrintProgressBar(stdout, 0.0);
  image1 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  detector.Detect(image1, &storage, &descriptors1);
  int deleted_count = 0;
  for (int i = 1; i < argc; i++) {
    for (int j = i + 1; j < argc; j++) {
      std::vector<cv::DMatch> matches;
      storage.clear();
      descriptors2 = cv::Mat();
      image2 = cv::imread(argv[j], CV_LOAD_IMAGE_GRAYSCALE);
      detector.Detect(image2, &storage, &descriptors2);
      interest_point::FindMatches(descriptors1, descriptors2, &matches);
      common::PrintProgressBar(stdout, static_cast<float>(j - 1) / (argc - 2));
      // if many descriptors match, we can ignore
      if (matches.size() >= static_cast<unsigned int>(descriptors1.rows / 4) || matches.size() >= 2000) {
        image_last = image2;
        descriptors_last = descriptors2;
        // we can delete everything up to here in this special case
        if (j == argc - 1) {
          for (int k = i + 1; k < argc; k++) {
            if (std::remove(argv[k])) {
              LOG(WARNING) << "Couldn't delete " << argv[k] << ".";
            }
            deleted_count++;
          }
          i = argc;
        }
        continue;
      } else {
        // we didn't skip anything
        if (j == i + 1) {
          image1 = image2;
          descriptors1 = descriptors2;
          if (matches.size() < static_cast<unsigned int>(descriptors1.rows / 10) && matches.size() < 300) {
            LOG(WARNING) << "Few matches between " << argv[i] << " and " << argv[j];
          }
        } else {
          // skipped over files
          image1 = image_last;
          descriptors1 = descriptors_last;
          // delete the skipped files
          for (int k = i + 1; k < j - 1; k++) {
            if (std::remove(argv[k])) {
              LOG(WARNING) << "Couldn't delete " << argv[k] << ".";
            }
            deleted_count++;
          }
          i = j - 2;
        }
        break;
      }
    }
  }

  printf("Deleted %d / %d files.\n", deleted_count, argc - 1);

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
