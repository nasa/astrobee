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
#include <camera/camera_model.h>
#include <sparse_mapping/reprojection.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>
#include <camera/camera_params.h>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/vizcore.hpp>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>

DEFINE_bool(jump_to_3d_view, false,
            "Skip displaying images and features, go directly to the 3D view.");
DEFINE_bool(skip_3d_points, false,
            "Show just the cameras in 3D space, without the triangulated points.");
DEFINE_bool(only_3d_points, false,
            "Show just the the triangulated points.");
DEFINE_bool(skip_3d_images, false,
            "Show just the camera wireframes instead of the camera images in the 3D view.");
DEFINE_bool(fit_view_to_points_bdbox, false,
            "Zoom just enough to see all points in the cloud.");
DEFINE_bool(plot_matches, false,
            "For each feature, also plot (in red) its match in the next image.");

DEFINE_int32(first, 0,
             "The index of the first image to plot in the 3D view.");
DEFINE_int32(last, std::numeric_limits<int>::max(),
             "The index after the last image to plot in the 3D view.");
DEFINE_double(scale, 0.5,
              "The scale of images in the 3D view.");

cv::Affine3f EigenToCVAffine(const Eigen::Affine3d & e) {
  Eigen::Matrix3f r_e = e.rotation().cast<float>();
  Eigen::Vector3f t_e = e.translation().cast<float>();
  cv::Mat r, t;
  cv::eigen2cv(r_e, r);
  cv::eigen2cv(t_e, t);
  return cv::Affine3f(r, t);
}

void Draw3DFrame(cv::viz::Viz3d* window, const sparse_mapping::SparseMap & map,
                 int cid, cv::Vec3f const& center) {
  std::ostringstream str;

  // Convert to an affine, perhaps change the origin
  cv::Affine3f camera_pose = EigenToCVAffine(map.GetFrameGlobalTransform(cid).inverse());
  camera_pose.translation(camera_pose.translation() - center);

  // Load up the image
  cv::Mat image = cv::imread(map.GetFrameFilename(cid), CV_LOAD_IMAGE_GRAYSCALE);
  double f = map.GetCameraParameters().GetFocalLength();

  if (!FLAGS_only_3d_points) {
    if (FLAGS_skip_3d_images) {
      window->showWidget(std::string("frame") + std::to_string(cid),
                         cv::viz::WCameraPosition(cv::Vec2f(2*atan(image.cols * 0.5 / f),
                             2*atan(image.rows * 0.5 / f)),
                             FLAGS_scale), camera_pose);
    } else {
      window->showWidget(std::string("frame") + std::to_string(cid),
                         cv::viz::WCameraPosition(cv::Vec2f(2*atan(image.cols * 0.5 / f),
                             2*atan(image.rows * 0.5 / f)),
                             image, FLAGS_scale), camera_pose);
    }
  }
}

typedef struct {
  cv::viz::Viz3d* window;
  sparse_mapping::SparseMap* map;
  int cur_frame;
  char** frames;
  int num_frames;
  cv::Affine3f old_pose;
  int num_arrows;
  int num_screenshots;
} MapViewerState;

bool LocalizeFrame(MapViewerState* state, int frame) {
  if (frame >= 0) {
    cv::Affine3f camera_pose;
    std::vector<Eigen::Vector3d> landmarks;
    cv::Mat image;
    // display localized images
    if (state->num_frames > 0) {
      camera::CameraModel camera(Eigen::Vector3d(), Eigen::Matrix3d::Identity(),
                                 state->map->GetCameraParameters());
      image = cv::imread(state->frames[frame], CV_LOAD_IMAGE_GRAYSCALE);
      if (!state->map->Localize(state->frames[frame], &camera, &landmarks)) {
        LOG(ERROR) << "Failed to localize image.";
        state->window->showWidget("localize_pose", cv::viz::WText("", cv::Point2f(0, 0)));
        state->window->showWidget("localize_image", cv::viz::WText("", cv::Point2f(0, 0)));
        state->window->showWidget("frame_cloud", cv::viz::WText("Localization failed.", cv::Point2f(0, 0)));
        return true;
      }

      camera_pose = EigenToCVAffine(camera.GetTransform().inverse());
    } else {  // display map
      image = cv::imread(state->map->GetFrameFilename(frame), CV_LOAD_IMAGE_GRAYSCALE);
      camera_pose = EigenToCVAffine(state->map->GetFrameGlobalTransform(frame).inverse());
      std::map<int, int> fid_to_pid = state->map->GetFrameFidToPidMap(frame);
      for (std::map<int, int>::iterator it = fid_to_pid.begin(); it != fid_to_pid.end(); it++)
        landmarks.push_back(state->map->GetLandmarkPosition(it->second));
    }

    // update the viewer pose with the new frame
    if (state->cur_frame >= 0) {
      cv::Affine3f cur_pose = state->window->getViewerPose();
      cv::Affine3f new_pose(cur_pose.rvec(),  // + (camera_pose.rvec() - state->old_pose.rvec()),
                            cur_pose.translation() +
                            (camera_pose.translation() - state->old_pose.translation()));
      state->window->setViewerPose(new_pose);  // state->window->getViewerPose().concatenate(
      // state->old_pose.inv()).concatenate(camera_pose));
    }
    double f = state->map->GetCameraParameters().GetFocalLength();
    cv::viz::WCameraPosition im_pose(cv::Vec2f(2 * atan(image.cols * 0.5 / f), 2 * atan(image.rows * 0.5 / f)),
                                     image, 0.5, cv::viz::Color::red());
    im_pose.setRenderingProperty(cv::viz::LINE_WIDTH, 6.0);
    state->window->showWidget("localize_pose", im_pose, camera_pose);
    int window_w = state->window->getWindowSize().width;
    int window_h = state->window->getWindowSize().height;
    int image_w = std::min(image.cols, window_w / 4);
    int image_h = std::min(image.rows, std::min(window_h / 4,
                           static_cast<int>(static_cast<float>(image_w) / image.cols * image.rows)));
    cv::viz::WImageOverlay im_overlay(image, cv::Rect(window_w - image_w, window_h - image_h, image_w, image_h));
    state->window->showWidget("localize_image", im_overlay);
    state->old_pose = camera_pose;

    // draw point cloud for this frame
    cv::Mat cloud(1, landmarks.size(), CV_32FC3);
    cv::Point3f camera_translation = camera_pose.translation();
    if (landmarks.size() > 0) {  // opencv crashes with zero point point cloud...
      cv::Point3f* data = cloud.ptr<cv::Point3f>();
      for (unsigned int i = 0; i < landmarks.size(); i++) {
        Eigen::Map<Eigen::Vector3f> dest(reinterpret_cast<float*>(data));
        dest = landmarks[i].cast<float>();
        if (dest.norm() > 1e4) {
          dest.setZero();
        } else {
          cv::viz::WArrow arrow(camera_translation, data[0], 0.0003, cv::viz::Color::green());
          state->window->showWidget("arrow" + std::to_string(state->num_arrows++), arrow);
        }
        data++;
      }
      cv::viz::WCloud point_cloud(cloud, cv::viz::Color::cherry());
      point_cloud.setRenderingProperty(cv::viz::POINT_SIZE, 10.0);
      state->window->showWidget("frame_cloud", point_cloud);
    } else {
      state->window->showWidget("frame_cloud", cv::viz::WText("No landmarks in frame.", cv::Point2f(0, 0)));
    }
  }
  state->cur_frame = frame;

  return false;
}

int remainder(int cid, int num) {
  // Return the positive remainder
  while (cid < 0) cid += num;
  cid %= num;
  return cid;
}

sparse_mapping::SparseMap * map_ptr;
cv::viz::Viz3d * win_ptr;

// Set the origin be the centroid of all camera centers, for easier rotation
void Recenter() {
  win_ptr->removeAllWidgets();

  cv::Vec3f center(0, 0, 0);
  int num_frames = (*map_ptr).GetNumFrames();
  int count = 0;
  for (int cid = FLAGS_first; cid < FLAGS_last; cid++) {
    int cid2 = remainder(cid, num_frames);
    cv::Affine3f camera_pose = EigenToCVAffine((*map_ptr).GetFrameGlobalTransform(cid2).inverse());
    center += camera_pose.translation();
    count++;
  }
  center = center/count;

  for (int cid = FLAGS_first; cid < FLAGS_last; cid++) {
    int cid2 = remainder(cid, num_frames);
    Draw3DFrame(win_ptr, *map_ptr, cid2, center);
  }
  win_ptr->resetCamera();
}

void KeyboardCallback(const cv::viz::KeyboardEvent & event, void* param) {
  MapViewerState* state = reinterpret_cast<MapViewerState*>(param);
  int next_frame = -1;
  if (event.action == cv::viz::KeyboardEvent::KEY_UP) {
    if (event.symbol == "Left" || event.symbol == "Right") {
      if (event.symbol == "Left")
        next_frame = state->cur_frame - 1;
      else
        next_frame = state->cur_frame + 1;
      if (state->num_frames > 0) {
        if (next_frame < -1)
          next_frame = state->num_frames - 1;
        if (next_frame >= state->num_frames)
          next_frame = -1;
      } else {
        if (next_frame < -1)
          next_frame = state->map->GetNumFrames() - 1;
        if (next_frame >= static_cast<int>(state->map->GetNumFrames()))
          next_frame = -1;
      }

      if (state->cur_frame >= 0) {
        state->window->removeWidget("localize_pose");
        state->window->removeWidget("localize_image");
        state->window->removeWidget("frame_cloud");
        for (int i = 0; i < state->num_arrows; i++)
          state->window->removeWidget("arrow" + std::to_string(i));
        state->num_arrows = 0;
      }
      LocalizeFrame(state, next_frame);
    }
    if (event.symbol == "s") {
      std::ostringstream ss;
      ss << std::setw(7) << std::setfill('0') << state->num_screenshots++;
      std::string filename = "shot" + ss.str() + ".png";
      state->window->saveScreenshot(filename);
    }
    if (event.symbol == "v") {
      LOG(INFO) << "Recording video.";
      for (next_frame = 0; next_frame < state->num_frames; next_frame++) {
        if (state->cur_frame >= 0) {
          state->window->removeWidget("localize_pose");
          state->window->removeWidget("localize_image");
          state->window->removeWidget("frame_cloud");
          for (int i = 0; i < state->num_arrows; i++)
            state->window->removeWidget("arrow" + std::to_string(i));
          state->num_arrows = 0;
        }
        if (!LocalizeFrame(state, next_frame)) {
          std::ostringstream ss;
          ss << std::setw(7) << std::setfill('0') << state->num_screenshots++;
          std::string filename = "shot" + ss.str() + ".png";
          state->window->saveScreenshot(filename);
        }
      }
      LOG(INFO) << "Recording complete.";
    }
    std::string poseFile = "pose.txt";
    if (event.symbol == "a") {
      // Save pose to disk
      cv::Affine3d pose = state->window->getViewerPose();
      cv:: Matx33d R = pose.rotation();
      cv:: Vec3d t   = pose.translation();
      std::ofstream pf(poseFile.c_str());
      std::cout << "Rotation is " << R << std::endl;
      std::cout << "Translation is " << t << std::endl;
      std::cout << "Saving pose to: " << poseFile << std::endl;
      for (int c = 0; c < R.cols; c++) {
        for (int r = 0; r < R.rows; r++) {
          pf << R(c, r) << std::endl;
        }
      }
      for (int c = 0; c < t.rows; c++)
        pf << t(c) << std::endl;
    }
    if (event.symbol == "b") {
      // Read pose from disk
      cv:: Matx33d R;
      cv:: Vec3d t;
      std::cout << "Reading pose from: " << poseFile << std::endl;
      std::ifstream pf(poseFile.c_str());
      for (int c = 0; c < R.cols; c++) {
        for (int r = 0; r < R.rows; r++) {
          pf >> R(c, r);
        }
      }
      for (int c = 0; c < t.rows; c++)
        pf >> t(c);
      state->window->setViewerPose(cv::Affine3d(R, t));
      std::cout << "Rotation is " << R << std::endl;
      std::cout << "Translation is " << t << std::endl;
    }

    if (event.symbol == "d") {
      // Change the origin to make it easier to do rotations
      Recenter();
    }
    if (event.symbol == "g") {
      // Move to a different chunk of the map
      int len = FLAGS_last - FLAGS_first;
      FLAGS_first += len;
      FLAGS_last += len;
      LOG(INFO) << "Plotting cameras between: " << FLAGS_first << ' ' << FLAGS_last;
      Recenter();
    }
  }
}

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " map.map [image1.jpg image2.jpg ...]" << "\n";
    return 0;
  }

  // Do a visualization of all the projected keypoints into the
  // cameras. Navigate with the arrow keys or with 'a' and 'd'. Press
  // 'c' to show the cameras and points in 3D.

  // Check if input exists
  std::ifstream f(argv[1]);
  if (!f.good()) {
    LOG(FATAL) << "File does not exist: " << argv[1];
  }

  // Load up the NVM
  std::string map_file = argv[1];
  sparse_mapping::SparseMap map(map_file);
  LOG(INFO) << "Loaded " << argv[1];
  LOG(INFO) << "\t" << map.GetNumFrames() << " cameras and "
            << map.GetNumLandmarks() << " points";

  camera::CameraParameters camera_param = map.GetCameraParameters();

  int cid = 0;
  int num_images = map.GetNumFrames();
  while (1) {
    if (FLAGS_jump_to_3d_view)
      break;

    std::string imfile = map.GetFrameFilename(cid);
    {
      std::ifstream f(imfile.c_str());
      if (!f.good()) LOG(FATAL) << "File does not exist: " << imfile << std::endl;
    }

    cv::Mat image = cv::imread(imfile);
    LOG(INFO) << "Showing CID: " << cid << " " << imfile;

    // cv::Mat area_check(image.rows, image.cols, CV_8U);
    // cv::rectangle(area_check, cv::Point2f(0, 0), cv::Point2f(area_check.cols,
    //      area_check.rows), 0, CV_FILLED);

    const Eigen::Matrix2Xd & keypoint_map = map.GetFrameKeypoints(cid);

    // Iterate through control points and draw ones that apply to this camera
    Eigen::Vector2d output;
    for (size_t pid = 0; pid < map.GetNumLandmarks(); pid++) {
      const std::map<int, int> & cid_to_fid = map.GetLandmarkCidToFidMap(pid);
      std::map<int, int>::const_iterator it = cid_to_fid.find(cid);
      if (it != cid_to_fid.end()) {
        cv::Scalar color;
        if (cid_to_fid.size() == 2) {
          // Blue
          color = cv::Scalar(255, 0, 0);
        } else if (cid_to_fid.size() == 3) {
          // Yellow
          color = cv::Scalar(0, 255, 255);
        } else {
          // Green
          color = cv::Scalar(0, 255, 0);
        }

        // Draw the Point location
        camera_param.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(keypoint_map.col(it->second), &output);
        cv::Point2f pt(output[0], output[1]);
        cv::circle(image, pt, 3, color);
        // cv::circle(area_check, pt, 25, 255, CV_FILLED);

        // Draw a segment from the current point to its match in next image
        if (FLAGS_plot_matches) {
          std::map<int, int>::const_iterator it2 = cid_to_fid.find(cid+1);
          if (it2 != cid_to_fid.end()) {
            const Eigen::Matrix2Xd & keypoint_map2 = map.GetFrameKeypoints(cid+1);
            cv::Scalar color2(0, 0, 255);  // red

            // Draw the point location
            camera_param.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(keypoint_map2.col(it2->second), &output);
            cv::Point2f pt2(output[0], output[1]);
            cv::circle(image, pt2, 3, color2);
            cv::line(image, pt, pt2, color2);
          }
        }
      }
    }


    std::string windowName = map_file + ": individual frames";
    cv::namedWindow(windowName);
    cv::imshow(windowName, image);

    // int filled_area = 0;
    // for (int i = 0; i < area_check.rows; i++)
    //   for (int j = 0; j < area_check.cols; j++)
    //     if (area_check.at<unsigned int>(i, j))
    //       filled_area++;
    // std::cout << cid << " " << ((float)filled_area / (area_check.rows * area_check.cols)) <<
    // " " << keypoint_map.cols() << "\n";
    // LOG(INFO) << "Showimg image " << map.GetFrameFilename(cid) << " with index " << cid;
    int ret = cv::waitKey(0) & 0xFFFFF;  // on some machines has bits set in front...
    if (ret == 'a' || ret == 65361 /*left arrow key*/) {
      cid = (cid + num_images - 1) % num_images;
    }
    if (ret == 'd' || ret == 65363 /*right arrow key*/) {
      cid = (cid + num_images + 1) % num_images;
    }
    if (ret == 'c') {
      // Stop this look. Go to the next step, creating the 3D view.
      break;
    }
    if (ret == 'q') {
      return 0;
    }
  }

  // Create a window
  cv::viz::Viz3d window(map_file + ": 3D view");
  window.setWindowSize(cv::Size(1280, 720));
  window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

  int num_frames = map.GetNumFrames();
  if ( FLAGS_last == std::numeric_limits<int>::max())
    FLAGS_last = num_frames;

  // Adding widgets for all the cameras
  // Draw only frames withing specified range
  cv::Vec3f center(0, 0, 0);
  for (int cid = FLAGS_first; cid < FLAGS_last; cid++) {
    int cid2 = remainder(cid, num_frames);
    Draw3DFrame(&window, map, cid2, center);
  }

  // Need these to later manipulate the gui
  map_ptr = &map;
  win_ptr = &window;

  if (!FLAGS_skip_3d_points) {
    // Add a widget to display all the points
    cv::Mat cloud(1, map.GetNumLandmarks(), CV_32FC3);
    cv::Point3f* data = cloud.ptr<cv::Point3f>();
    for (size_t pid = 0; pid < map.GetNumLandmarks(); pid++) {
      Eigen::Map<Eigen::Vector3f> dest(reinterpret_cast<float*>(data));
      dest = map.GetLandmarkPosition(pid).cast<float>();
      if (dest.norm() > 1e4) {
        dest.setZero();
      }
      data++;
    }
    window.showWidget("CLOUD", cv::viz::WCloud(cloud));
    if (!FLAGS_fit_view_to_points_bdbox)
      window.setViewerPose(EigenToCVAffine(map.GetFrameGlobalTransform(0)).inv());
  }

  MapViewerState callback_state = {&window, &map, -1, argv + 2, argc - 2};
  callback_state.num_arrows = 0;
  callback_state.num_screenshots = 0;

  window.registerKeyboardCallback(KeyboardCallback, &callback_state);

  while (!window.wasStopped()) {
    window.spinOnce();
  }

  return 0;
}
