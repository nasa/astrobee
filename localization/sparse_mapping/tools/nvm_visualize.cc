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

#include <ff_common/init.h>
#include <ff_common/utils.h>
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
#include <boost/filesystem.hpp>

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
DEFINE_bool(enable_image_deletion, false,
            "When viewing only images, so without a map, enable deleting the currently "
            "seen image with the Del key.");

DEFINE_int32(first, 0,
             "The index of the first image to plot in the 3D view.");
DEFINE_int32(last, std::numeric_limits<int>::max(),
             "The index after the last image to plot in the 3D view.");
DEFINE_double(scale, 0.5,
              "The scale of images in the 3D view.");

// Global variables
std::shared_ptr<sparse_mapping::SparseMap> g_map_ptr;
cv::viz::Viz3d * g_win_ptr;
int g_cid;
cv::Mat * g_image;
std::string * g_windowName;
std::vector<std::string> g_openWindows;

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
  cv::Mat image = cv::imread(map.GetFrameFilename(cid), cv::IMREAD_GRAYSCALE);
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
      image = cv::imread(state->frames[frame], cv::IMREAD_GRAYSCALE);
      if (!state->map->Localize(state->frames[frame], &camera, &landmarks)) {
        LOG(ERROR) << "Failed to localize image.";
        state->window->showWidget("localize_pose", cv::viz::WText("", cv::Point2f(0, 0)));
        state->window->showWidget("localize_image", cv::viz::WText("", cv::Point2f(0, 0)));
        state->window->showWidget("frame_cloud", cv::viz::WText("Localization failed.", cv::Point2f(0, 0)));
        return true;
      }

      camera_pose = EigenToCVAffine(camera.GetTransform().inverse());
    } else {  // display map
      image = cv::imread(state->map->GetFrameFilename(frame), cv::IMREAD_GRAYSCALE);
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

// Set the origin be the centroid of all camera centers, for easier rotation
void Recenter() {
  sparse_mapping::SparseMap & map = *g_map_ptr;  // make notation easy

  g_win_ptr->removeAllWidgets();

  cv::Vec3f center(0, 0, 0);
  int num_frames = map.GetNumFrames();
  int count = 0;
  for (int cid = FLAGS_first; cid < FLAGS_last; cid++) {
    int cid2 = remainder(cid, num_frames);
    cv::Affine3f camera_pose = EigenToCVAffine(map.GetFrameGlobalTransform(cid2).inverse());
    center += camera_pose.translation();
    count++;
  }
  center = center/count;

  for (int cid = FLAGS_first; cid < FLAGS_last; cid++) {
    int cid2 = remainder(cid, num_frames);
    Draw3DFrame(g_win_ptr, map, cid2, center);
  }
  g_win_ptr->resetCamera();
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

static void onMouse(int event, int x, int y, int, void*) {
  // Make notation easy
  sparse_mapping::SparseMap & map = *g_map_ptr;
  cv::Mat & image = *g_image;
  std::string & windowName = *g_windowName;
  int cid = g_cid;

  // Print the current image and the pixel clicked onto.
  // This is useful in assembling a subset of images.
  if (event == cv::EVENT_LBUTTONDOWN) {
    std::cout << "Pixel: " << x << " " << y << " in "
              << map.GetFrameFilename(cid)  << "\n";
    return;
  }

  if (event != cv::EVENT_MBUTTONDOWN)
    return;

  // Identify in the map the feature that was clicked with the middle
  // mouse button. Then list the images that have that feature and
  // show them in separate windows with the feature in each as a dot.

  // If we have some windows open already, we close them, reversing
  // the earlier call.
  if (!g_openWindows.empty()) {
    for (size_t it = 0; it < g_openWindows.size(); it++) {
      cv::destroyWindow(g_openWindows[it]);
    }
    g_openWindows.clear();
    return;
  }

  camera::CameraParameters camera_param = map.GetCameraParameters();
  const Eigen::Matrix2Xd & keypoint_map = map.GetFrameKeypoints(cid);

  // Locate that point in the image
  Eigen::Vector2d curr_pix(x, y);
  int curr_pid = -1;

  for (size_t pid = 0; pid < map.GetNumLandmarks(); pid++) {
    const std::map<int, int> & cid_to_fid = map.GetLandmarkCidToFidMap(pid);
    std::map<int, int>::const_iterator it = cid_to_fid.find(cid);
    if (it == cid_to_fid.end()) continue;

    // Assume that the desired feature is no further than this from
    // where the mouse clicked.
    double tol = 15.0;  // Making this big risks mis-identification

    Eigen::Vector2d dist_pix;
    camera_param.Convert<camera::UNDISTORTED_C, camera::DISTORTED>
      (keypoint_map.col(it->second), &dist_pix);
    if ((dist_pix - curr_pix).norm() > tol) continue;

    curr_pid = pid;

    // Plot the identified feature for verification
    cv::Scalar color = cv::Scalar(0, 0, 255);  // red
    cv::Point2f pt(dist_pix[0], dist_pix[1]);
    int radius = 3;
    cv::circle(image, pt, 1, color, radius, cv::FILLED);
    cv::imshow(windowName, image);
    break;
  }

  if (curr_pid < 0) {
    std::cout << "The pixel that was clicked on is not in the map." << std::endl;
    return;
  }

  // List and display the images having this feature with the feature
  // shown as a dot.
  std::cout << "Images having this feature: ";
  const std::map<int, int> & cid_to_fid = map.GetLandmarkCidToFidMap(curr_pid);
  for (std::map<int, int>::const_iterator it = cid_to_fid.begin();
       it != cid_to_fid.end(); it++) {
    int curr_cid = it->first;
    int curr_fid = it->second;
    std::string imfile = map.GetFrameFilename(curr_cid);
    std::cout << imfile << " ";
    {
      std::ifstream f(imfile.c_str());
      if (!f.good()) LOG(FATAL) << "File does not exist: " << imfile << std::endl;
    }

    cv::Mat curr_image = cv::imread(imfile);

    const Eigen::Matrix2Xd & curr_keypoint_map = map.GetFrameKeypoints(curr_cid);
    Eigen::Vector2d dist_pix;
    camera_param.Convert<camera::UNDISTORTED_C, camera::DISTORTED>
      (curr_keypoint_map.col(curr_fid), &dist_pix);

    cv::Scalar color = cv::Scalar(0, 0, 255);  // red
    cv::Point2f pt(dist_pix[0], dist_pix[1]);
    int radius = 3;
    cv::circle(curr_image, pt, 1, color, radius, cv::FILLED);
    std::string currWindowName = "Feature in: " + imfile;
    cv::imshow(currWindowName, curr_image);
    g_openWindows.push_back(currWindowName);
  }
  std::cout << std::endl;
}

// Form a map with images only, for the purpose of visualization
std::shared_ptr<sparse_mapping::SparseMap> map_with_no_features(int argc, char ** argv) {
  std::vector<std::string> images;
  for (int i = 1; i < argc; i++) {
    std::string image_name = argv[i];
    if (ff_common::file_extension(image_name) != "jpg")
      LOG(FATAL) << "Unsupported image: " << image_name;
    images.push_back(image_name);
  }

  camera::CameraParameters cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0),
                                      Eigen::Vector2d(0, 0));
  return std::shared_ptr<sparse_mapping::SparseMap>
    (new sparse_mapping::SparseMap(images, "SURF", cam_params));
}

// Delete an image from disk and from a map, such as when we would like to weed out similar images.
void deleteImageFromDiskAndMap(sparse_mapping::SparseMap & map, std::string const& image_name) {
  // This must not be attempted for non-empty maps.
  if (!map.pid_to_cid_fid_.empty())
    LOG(FATAL) << "Tried to delete images from a non-empty map. That is not supported.";

  std::vector<std::string> & images = map.cid_to_filename_;  // alias
  auto pos = std::find(images.begin(), images.end(), image_name);
  if (pos == images.end()) {
    std::cout << "Could not find image: " << image_name << std::endl;
  } else {
    std::cout << "Deleting from disk image: " << image_name << std::endl;
    images.erase(pos);  // this changes the map
    boost::filesystem::path image_path(image_name);
    if ( boost::filesystem::exists(image_path))
      boost::filesystem::remove(image_path);
  }
}

bool file_exists(const std::string& name) {
  std::ifstream f(name.c_str());
  return f.good();
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " [ map.map ] [image1.jpg image2.jpg ...]" << "\n";
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

  // Load up the map. Assign it to global variable g_map_ptr so that we can access
  // it in the GUI. If only images are provided, cook up a map. This is useful
  // for visualizing images.
  std::string map_file = argv[1];
  if (ff_common::file_extension(map_file) == "map") {
    g_map_ptr = std::shared_ptr<sparse_mapping::SparseMap>(new sparse_mapping::SparseMap(map_file));
  } else {
    g_map_ptr = map_with_no_features(argc, argv);
  }

  // Make notation easy
  sparse_mapping::SparseMap & map = *g_map_ptr;

  LOG(INFO) << "Loaded " << argv[1];
  LOG(INFO) << "\t" << map.GetNumFrames() << " cameras and "
            << map.GetNumLandmarks() << " points";

  camera::CameraParameters camera_param = map.GetCameraParameters();

  int cid = FLAGS_first;
  int num_images = map.GetNumFrames();
  bool windowInitalized = false;
  while (1) {
    if (FLAGS_jump_to_3d_view)
      break;

    std::string imfile = map.GetFrameFilename(cid);
    if (!file_exists(imfile))
      LOG(FATAL) << "File does not exist: " << imfile << std::endl;

    cv::Mat image = cv::imread(imfile);
    g_image = &image;  // to be able to use it in callbacks
    // Below LOG(INFO) is not used as it prints too much junk. Use cout.
    std::cout << "CID: " << cid << " " << imfile << std::endl;

    // Export this to be used on onMouse
    g_cid = cid;

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


    // Create a window that can be resized by the user
    std::string windowName = map_file + ": individual frames";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
    g_windowName = &windowName;  // Export this to be used by onMouse
    cv::setMouseCallback(windowName, onMouse, 0);
    cv::imshow(windowName, image);

    // This gets called only once to set the initial size that later the user can modify
    if (!windowInitalized) {
      cv::resizeWindow(windowName, image.cols, image.rows);
      windowInitalized = true;
    }

    int ret = cv::waitKey(0) & 0xFF;  // On some machines there are bits set in front.

    // Navigate through the images with the arrow keys.
    if (ret == 'a'  ||  // key 'a'
        ret == 0x51 ||  // left arrow key
        ret == 158      // ins on numpad (when the arrow keys fail to work)
        ) {
      cid = (cid + num_images - 1) % num_images;  // go to prev image
    }
    if (ret == 'd'  ||  // key 'd'
        ret == 0x53 ||  // right arrow key
        ret == 159      // del on numpad (when the arrow keys fail to work)
        ) {
      cid = (cid + num_images + 1) % num_images;  // go to next image
    }

    if (ret == 80) {
      cid = 0;  // The 'Home' key was pressed, go to the first image
    }

    if (ret == 87) {
      cid = num_images - 1;  // The 'End' key was pressed, go to the last image
    }

    if ((ret =='x' || ret == 255) && FLAGS_enable_image_deletion) {
      deleteImageFromDiskAndMap(map, imfile);
      num_images = map.GetNumFrames();  // update the number of images
      if (num_images == 0) {
        std::cout << "No more images left." << std::endl;
        return 0;
      }
      cid = cid % num_images;  // now it will point to next image
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

  // Need this to later manipulate the gui
  g_win_ptr = &window;

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
