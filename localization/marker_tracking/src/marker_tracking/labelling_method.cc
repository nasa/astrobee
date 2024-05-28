/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#include <marker_tracking/labelling_method.h>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <ar_track_alvar/Line.h>

marker_tracking::Labeling::Labeling(camera::CameraParameters const& cam) :
  bw_(nullptr), cam_(cam) {
  thresh_param1_ = 31;
  thresh_param2_ = 5;
}

marker_tracking::Labeling::~Labeling() {
}

bool marker_tracking::Labeling::CheckBorder(std::vector<cv::Point> contour, int width, int height) {
  for (int i = 0; i < contour.size(); ++i) {
    cv::Point pt = contour[i];
    if ((pt.x <= 1) || (pt.x >= width-2) || (pt.y <= 1) || (pt.y >= height-2))
      return false;
  }
  return true;
}

void marker_tracking::Labeling::SetThreshParams(int param1, int param2) {
  thresh_param1_ = param1;
  thresh_param2_ = param2;
}

marker_tracking::LabelingCvSeq::LabelingCvSeq(camera::CameraParameters const& cam) :
  Labeling(cam), n_blobs_(0), min_edge_(20), min_area_(25) {
}

marker_tracking::LabelingCvSeq::~LabelingCvSeq() {
}

void marker_tracking::LabelingCvSeq::LabelSquares(std::shared_ptr<cv::Mat> image) {
  if (bw_ == NULL) bw_.reset(new cv::Mat(image->cols, image->rows, CV_8UC1));

  // Convert grayscale and threshold
  assert(image->channels() == 1);

  cv::adaptiveThreshold(*image, *bw_, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, thresh_param1_,
                        thresh_param2_);

  std::vector< std::vector<cv::Point> > squares;
  std::vector< std::vector<cv::Point> > square_contours;

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(*bw_, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

  for (size_t i = 0; i < contours.size(); ++i) {
    if (contours[i].size() < min_edge_) {
      continue;
    }
    std::vector<cv::Point> result;
    bool is_closed = true;
    bool oriented = false;
    cv::approxPolyDP(contours[i], result, cv::arcLength(contours[i], is_closed) * 0.035, is_closed);

    if ( result.size() == 4 && CheckBorder(result, image->cols, image->rows) &&
         fabs(cv::contourArea(result, oriented)) > min_area_ &&
         cv::isContourConvex(result) ) {
      squares.push_back(result);
      square_contours.push_back(contours[i]);
    }
  }  // for loop

  n_blobs_ = squares.size();
  blob_corners.resize(n_blobs_);

  // For every detected 4-corner blob
  for (int i = 0; i < n_blobs_; ++i) {
    std::vector<alvar::Line> fitted_lines(4);
    blob_corners[i].resize(4);
    std::vector<cv::Point> sq = squares[i];
    std::vector<cv::Point> square_contour = square_contours[i];

    for (int j = 0; j < 4; ++j) {
      cv::Point pt0 = sq[j];
      cv::Point pt1 = sq[(j+1)%4];
      int k0 = -1, k1 = -1;
      for (int k = 0; k < square_contour.size(); k++) {
        cv::Point pt2 = square_contour[k];
        if ((pt0.x == pt2.x) && (pt0.y == pt2.y)) k0 = k;
        if ((pt1.x == pt2.x) && (pt1.y == pt2.y)) k1 = k;
      }
      int len;
      if (k1 >= k0)
        len = k1 - k0 - 1;  // neither k0 nor k1 are included
      else
        len = square_contour.size() - k0 + k1 - 1;
      if (len == 0) len = 1;

      cv::Mat line_data(1, len, CV_32FC2);
      for (int l = 0; l < len; l++) {
        int ll = (k0 + l + 1) % square_contour.size();
        cv::Point p = square_contour[ll];
        Eigen::Vector2d undistorted;
        cam_.Convert<camera::DISTORTED, camera::UNDISTORTED>(Eigen::Vector2d(p.x, p.y), &undistorted);
        cv::Point2f pp;
        pp.x = undistorted[0];
        pp.y = undistorted[1];

        line_data.at<cv::Point2f>(0, l) = pp;
      }

      // Fit edge and put to vector of edges
      cv::Vec4f params;
      cv::fitLine(line_data, params, cv::DIST_L2, 0, 0.01, 0.01);
      fitted_lines[j] = alvar::Line(params);
    }

    // Calculated four intersection points
    for (size_t j = 0; j < 4; ++j) {
      alvar::PointDouble intc = alvar::Intersection(fitted_lines[j], fitted_lines[(j + 1) % 4]);

      Eigen::Vector2d distorted;
      cam_.Convert<camera::UNDISTORTED, camera::DISTORTED>(Eigen::Vector2d(intc.x, intc.y), &distorted);
      intc.x = distorted[0];
      intc.y = distorted[1];

      // Should we make this always counter-clockwise or clockwise?
      /*
        if (image->origin && j == 1) blob_corners[i][3] = intc;
        else if (image->origin && j == 3) blob_corners[i][1] = intc;
        else blob_corners[i][j] = intc;
      */
      blob_corners[i][j] = intc;
    }
  }
}
