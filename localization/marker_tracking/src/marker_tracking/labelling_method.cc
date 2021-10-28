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

#include <Eigen/Core>
#include <alvar/Line.h>

marker_tracking::Labeling::Labeling(camera::CameraParameters const& cam) :
  bw_(NULL), cam_(cam) {
  thresh_param1_ = 31;
  thresh_param2_ = 5;
}

marker_tracking::Labeling::~Labeling() {
  if (bw_)
    cvReleaseImage(&bw_);
}

bool marker_tracking::Labeling::CheckBorder(CvSeq* contour, int width, int height) {
  bool ret = true;
  for (int i = 0; i < contour->total; ++i) {
    CvPoint* pt = reinterpret_cast<CvPoint*>(cvGetSeqElem(contour, i));
    if ((pt->x <= 1) || (pt->x >= width-2) || (pt->y <= 1) || (pt->y >= height-2)) ret = false;
  }
  return ret;
}

void marker_tracking::Labeling::SetThreshParams(int param1, int param2) {
  thresh_param1_ = param1;
  thresh_param2_ = param2;
}

marker_tracking::LabelingCvSeq::LabelingCvSeq(camera::CameraParameters const& cam) :
  Labeling(cam), n_blobs_(0), min_edge_(20), min_area_(25) {
  storage_ = cvCreateMemStorage(0);
}

marker_tracking::LabelingCvSeq::~LabelingCvSeq() {
  if (storage_)
    cvReleaseMemStorage(&storage_);
}

void marker_tracking::LabelingCvSeq::LabelSquares(IplImage* image) {
  if (bw_ == NULL) {
    bw_ = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
  }

  // Convert grayscale and threshold
  assert(image->nChannels == 1);

  cvAdaptiveThreshold(image, bw_, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, thresh_param1_, thresh_param2_);

  CvSeq* contours;
  CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSeq), storage_);
  CvSeq* square_contours = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSeq), storage_);

  cvFindContours(bw_, storage_, &contours, sizeof(CvContour),
                 CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

  while (contours) {
    if (contours->total < min_edge_) {
      contours = contours->h_next;
      continue;
    }

    CvSeq* result = cvApproxPoly(contours, sizeof(CvContour), storage_,
                                 CV_POLY_APPROX_DP, cvContourPerimeter(contours) * 0.035, 0);

    if ( result->total == 4 && CheckBorder(result, image->width, image->height) &&
         fabs(cvContourArea(result, CV_WHOLE_SEQ)) > min_area_ &&
         cvCheckContourConvexity(result) ) {
      cvSeqPush(squares, result);
      cvSeqPush(square_contours, contours);
    }
    contours = contours->h_next;
  }

  n_blobs_ = squares->total;
  blob_corners.resize(n_blobs_);

  // For every detected 4-corner blob
  for (int i = 0; i < n_blobs_; ++i) {
    std::vector<alvar::Line> fitted_lines(4);
    blob_corners[i].resize(4);
    CvSeq* sq = reinterpret_cast<CvSeq*>(cvGetSeqElem(squares, i));
    CvSeq* square_contour = reinterpret_cast<CvSeq*>(cvGetSeqElem(square_contours, i));

    for (int j = 0; j < 4; ++j) {
      CvPoint* pt0 = reinterpret_cast<CvPoint*>(cvGetSeqElem(sq, j));
      CvPoint* pt1 = reinterpret_cast<CvPoint*>(cvGetSeqElem(sq, (j+1)%4));
      int k0 = -1, k1 = -1;
      for (int k = 0; k < square_contour->total; k++) {
        CvPoint* pt2 = reinterpret_cast<CvPoint*>(cvGetSeqElem(square_contour, k));
        if ((pt0->x == pt2->x) && (pt0->y == pt2->y)) k0=k;
        if ((pt1->x == pt2->x) && (pt1->y == pt2->y)) k1=k;
      }
      int len;
      if (k1 >= k0)
        len = k1 - k0 - 1;  // neither k0 nor k1 are included
      else
        len = square_contour->total - k0 + k1 - 1;
      if (len == 0) len = 1;

      CvMat* line_data = cvCreateMat(1, len, CV_32FC2);
      for (int l = 0; l < len; l++) {
        int ll = (k0 + l + 1) % square_contour->total;
        CvPoint* p = reinterpret_cast<CvPoint*>(cvGetSeqElem(square_contour, ll));
        Eigen::Vector2d undistorted;
        cam_.Convert<camera::DISTORTED, camera::UNDISTORTED>(Eigen::Vector2d(p->x, p->y), &undistorted);
        CvPoint2D32f pp;
        pp.x = undistorted[0];
        pp.y = undistorted[1];

        CV_MAT_ELEM(*line_data, CvPoint2D32f, 0, l) = pp;
      }

      // Fit edge and put to vector of edges
      float params[4] = {0};

      cvFitLine(line_data, CV_DIST_L2, 0, 0.01, 0.01, params);
      fitted_lines[j] = alvar::Line(params);

      cvReleaseMat(&line_data);
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

  cvClearMemStorage(storage_);
}
