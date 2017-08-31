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

#ifndef MARKER_TRACKING_LABELLING_METHOD_H_
#define MARKER_TRACKING_LABELLING_METHOD_H_

/**
 * This is a modification of the ALVAR method to use our camera object and
 * hopefully modified to use less memory
 */

#include <camera/camera_params.h>

#include <opencv2/core/core_c.h>
#include <alvar/Alvar.h>
#include <alvar/Util.h>

#include <vector>

namespace marker_tracking {

/**
 * \brief Base class for labeling connected components from binary image.
 */
class ALVAR_EXPORT Labeling {
 protected :
  /**
   * \brief Pointer to binary image that is then labeled.
   */
  IplImage *bw_;

  camera::CameraParameters cam_;
  int thresh_param1_, thresh_param2_;

 public :

  /**
   * \brief Vector of 4-length vectors where the corners of detected blobs are stored.
   */
  std::vector<std::vector<alvar::PointDouble> > blob_corners;


  /** Constructor */
  explicit Labeling(camera::CameraParameters const& cam);

  /** Destructor*/
  virtual ~Labeling();

  /**
   * \brief Labels image and filters blobs to obtain square-shaped objects from the scene.
   */
  virtual void LabelSquares(IplImage* image) = 0;

  bool CheckBorder(CvSeq* contour, int width, int height);

  void SetThreshParams(int param1, int param2);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

/**
 * \brief Labeling class that uses OpenCV routines to find connected components.
 */
class ALVAR_EXPORT LabelingCvSeq : public Labeling {
 protected :
  int n_blobs_;
  int min_edge_;
  int min_area_;
  CvMemStorage* storage_;

 public:
  explicit LabelingCvSeq(camera::CameraParameters const& cam);
  virtual ~LabelingCvSeq();

  void LabelSquares(IplImage* image);
};

}  // namespace marker_tracking

#endif  // MARKER_TRACKING_LABELLING_METHOD_H_
