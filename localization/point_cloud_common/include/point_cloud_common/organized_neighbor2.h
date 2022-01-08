/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef POINT_CLOUD_COMMON_ORGANIZED_NEIGHBOR2_H_
#define POINT_CLOUD_COMMON_ORGANIZED_NEIGHBOR2_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/common/eigen.h>

#include <algorithm>
#include <queue>
#include <vector>

// Modified version of pcl organized neighbor to enable preseting the camera intrinsics matrix.
namespace pcl {
namespace search {
/** \brief OrganizedNeighbor is a class for optimized nearest neigbhor search in organized point clouds.
 * \author Radu B. Rusu, Julius Kammerl, Suat Gedikli, Koen Buys
 * \ingroup search
 */
template <typename PointT>
class OrganizedNeighbor2 : public pcl::search::Search<PointT> {
 public:
  // public typedefs
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef boost::shared_ptr<PointCloud> PointCloudPtr;

  typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
  typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

  typedef boost::shared_ptr<pcl::search::OrganizedNeighbor2<PointT> > Ptr;
  typedef boost::shared_ptr<const pcl::search::OrganizedNeighbor2<PointT> > ConstPtr;

  using pcl::search::Search<PointT>::indices_;
  using pcl::search::Search<PointT>::sorted_results_;
  using pcl::search::Search<PointT>::input_;

  /** \brief Constructor
   * \param[in] sorted_results whether the results should be return sorted in ascending order on the distances or not.
   *        This applies only for radius search, since knn always returns sorted resutls
   */
  explicit OrganizedNeighbor2(bool sorted_results = false)
      : Search<PointT>("OrganizedNeighbor2", sorted_results), mask_() {}

  /** \brief Empty deconstructor. */
  virtual ~OrganizedNeighbor2() {}

  /** \brief Provide a pointer to the input data set, if user has focal length he must set it before calling this
   * \param[in] cloud the const boost shared pointer to a PointCloud message
   * \param[in] indices the const boost shared pointer to PointIndices
   */
  virtual void setInputCloud(const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = IndicesConstPtr()) {
    input_ = cloud;

    mask_.resize(input_->size());
    input_ = cloud;
    indices_ = indices;

    if (indices_.get() != NULL && indices_->size() != 0) {
      mask_.assign(input_->size(), 0);
      for (std::vector<int>::const_iterator iIt = indices_->begin(); iIt != indices_->end(); ++iIt) mask_[*iIt] = 1;
    } else {
      mask_.assign(input_->size(), 1);
    }
  }

  /** \brief Search for all neighbors of query point that are within a given radius.
   * \param[in] p_q the given query point
   * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
   * \param[out] k_indices the resultant indices of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
   * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
   * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
   * returned.
   * \return number of neighbors found in radius
   */
  int radiusSearch(const PointT& p_q, double radius, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances,
                   unsigned int max_nn = 0) const;

  /** \brief Search for the k-nearest neighbors for a given query point.
   * \note limiting the maximum search radius (with setMaxDistance) can lead to a significant improvement in search
   * speed \param[in] p_q the given query point (\ref setInputCloud must be given a-priori!) \param[in] k the number of
   * neighbors to search for (used only if horizontal and vertical window not given already!) \param[out] k_indices the
   * resultant point indices (must be resized to \a k beforehand!) \param[out] k_sqr_distances \note this function does
   * not return distances \return number of neighbors found
   * @todo still need to implements this functionality
   */
  int nearestKSearch(const PointT& p_q, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances) const;

  /** \brief projects a point into the image
   * \param[in] p point in 3D World Coordinate Frame to be projected onto the image plane
   * @return The 2D projected point in pixel coordinates (u,v)
   */
  Eigen::Vector2f projectPoint(const Eigen::Vector3f& p) const;

  void setIntrinsicsMatrix(const Eigen::Matrix3d& intrinsics_matrix) {
    intrinsics_matrix_ = intrinsics_matrix.cast<float>();
    KR_KRT_ = intrinsics_matrix_ * intrinsics_matrix_.transpose();
  }

 protected:
  struct Entry {
    Entry(int idx, float dist) : index(idx), distance(dist) {}
    Entry() : index(0), distance(0) {}
    unsigned index;
    float distance;

    inline bool operator<(const Entry& other) const { return (distance < other.distance); }
  };

  /** \brief test if point given by index is among the k NN in results to the query point.
   * \param[in] query query point
   * \param[in] k number of maximum nn interested in
   * \param[in] queue priority queue with k NN
   * \param[in] index index on point to be tested
   * \return wheter the top element changed or not.
   */
  inline bool testPoint(const PointT& query, unsigned k, std::priority_queue<Entry>& queue, unsigned index) const {
    const PointT& point = input_->points[index];
    if (mask_[index] && pcl_isfinite(point.x)) {
      // float squared_distance = (point.getVector3fMap () - query.getVector3fMap ()).squaredNorm ();
      float dist_x = point.x - query.x;
      float dist_y = point.y - query.y;
      float dist_z = point.z - query.z;
      float squared_distance = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
      if (queue.size() < k) {
        queue.push(Entry(index, squared_distance));
      } else if (queue.top().distance > squared_distance) {
        queue.pop();
        queue.push(Entry(index, squared_distance));
        return true;  // top element has changed!
      }
    }
    return false;
  }

  inline void clipRange(int& begin, int& end, int min, int max) const {
    begin = std::max(std::min(begin, max), min);
    end = std::min(std::max(end, min), max);
  }

  /** \brief Obtain a search box in 2D from a sphere with a radius in 3D
   * \param[in] point the query point (sphere center)
   * \param[in] squared_radius the squared sphere radius
   * \param[out] minX the min X box coordinate
   * \param[out] minY the min Y box coordinate
   * \param[out] maxX the max X box coordinate
   * \param[out] maxY the max Y box coordinate
   */
  void getProjectedRadiusSearchBox(const PointT& point, float squared_radius, unsigned& minX, unsigned& minY,
                                   unsigned& maxX, unsigned& maxY) const;

  /** \brief the intrinsics matrix. */
  Eigen::Matrix3f intrinsics_matrix_;

  Eigen::Matrix3f KR_KRT_;

  /** \brief mask, indicating whether the point was in the indices list or not.*/
  std::vector<unsigned char> mask_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace search
}  // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <point_cloud_common/organized_neighbor2_impl.h>
#endif

#endif  // POINT_CLOUD_COMMON_ORGANIZED_NEIGHBOR2_H_
