/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SURFACE_NORMAL2_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SURFACE_NORMAL2_H_

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/point_cloud.h>

namespace pcl {
namespace registration {
// TODO(rsoussan): This fixes a bug in PCL for setSourceNormals, where PCL hardcodes the normal type to pcl::Normal
// when it should use NormalT.  Switch back to PCL implementation when this is fixed. Additionally this should be
// either fixed in setTargetNormals or requiresTargetNormals should return false (ideally the second, and
// requires*Points should also be false).
/**
 * @b CorrespondenceRejectorSurfaceNormal2 implements a simple correspondence
 * rejection method based on the angle between the normals at correspondent points.
 *
 * \note If \ref setInputCloud and \ref setInputTarget are given, then the
 * distances between correspondences will be estimated using the given XYZ
 * data, and not read from the set of input correspondences.
 *
 * \author Aravindhan K Krishnan (original code from libpointmatcher: https://github.com/ethz-asl/libpointmatcher)
 * \ingroup registration
 */
class PCL_EXPORTS CorrespondenceRejectorSurfaceNormal2 : public CorrespondenceRejector {
  using CorrespondenceRejector::getClassName;
  using CorrespondenceRejector::input_correspondences_;
  using CorrespondenceRejector::rejection_name_;

 public:
  typedef boost::shared_ptr<CorrespondenceRejectorSurfaceNormal2> Ptr;
  typedef boost::shared_ptr<const CorrespondenceRejectorSurfaceNormal2> ConstPtr;

  /** \brief Empty constructor. Sets the threshold to 1.0. */
  CorrespondenceRejectorSurfaceNormal2() : threshold_(1.0), data_container_() {
    rejection_name_ = "CorrespondenceRejectorSurfaceNormal2";
  }

  /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
   * \param[in] original_correspondences the set of initial correspondences given
   * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
   */
  // void
  // getRemainingCorrespondences (const pcl::Correspondences& original_correspondences,
  //                            pcl::Correspondences& remaining_correspondences);

  /** \brief Set the thresholding angle between the normals for correspondence rejection.
   * \param[in] threshold cosine of the thresholding angle between the normals for rejection
   */
  inline void setThreshold(double threshold) { threshold_ = threshold; };

  /** \brief Get the thresholding angle between the normals for correspondence rejection. */
  inline double getThreshold() const { return threshold_; };

  void getRemainingCorrespondences(const pcl::Correspondences& original_correspondences,
                                   pcl::Correspondences& remaining_correspondences) {
    if (!data_container_) {
      PCL_ERROR("[pcl::registratin::%s::getRemainingCorrespondences] DataContainer object is not initialized!\n",
                getClassName().c_str());
      return;
    }

    unsigned int number_valid_correspondences = 0;
    remaining_correspondences.resize(original_correspondences.size());

    // Test each correspondence
    for (size_t i = 0; i < original_correspondences.size(); ++i) {
      // TODO(rsoussan): Abs val?
      if (std::abs(boost::static_pointer_cast<DataContainer<pcl::PointXYZI, pcl::PointXYZINormal> >(data_container_)
                     ->getCorrespondenceScoreFromNormals(original_correspondences[i])) > threshold_) {
        remaining_correspondences[number_valid_correspondences++] = original_correspondences[i];
      }
    }
    remaining_correspondences.resize(number_valid_correspondences);
  }

  /** \brief Initialize the data container object for the point type and the normal type. */
  template <typename PointT, typename NormalT>
  inline void initializeDataContainer() {
    data_container_.reset(new DataContainer<PointT, NormalT>);
  }

  /** \brief Provide a source point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
   * \param[in] input a cloud containing XYZ data
   */
  template <typename PointT>
  inline void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& input) {
    PCL_WARN("[pcl::registration::%s::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.\n",
             getClassName().c_str());
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::setInputCloud] Initialize the data container object by calling intializeDataContainer "
        "() before using this function.\n",
        getClassName().c_str());
      return;
    }
    boost::static_pointer_cast<DataContainer<PointT> >(data_container_)->setInputSource(input);
  }

  /** \brief Provide a source point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
   * \param[in] input a cloud containing XYZ data
   */
  template <typename PointT>
  inline void setInputSource(const typename pcl::PointCloud<PointT>::ConstPtr& input) {
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::setInputCloud] Initialize the data container object by calling intializeDataContainer "
        "() before using this function.\n",
        getClassName().c_str());
      return;
    }
    boost::static_pointer_cast<DataContainer<PointT> >(data_container_)->setInputSource(input);
  }

  /** \brief Get the target input point cloud */
  template <typename PointT>
  inline typename pcl::PointCloud<PointT>::ConstPtr getInputSource() const {
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::getInputSource] Initialize the data container object by calling "
        "intializeDataContainer () before using this function.\n",
        getClassName().c_str());
      return;
    }
    return (boost::static_pointer_cast<DataContainer<PointT> >(data_container_)->getInputSource());
  }

  /** \brief Provide a target point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
   * \param[in] target a cloud containing XYZ data
   */
  template <typename PointT>
  inline void setInputTarget(const typename pcl::PointCloud<PointT>::ConstPtr& target) {
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::setInputTarget] Initialize the data container object by calling "
        "intializeDataContainer () before using this function.\n",
        getClassName().c_str());
      return;
    }
    boost::static_pointer_cast<DataContainer<PointT> >(data_container_)->setInputTarget(target);
  }

  /** \brief Provide a pointer to the search object used to find correspondences in
   * the target cloud.
   * \param[in] tree a pointer to the spatial search object.
   * \param[in] force_no_recompute If set to true, this tree will NEVER be
   * recomputed, regardless of calls to setInputTarget. Only use if you are
   * confident that the tree will be set correctly.
   */
  template <typename PointT>
  inline void setSearchMethodTarget(const boost::shared_ptr<pcl::search::KdTree<PointT> >& tree,
                                    bool force_no_recompute = false) {
    boost::static_pointer_cast<DataContainer<PointT> >(data_container_)
      ->setSearchMethodTarget(tree, force_no_recompute);
  }

  /** \brief Get the target input point cloud */
  template <typename PointT>
  inline typename pcl::PointCloud<PointT>::ConstPtr getInputTarget() const {
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::getInputTarget] Initialize the data container object by calling "
        "intializeDataContainer () before using this function.\n",
        getClassName().c_str());
      return;
    }
    return (boost::static_pointer_cast<DataContainer<PointT> >(data_container_)->getInputTarget());
  }

  /** \brief Set the normals computed on the input point cloud
   * \param[in] normals the normals computed for the input cloud
   */
  template <typename PointT, typename NormalT>
  inline void setInputNormals(const typename pcl::PointCloud<NormalT>::ConstPtr& normals) {
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::setInputNormals] Initialize the data container object by calling "
        "intializeDataContainer () before using this function.\n",
        getClassName().c_str());
      return;
    }
    boost::static_pointer_cast<DataContainer<PointT, NormalT> >(data_container_)->setInputNormals(normals);
  }

  /** \brief Get the normals computed on the input point cloud */
  template <typename NormalT>
  inline typename pcl::PointCloud<NormalT>::Ptr getInputNormals() const {
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::getInputNormals] Initialize the data container object by calling "
        "intializeDataContainer () before using this function.\n",
        getClassName().c_str());
      return;
    }
    return (boost::static_pointer_cast<DataContainer<pcl::PointXYZI, NormalT> >(data_container_)->getInputNormals());
  }

  /** \brief Set the normals computed on the target point cloud
   * \param[in] normals the normals computed for the input cloud
   */
  template <typename PointT, typename NormalT>
  inline void setTargetNormals(const typename pcl::PointCloud<NormalT>::ConstPtr& normals) {
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::setTargetNormals] Initialize the data container object by calling "
        "intializeDataContainer () before using this function.\n",
        getClassName().c_str());
      return;
    }
    boost::static_pointer_cast<DataContainer<PointT, NormalT> >(data_container_)->setTargetNormals(normals);
  }

  /** \brief Get the normals computed on the target point cloud */
  template <typename NormalT>
  inline typename pcl::PointCloud<NormalT>::Ptr getTargetNormals() const {
    if (!data_container_) {
      PCL_ERROR(
        "[pcl::registration::%s::getTargetNormals] Initialize the data container object by calling "
        "intializeDataContainer () before using this function.\n",
        getClassName().c_str());
      return;
    }
    return (boost::static_pointer_cast<DataContainer<pcl::PointXYZI, NormalT> >(data_container_)->getTargetNormals());
  }

  /** \brief See if this rejector requires source points */
  bool requiresSourcePoints() const { return (false); }

  /** \brief Blob method for setting the source cloud */
  void setSourcePoints(pcl::PCLPointCloud2::ConstPtr cloud2) {
    if (!data_container_) initializeDataContainer<PointXYZI, Normal>();
    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
    fromPCLPointCloud2(*cloud2, *cloud);
    setInputSource<PointXYZI>(cloud);
  }

  /** \brief See if this rejector requires a target cloud */
  bool requiresTargetPoints() const { return (false); }

  /** \brief Method for setting the target cloud */
  void setTargetPoints(pcl::PCLPointCloud2::ConstPtr cloud2) {
    if (!data_container_) initializeDataContainer<PointXYZI, Normal>();
    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
    fromPCLPointCloud2(*cloud2, *cloud);
    setInputTarget<PointXYZI>(cloud);
  }

  /** \brief See if this rejector requires source normals */
  bool requiresSourceNormals() const { return (true); }

  /** \brief Blob method for setting the source normals */
  void setSourceNormals(pcl::PCLPointCloud2::ConstPtr cloud2) {
    if (!data_container_) initializeDataContainer<PointXYZI, pcl::PointXYZINormal>();
    PointCloud<pcl::PointXYZINormal>::Ptr cloud(new PointCloud<pcl::PointXYZINormal>);
    fromPCLPointCloud2(*cloud2, *cloud);
    setInputNormals<PointXYZI, pcl::PointXYZINormal>(cloud);
  }

  /** \brief See if this rejector requires target normals*/
  bool requiresTargetNormals() const { return (false); }

  /** \brief Method for setting the target normals */
  void setTargetNormals(pcl::PCLPointCloud2::ConstPtr cloud2) {
    if (!data_container_) initializeDataContainer<PointXYZI, pcl::PointXYZINormal>();
    PointCloud<pcl::PointXYZINormal>::Ptr cloud(new PointCloud<pcl::PointXYZINormal>);
    fromPCLPointCloud2(*cloud2, *cloud);
    setTargetNormals<PointXYZI, pcl::PointXYZINormal>(cloud);
  }

 protected:
  /** \brief Apply the rejection algorithm.
   * \param[out] correspondences the set of resultant correspondences.
   */
  inline void applyRejection(pcl::Correspondences& correspondences) {
    getRemainingCorrespondences(*input_correspondences_, correspondences);
  }

  /** \brief The median distance threshold between two correspondent points in source <-> target. */
  double threshold_;

  typedef boost::shared_ptr<DataContainerInterface> DataContainerPtr;
  /** \brief A pointer to the DataContainer object containing the input and target point clouds */
  DataContainerPtr data_container_;
};
}  // namespace registration
}  // namespace pcl

#endif
