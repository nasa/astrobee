/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2010-2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Open Source Robotics Foundation, Inc. nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MAPPER_PCL_CONVERSIONS_H_
#define MAPPER_PCL_CONVERSIONS_H_

#include <pcl/point_cloud.h>
#include <vector>

// This class is extracted from:
//   https://github.com/ros-perception/pcl_conversions
// In our project we use it to convert from PointCloud2 to pcl
// The reason we copied this library here was to avoid adding all
// dependencies in the original pcl_conversions
namespace pcl_conversions {

inline
void ToPCL(const ros::Time &stamp, pcl::uint64_t *pcl_stamp) {
  *pcl_stamp = stamp.toNSec() / 1000ull;  // Convert from ns to us
}

inline
void ToPCL(const std_msgs::Header &header, pcl::PCLHeader *pcl_header) {
  ToPCL(header.stamp, &pcl_header->stamp);
  pcl_header->seq = header.seq;
  pcl_header->frame_id = header.frame_id;
}

inline
void ToPCL(const sensor_msgs::PointField &pf, pcl::PCLPointField *pcl_pf) {
  pcl_pf->name = pf.name;
  pcl_pf->offset = pf.offset;
  pcl_pf->datatype = pf.datatype;
  pcl_pf->count = pf.count;
}

inline
void ToPCL(const std::vector<sensor_msgs::PointField> &pfs, std::vector<pcl::PCLPointField> *pcl_pfs) {
  pcl_pfs->resize(pfs.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = pfs.begin();
  int i = 0;
  for (; it != pfs.end(); ++it, ++i)
    ToPCL(*(it), &(*pcl_pfs)[i]);
}

inline
void CopyPointCloud2MetaData(const sensor_msgs::PointCloud2 &pc2, pcl::PCLPointCloud2 *pcl_pc2) {
  ToPCL(pc2.header, &pcl_pc2->header);
  pcl_pc2->height = pc2.height;
  pcl_pc2->width = pc2.width;
  ToPCL(pc2.fields, &pcl_pc2->fields);
  pcl_pc2->is_bigendian = pc2.is_bigendian;
  pcl_pc2->point_step = pc2.point_step;
  pcl_pc2->row_step = pc2.row_step;
  pcl_pc2->is_dense = pc2.is_dense;
}

inline
void ToPCL(const sensor_msgs::PointCloud2 &pc2, pcl::PCLPointCloud2 *pcl_pc2) {
  CopyPointCloud2MetaData(pc2, pcl_pc2);
  pcl_pc2->data = pc2.data;
}

template<typename T>
void FromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> *pcl_cloud) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::ToPCL(cloud, &pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
}

}  // namespace pcl_conversions

#endif  // MAPPER_PCL_CONVERSIONS_H_
