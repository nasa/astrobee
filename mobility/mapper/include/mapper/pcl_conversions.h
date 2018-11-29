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


#include <ros/ros.h>

#include <pcl/conversions.h>

#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>

#include <pcl/PCLPointField.h>
#include <sensor_msgs/PointField.h>

#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <vector>
#include <string>

// subset of  https://github.com/ros-perception/pcl_conversion/
// copied due to reduced overhead?

namespace pcl_conversions {

/** PCLHeader <=> Header **/

inline void fromPCL(const pcl::uint64_t &pcl_stamp, ros::Time &stamp) {
  stamp.fromNSec(pcl_stamp * 1000ull);  // Convert from us to ns
}

inline void toPCL(const ros::Time &stamp, pcl::uint64_t &pcl_stamp) {
  pcl_stamp = stamp.toNSec() / 1000ull;  // Convert from ns to us
}

inline ros::Time fromPCL(const pcl::uint64_t &pcl_stamp) {
  ros::Time stamp;
  fromPCL(pcl_stamp, stamp);
  return stamp;
}

inline pcl::uint64_t toPCL(const ros::Time &stamp) {
  pcl::uint64_t pcl_stamp;
  toPCL(stamp, pcl_stamp);
  return pcl_stamp;
}

/** PCLHeader <=> Header **/

inline void fromPCL(const pcl::PCLHeader &pcl_header,
                    std_msgs::Header &header) {
  fromPCL(pcl_header.stamp, header.stamp);
  header.seq = pcl_header.seq;
  header.frame_id = pcl_header.frame_id;
}

inline void toPCL(const std_msgs::Header &header, pcl::PCLHeader &pcl_header) {
  toPCL(header.stamp, pcl_header.stamp);
  pcl_header.seq = header.seq;
  pcl_header.frame_id = header.frame_id;
}

inline std_msgs::Header fromPCL(const pcl::PCLHeader &pcl_header) {
  std_msgs::Header header;
  fromPCL(pcl_header, header);
  return header;
}

inline pcl::PCLHeader toPCL(const std_msgs::Header &header) {
  pcl::PCLHeader pcl_header;
  toPCL(header, pcl_header);
  return pcl_header;
}

/** PCLPointField <=> PointField **/

inline void fromPCL(const pcl::PCLPointField &pcl_pf,
                    sensor_msgs::PointField &pf) {
  pf.name = pcl_pf.name;
  pf.offset = pcl_pf.offset;
  pf.datatype = pcl_pf.datatype;
  pf.count = pcl_pf.count;
}

inline void fromPCL(const std::vector<pcl::PCLPointField> &pcl_pfs,
                    std::vector<sensor_msgs::PointField> &pfs) {
  pfs.resize(pcl_pfs.size());
  std::vector<pcl::PCLPointField>::const_iterator it = pcl_pfs.begin();
  int i = 0;
  for (; it != pcl_pfs.end(); ++it, ++i) {
    fromPCL(*(it), pfs[i]);
  }
}

inline void toPCL(const sensor_msgs::PointField &pf,
                  pcl::PCLPointField &pcl_pf) {
  pcl_pf.name = pf.name;
  pcl_pf.offset = pf.offset;
  pcl_pf.datatype = pf.datatype;
  pcl_pf.count = pf.count;
}

inline void toPCL(const std::vector<sensor_msgs::PointField> &pfs,
                  std::vector<pcl::PCLPointField> &pcl_pfs) {
  pcl_pfs.resize(pfs.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = pfs.begin();
  int i = 0;
  for (; it != pfs.end(); ++it, ++i) {
    toPCL(*(it), pcl_pfs[i]);
  }
}

/** PCLPointCloud2 <=> PointCloud2 **/

inline void copyPCLPointCloud2MetaData(const pcl::PCLPointCloud2 &pcl_pc2,
                                       sensor_msgs::PointCloud2 &pc2) {
  fromPCL(pcl_pc2.header, pc2.header);
  pc2.height = pcl_pc2.height;
  pc2.width = pcl_pc2.width;
  fromPCL(pcl_pc2.fields, pc2.fields);
  pc2.is_bigendian = pcl_pc2.is_bigendian;
  pc2.point_step = pcl_pc2.point_step;
  pc2.row_step = pcl_pc2.row_step;
  pc2.is_dense = pcl_pc2.is_dense;
}

inline void fromPCL(const pcl::PCLPointCloud2 &pcl_pc2,
                    sensor_msgs::PointCloud2 &pc2) {
  copyPCLPointCloud2MetaData(pcl_pc2, pc2);
  pc2.data = pcl_pc2.data;
}

inline void moveFromPCL(pcl::PCLPointCloud2 &pcl_pc2,
                        sensor_msgs::PointCloud2 &pc2) {
  copyPCLPointCloud2MetaData(pcl_pc2, pc2);
  pc2.data.swap(pcl_pc2.data);
}

inline void copyPointCloud2MetaData(const sensor_msgs::PointCloud2 &pc2,
                                    pcl::PCLPointCloud2 &pcl_pc2) {
  toPCL(pc2.header, pcl_pc2.header);
  pcl_pc2.height = pc2.height;
  pcl_pc2.width = pc2.width;
  toPCL(pc2.fields, pcl_pc2.fields);
  pcl_pc2.is_bigendian = pc2.is_bigendian;
  pcl_pc2.point_step = pc2.point_step;
  pcl_pc2.row_step = pc2.row_step;
  pcl_pc2.is_dense = pc2.is_dense;
}

inline void toPCL(const sensor_msgs::PointCloud2 &pc2,
                  pcl::PCLPointCloud2 &pcl_pc2) {
  copyPointCloud2MetaData(pc2, pcl_pc2);
  pcl_pc2.data = pc2.data;
}

inline void moveToPCL(sensor_msgs::PointCloud2 &pc2,
                      pcl::PCLPointCloud2 &pcl_pc2) {
  copyPointCloud2MetaData(pc2, pcl_pc2);
  pcl_pc2.data.swap(pc2.data);
}

}  // namespace pcl_conversions

namespace pcl {

/** Overload pcl::getFieldIndex **/

inline int getFieldIndex(const sensor_msgs::PointCloud2 &cloud,
                         const std::string &field_name) {
  // Get the index we need
  for (size_t d = 0; d < cloud.fields.size(); ++d) {
    if (cloud.fields[d].name == field_name) {
      return (static_cast<int>(d));
    }
  }
  return (-1);
}

/** Overload pcl::getFieldsList **/

inline std::string getFieldsList(const sensor_msgs::PointCloud2 &cloud) {
  std::string result;
  for (size_t i = 0; i < cloud.fields.size() - 1; ++i) {
    result += cloud.fields[i].name + " ";
  }
  result += cloud.fields[cloud.fields.size() - 1].name;
  return (result);
}

/** Provide to/fromROSMsg for sensor_msgs::PointCloud2 <=> pcl::PointCloud<T>
 * **/

template <typename T>
void toROSMsg(const pcl::PointCloud<T> &pcl_cloud,
              sensor_msgs::PointCloud2 &cloud) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  pcl_conversions::moveFromPCL(pcl_pc2, cloud);
}

template <typename T>
void fromROSMsg(const sensor_msgs::PointCloud2 &cloud,
                pcl::PointCloud<T> &pcl_cloud) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}

template <typename T>
void moveFromROSMsg(sensor_msgs::PointCloud2 &cloud,
                    pcl::PointCloud<T> &pcl_cloud) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::moveToPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}

/** Overload pcl::createMapping **/

template <typename PointT>
void createMapping(const std::vector<sensor_msgs::PointField> &msg_fields,
                   MsgFieldMap &field_map) {
  std::vector<pcl::PCLPointField> pcl_msg_fields;
  pcl_conversions::toPCL(msg_fields, pcl_msg_fields);
  return createMapping<PointT>(pcl_msg_fields, field_map);
}

}  // namespace pcl

namespace ros {
template <>
struct DefaultMessageCreator<pcl::PCLPointCloud2> {
  boost::shared_ptr<pcl::PCLPointCloud2> operator()() {
    boost::shared_ptr<pcl::PCLPointCloud2> msg(new pcl::PCLPointCloud2());
    return msg;
  }
};

namespace message_traits {
template <>
struct MD5Sum<pcl::PCLPointCloud2> {
  static const char *value() {
    return MD5Sum<sensor_msgs::PointCloud2>::value();
  }
  static const char *value(const pcl::PCLPointCloud2 &) { return value(); }

  static const uint64_t static_value1 =
      MD5Sum<sensor_msgs::PointCloud2>::static_value1;
  static const uint64_t static_value2 =
      MD5Sum<sensor_msgs::PointCloud2>::static_value2;

  // If the definition of sensor_msgs/PointCloud2 changes, we'll get a compile
  // error here.
  ROS_STATIC_ASSERT(static_value1 == 0x1158d486dd51d683ULL);
  ROS_STATIC_ASSERT(static_value2 == 0xce2f1be655c3c181ULL);
};

template <>
struct DataType<pcl::PCLPointCloud2> {
  static const char *value() {
    return DataType<sensor_msgs::PointCloud2>::value();
  }
  static const char *value(const pcl::PCLPointCloud2 &) { return value(); }
};

template <>
struct Definition<pcl::PCLPointCloud2> {
  static const char *value() {
    return Definition<sensor_msgs::PointCloud2>::value();
  }
  static const char *value(const pcl::PCLPointCloud2 &) { return value(); }
};

template <>
struct HasHeader<pcl::PCLPointCloud2> : TrueType {};
}  // namespace message_traits

namespace serialization {
/*
 * Provide a custom serialization for pcl::PCLPointCloud2
 */
template <>
struct Serializer<pcl::PCLPointCloud2> {
  template <typename Stream>
  inline static void write(Stream &stream, const pcl::PCLPointCloud2 &m) {
    std_msgs::Header header;
    pcl_conversions::fromPCL(m.header, header);
    stream.next(header);
    stream.next(m.height);
    stream.next(m.width);
    std::vector<sensor_msgs::PointField> pfs;
    pcl_conversions::fromPCL(m.fields, pfs);
    stream.next(pfs);
    stream.next(m.is_bigendian);
    stream.next(m.point_step);
    stream.next(m.row_step);
    stream.next(m.data);
    stream.next(m.is_dense);
  }

  template <typename Stream>
  inline static void read(Stream &stream, pcl::PCLPointCloud2 &m) {
    std_msgs::Header header;
    stream.next(header);
    pcl_conversions::toPCL(header, m.header);
    stream.next(m.height);
    stream.next(m.width);
    std::vector<sensor_msgs::PointField> pfs;
    stream.next(pfs);
    pcl_conversions::toPCL(pfs, m.fields);
    stream.next(m.is_bigendian);
    stream.next(m.point_step);
    stream.next(m.row_step);
    stream.next(m.data);
    stream.next(m.is_dense);
  }

  inline static uint32_t serializedLength(const pcl::PCLPointCloud2 &m) {
    uint32_t length = 0;

    std_msgs::Header header;
    pcl_conversions::fromPCL(m.header, header);
    length += serializationLength(header);
    length += 4;  // height
    length += 4;  // width
    std::vector<sensor_msgs::PointField> pfs;
    pcl_conversions::fromPCL(m.fields, pfs);
    length += serializationLength(pfs);  // fields
    length += 1;                         // is_bigendian
    length += 4;                         // point_step
    length += 4;                         // row_step
    length += 4;                         // data's size
    length += m.data.size() * sizeof(pcl::uint8_t);
    length += 1;  // is_dense

    return length;
  }
};

/*
 * Provide a custom serialization for pcl::PCLPointField
 */
template <>
struct Serializer<pcl::PCLPointField> {
  template <typename Stream>
  inline static void write(Stream &stream, const pcl::PCLPointField &m) {
    stream.next(m.name);
    stream.next(m.offset);
    stream.next(m.datatype);
    stream.next(m.count);
  }

  template <typename Stream>
  inline static void read(Stream &stream, pcl::PCLPointField &m) {
    stream.next(m.name);
    stream.next(m.offset);
    stream.next(m.datatype);
    stream.next(m.count);
  }

  inline static uint32_t serializedLength(const pcl::PCLPointField &m) {
    uint32_t length = 0;

    length += serializationLength(m.name);
    length += serializationLength(m.offset);
    length += serializationLength(m.datatype);
    length += serializationLength(m.count);

    return length;
  }
};

/*
 * Provide a custom serialization for pcl::PCLHeader
 */
template <>
struct Serializer<pcl::PCLHeader> {
  template <typename Stream>
  inline static void write(Stream &stream, const pcl::PCLHeader &m) {
    std_msgs::Header header;
    pcl_conversions::fromPCL(m, header);
    stream.next(header);
  }

  template <typename Stream>
  inline static void read(Stream &stream, pcl::PCLHeader &m) {
    std_msgs::Header header;
    stream.next(header);
    pcl_conversions::toPCL(header, m);
  }

  inline static uint32_t serializedLength(const pcl::PCLHeader &m) {
    uint32_t length = 0;

    std_msgs::Header header;
    pcl_conversions::fromPCL(m, header);
    length += serializationLength(header);

    return length;
  }
};
}  // namespace serialization

}  // namespace ros

#endif  // MAPPER_PCL_CONVERSIONS_H_
