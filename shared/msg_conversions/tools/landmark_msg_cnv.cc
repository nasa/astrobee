// Copyright 2015 Intelligent Robotics Group, NASA ARC

#include <ff_msgs/VisualLandmarks.h>

#include <common/init.h>
#include <common/utils.h>
#include <gflags/gflags.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

DEFINE_string(input_topic, "/loc/ml/features",
              "The input features topic to convert.");
DEFINE_string(output_topic, "/loc/rviz",
              "The output topic suitable for rviz.");

ros::Publisher landmarks_pub;

void landmarks_callback(ff_msgs::VisualLandmarksPtr const & l) {
  sensor_msgs::PointCloud2 out;
  out.header = std_msgs::Header();
  out.header.stamp = ros::Time::now();
  out.header.frame_id = "world";
  out.height = 1;
  out.width = l->landmarks.size();

  out.fields.resize(3);
  out.fields[0].name = "x";
  out.fields[0].offset = 0;
  out.fields[0].datatype = 7;
  out.fields[0].count = 1;
  out.fields[1].name = "y";
  out.fields[1].offset = 4;
  out.fields[1].datatype = 7;
  out.fields[1].count = 1;
  out.fields[2].name = "z";
  out.fields[2].offset = 8;
  out.fields[2].datatype = 7;
  out.fields[2].count = 1;

  out.is_bigendian = false;
  out.point_step = 12;
  out.row_step = out.point_step * out.width;
  out.is_dense = true;
  out.data.resize(out.row_step);

  for (unsigned int i = 0; i < l->landmarks.size(); i++) {
    memcpy(&out.data[out.point_step * i + 0], &l->landmarks[i].x, 4);
    memcpy(&out.data[out.point_step * i + 4], &l->landmarks[i].y, 4);
    memcpy(&out.data[out.point_step * i + 8], &l->landmarks[i].z, 4);
  }

  landmarks_pub.publish(out);
}

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  ros::init(argc, argv, "landmark_msg_cnv");

  ros::NodeHandle nh;

  ros::Subscriber landmarks_sub = nh.subscribe(FLAGS_input_topic, 5, &landmarks_callback);
  landmarks_pub = nh.advertise<sensor_msgs::PointCloud2>(FLAGS_output_topic, 5);
  ros::spin();

  return 0;
}
