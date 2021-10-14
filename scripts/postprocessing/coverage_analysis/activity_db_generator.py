#!/usr/bin/python

""" Creates a CSV file containing the Mapped Landmark features per robot pose published by
    the /loc/ml/features topic
"""

import sys

import numpy as np
import rospy
from ff_msgs.msg import VisualLandmarks
from tf.transformations import *


class Activity_DBGenerator:
    def __init__(self):
        print("Activity_DBGenerator started...")

        self.activity_name = ""
        self.activity_date = ""
        self.output_filename = ""
        self.fileOut = None

    def callback(self, data):
        count = len(data.landmarks)
        ts = data.header.stamp

        if count == 0:
            robot_pose_str = "nan, nan, nan"
        else:
            robot_pose = self.body_frame_from_navcam(
                data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z,
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w,
            )

            robot_pose_str = "{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format(
                robot_pose[0],
                robot_pose[1],
                robot_pose[2],
                robot_pose[3],
                robot_pose[4],
                robot_pose[5],
                robot_pose[6],
            )

            self.fileOut.write("{:d} {:s}\n".format(count, robot_pose_str))

        for i in range(count):
            ml_pose_str = "{:.3f} {:.3f} {:.3f}".format(
                data.landmarks[i].x, data.landmarks[i].y, data.landmarks[i].z
            )
            self.fileOut.write("{:s}\n".format(ml_pose_str))

    def body_frame_from_navcam(self, px, py, pz, qi, qj, qk, qw):
        # As defined in Astrobee repo: ${HOME}/astrobee/astrobee/config/robots/bumble.config
        pose_const = np.array([0.1157 + 0.002, -0.0422, -0.0826, 1.0])
        quat_const = [0.500, 0.500, 0.500, 0.500]

        rot_const = quaternion_matrix(quat_const)
        rot_const[:, 3] = pose_const.T

        rot_in = quaternion_matrix([qi, qj, qk, qw])
        rot_in[:, 3] = np.array([px, py, pz, 1.0]).T

        t_body = rot_in.dot(rot_const)

        quat_out = np.zeros((4, 4))
        quat_out[0:3, 0:3] = t_body[0:3, 0:3]
        quat_out[3, 3] = 1.0

        quat_out = quaternion_from_matrix(quat_out)
        pose_out = t_body[0:3:, 3]

        return [
            pose_out[0],
            pose_out[1],
            pose_out[2],
            quat_out[0],
            quat_out[1],
            quat_out[2],
            quat_out[3],
        ]

    def open_file(self, file, activity, map, date):
        self.fileOut = open(file, "w")
        self.fileOut.write(
            "### Date-Map-Activity: " + date + "-" + map + "-" + activity + "\n"
        )
        self.fileOut.write(
            "### ML Number Robot Pose (x,y,z,r,p,y,w)\n### ML Coordinate(x,y,z)\n"
        )

    def close_file(self):
        self.fileOut.close()

    def listener(self):
        rospy.init_node("features_database_update", anonymous=True)
        rospy.Subscriber("/loc/ml/features", VisualLandmarks, self.callback)
        rospy.spin()


if __name__ == "__main__":

    try:

        if len(sys.argv) < 5:
            print(
                "Usage: activity_db_generator.py <activity date> <map name> <activity name> <location to save>"
            )
            print("       <activity date>    = 'YYYYMMDD'")
            print("       <map name>         = 'mapName'")
            print("       <activity name>    = 'activityName'")
            print("       <location to save> = '/location/to/save/'")
            sys.exit(1)

        else:
            activity_date = sys.argv[1]
            map_name = sys.argv[2]
            activity_name = sys.argv[3]
            location_name = sys.argv[4]
            output_filename = (
                location_name
                + activity_date
                + "_"
                + map_name
                + "_"
                + activity_name
                + "_db.csv"
            )

        obj = Activity_DBGenerator()
        obj.open_file(output_filename, activity_name, map_name, activity_date)
        obj.listener()

    except KeyboardInterrupt:
        print("\n <-CTRL-C EXIT: USER manually exited!->")
        obj.close_file()
        sys.exit(0)
