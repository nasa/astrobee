#!/usr/bin/python
import sys
import rospy

from ff_msgs.msg import VisualLandmarks

use_csv_format = False

def callback(data):
    count=len(data.landmarks)
    if use_csv_format:
        ts = data.header.stamp
        if count == 0:
            pose_str = "nan, nan, nan"
        else:
            pose_str = "{:.3f}, {:.3f}, {:.3f}".format(data.pose.position.x,
                data.pose.position.y, data.pose.position.z)
            # print("%d.%s" % (ts.secs, format(ts.nsecs, '09d')) )
        print("%s, %d, %s" %
            (format(float(ts.secs)+float(ts.nsecs)/1E9, '.3f'), count, pose_str))
    else:
        print(count)

def listener(type):
    rospy.init_node('features_counter', anonymous=True)
    rospy.Subscriber("/loc/"+type+"/features", VisualLandmarks, callback)
    rospy.spin()
    
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: features_counter.py <type_of_feature> [csv_format]")
        print("       <type_of_feature> = 'ml' or 'ar'")
        print("       [cvs_format] = ',' | 'csv'")
        exit()
    else:
        feature = sys.argv[1]
        if len(sys.argv) > 2:
            if sys.argv[2] == ',' or sys.argv[2] == 'csv':
                use_csv_format = True
    if use_csv_format:
        print("seconds, " + feature + "_count, x, y, z")
    else:
        print("counting " + feature + " features:" )
    listener(feature)
