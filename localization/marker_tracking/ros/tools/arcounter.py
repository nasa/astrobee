#!/usr/bin/python
import rospy

from ff_msgs.msg import VisualLandmarks

def callback(data):
    print(len(data.landmarks))
    
def listener():
    rospy.init_node('arcounter', anonymous=True)
    rospy.Subscriber("/loc/ml/features", VisualLandmarks, callback)
    rospy.spin()
    
if __name__ == '__main__':
    print("arcounter")
    listener()
