#!/usr/bin/python
import rospy

from ff_hw_msgs.msg import EpsChannelState

def callback(data):
    for c in data.channels:
        if c.name.find("PMC") >= 0 :
            print("%s : %f" % (c.name, c.value))
        if c.name.find("System") >= 0 :
            print("SYSTEM CURRENT = %f" % c.value)
    
def listener():
    rospy.init_node('pmc_consumption', anonymous=True)
    rospy.Subscriber("/hw/eps/channels", EpsChannelState, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
