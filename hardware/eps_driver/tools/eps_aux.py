#!/usr/bin/python
import rospy

from ff_hw_msgs.msg import EpsHousekeeping

def callback(data):
    aux = float('nan')
    system = float('nan')
    for c in data.values:
        if c.name.find("AUX_PWR_I") >= 0 :
            aux = c.value
        if c.name.find("SYSTEM_I") >= 0 :
            system = c.value
    print("system= %.3fA | aux= %.3fA " % (system, aux))
    
def listener():
    rospy.init_node('pmc_consumption', anonymous=True)
    rospy.Subscriber("/hw/eps/housekeeping", EpsHousekeeping, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
