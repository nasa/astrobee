#!/usr/bin/python
import rospy
from ff_hw_msgs.msg import EpsHousekeeping


def callback(data):
    pmc_1 = float("nan")
    pmc_2 = float("nan")
    system = float("nan")
    for c in data.values:
        if c.name.find("MOTOR1_I") >= 0:
            pmc_1 = c.value
        if c.name.find("MOTOR2_I") >= 0:
            pmc_2 = c.value
        if c.name.find("SYSTEM_I") >= 0:
            system = c.value
    print(("system= %.3fA | pmc_1= %.3fA | pmc_2=%.3fA" % (system, pmc_1, pmc_2)))


def listener():
    rospy.init_node("pmc_consumption", anonymous=True)
    rospy.Subscriber("/hw/eps/housekeeping", EpsHousekeeping, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
