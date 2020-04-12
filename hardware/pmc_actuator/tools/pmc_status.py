#!/usr/bin/python
import rospy

from ff_hw_msgs.msg import PmcTelemetry

def callback(data):
    # print("got new data!")
    index = 0
    for p in data.statuses:
        mode = p.status_1 & 0b1
        state = p.status_1 >> 2 & 0b11
        bad_crc = p.status_1 >> 6 & 0b1
        m_disabled = p.status_1 >> 5 & 0b1
        
        speedcam_line = p.status_2 >> 1 & 0b1
        print("PMC[%d]: cmd_id=%03d m_speed=%03d m_cur=%02d mode=%d, state=%d bad_crc=%d m_dis=%d sc_line=%d" % 
            (index, p.command_id, p.motor_speed, p.motor_current,
                mode, state, bad_crc, m_disabled, speedcam_line))
        index = index + 1
    
def listener():
    rospy.init_node('pmc_status', anonymous=True)
    rospy.Subscriber("/hw/pmc/telemetry", PmcTelemetry, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
