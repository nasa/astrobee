import rosbag
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

input_bag = rosbag.Bag('input.bag')
mapping_pose = []
for topic, msg, t in input_bag.read_messages(topics="/sparse_mapping/pose"):
    time = msg.header.time
    for i, pose in enumerate(msg.poses):
        point_x = pose.postion.x
        point_y = pose.postion.y
        point_z = pose.postion.z
        ori_x = pose.orientation.x
        ori_y = pose.orientation.y
        ori_z = pose.orientation.z
        ori_w = pose.orientation.w
        info = (time, point_x, point_y, point_z, ori_x, ori_y, ori_z, ori_w)
        mapping_pose.append(info)
        