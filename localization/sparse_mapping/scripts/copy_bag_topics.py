import rosbag
import sys

if len(sys.argv) < 3:
    print("Usage: python copy_bag_topics.py <input_bag> <output_bag>")
    sys.exit(1)

input_bag = sys.argv[1]
output_bag = sys.argv[2]

with rosbag.Bag(output_bag, 'a') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        outbag.write(topic, msg, t)
