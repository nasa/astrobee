import sys

import rosbag
import rospy
from ff_msgs.msg import Fault, FaultState


def process(msg, start):
    for f in msg.faults:
        # Fault 21 is perching arm node missing --> ignore
        if f.id != 21:
            elapsed = f.time_of_fault - start
            print(
                (
                    "secs_from_start=%d fault_id=%d timestamp=%d.%09d"
                    % (
                        int(elapsed.secs),
                        f.id,
                        f.time_of_fault.secs,
                        f.time_of_fault.nsecs,
                    )
                )
            )


if len(sys.argv) < 3:
    print("Usage: faults_during_bag.py short_bag_for_period ars_default_bag")
    exit(1)

short_bag_fn = sys.argv[1]
default_bag_fn = sys.argv[2]

print(("reading time bounds of %s" % short_bag_fn))
short_bag = rosbag.Bag(short_bag_fn)
start_ts = short_bag.get_start_time()
end_ts = short_bag.get_end_time()
short_bag.close()

print(
    (
        "will filter events of %s starting at %f to %f"
        % (default_bag_fn, start_ts, end_ts)
    )
)

default_bag = rosbag.Bag(default_bag_fn)

for topic, msg, time in default_bag.read_messages(
    topics=["/mgt/sys_monitor/state"],
    start_time=rospy.Time(start_ts),
    end_time=rospy.Time(end_ts),
):
    process(msg, rospy.Time(start_ts))
