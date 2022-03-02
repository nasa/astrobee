# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import numpy as np
import rosbag

def get_topic_rates(
    bag_name,
    topic,
    min_time_diff_for_gap,
    use_header_time=True,
    verbose=False,
    ignore_zero_time_diffs=True,
):
    with rosbag.Bag(bag_name, "r") as bag:
        last_time = 0.0
        gaps = 0
        time_diffs = []
        for _, msg, t in bag.read_messages([topic]):
            time = (
                msg.header.stamp.secs + msg.header.stamp.nsecs * 1.0e-9
                if use_header_time
                else t.secs + t.nsecs * 1.0e-9
            )
            time_diff = time - last_time
            if last_time != 0 and time_diff >= min_time_diff_for_gap:
                if verbose:
                    print(
                        (
                            topic
                            + " Gap: time: "
                            + str(time)
                            + ", last_time: "
                            + str(last_time)
                            + ", diff: "
                            + str(time_diff)
                        )
                    )
                gaps += 1
            if last_time != 0 and (time_diff != 0 or not ignore_zero_time_diffs):
                time_diffs.append(time_diff)
            last_time = time

        mean_time_diff = np.mean(time_diffs)
        min_time_diff = np.min(time_diffs)
        max_time_diff = np.max(time_diffs)
        stddev_time_diff = np.std(time_diffs)
        if verbose:
            if use_header_time:
                print("Using Header time.")
            else:
                print("Using Receive time.")
            print(
                (
                    "Found "
                    + str(gaps)
                    + " time diffs >= "
                    + str(min_time_diff_for_gap)
                    + " secs."
                )
            )
            print(("Mean time diff: " + str(mean_time_diff)))
            print(("Min time diff: " + str(min_time_diff)))
            print(("Max time diff: " + str(max_time_diff)))
            print(("Stddev time diff: " + str(stddev_time_diff)))
