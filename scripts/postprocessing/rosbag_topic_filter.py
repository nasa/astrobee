#!/usr/bin/env python
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

"""
Filter rosbag messages based on topic. Like a subset of 'rosbag filter' functionality,
but more robust to malformed bags because it doesn't need to parse the messages
to filter on topic.

Topic filtering is controlled by an accept list and a reject list, as follows:
- By default, all messages are accepted.
- However, if the accept list is non-empty, only topics that match one of the patterns
  in the accept list are accepted.
- If the reject list is non-empty, topics that match one of the reject patterns will be
  rejected, even if they match an accept pattern.

The patterns in the accept and reject list can be simple strings, or you can use
Python fnmatch Unix shell-style wildcards, like "/gs/*" to match any topic that starts
with "/gs/". Note that the "/" character is not treated as special in this context.
Remember to quote patterns that include wildcards so they are not expanded by your shell.

Specify multiple --accept or --reject arguments on the command line to add multiple
patterns to the accept or reject list.

Examples:
  topic_filter.py in.bag -r "/gs/gs_manager/*" out.bag
    Filters out rosjava messages produced by Guest Science Manager with malformed
    message definitions that break many rosbag API operations, including the
    command-line utilities 'rosbag filter' and 'rosbag check'.
  topic_filter.py in.bag -a "/mgt/sys_monitor/time_sync" out.bag
    Keep only a specific message of interest for data analysis so downstream script
    can run much faster on a smaller bag.
"""

import argparse
import fnmatch
import sys

import rosbag


def topic_filter(inbag_path, accept_patterns, reject_patterns, outbag_path):
    with rosbag.Bag(inbag_path, "r") as inbag:
        all_topics = inbag.get_type_and_topic_info()[1].keys()
        accept_topics = []
        for topic in all_topics:
            do_accept = True
            if accept_patterns:
                do_accept = any([fnmatch.fnmatch(topic, p) for p in accept_patterns])
            if reject_patterns:
                do_accept = do_accept and not any(
                    [fnmatch.fnmatch(topic, p) for p in reject_patterns]
                )
            if do_accept:
                accept_topics.append(topic)

        with rosbag.Bag(outbag_path, "w") as outbag:
            for topic, msg, t in inbag.read_messages(accept_topics):
                outbag.write(topic, msg, t)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "-a",
        "--accept",
        nargs="?",
        help="Add topic pattern to accept list",
        action="append",
    )
    parser.add_argument(
        "-r",
        "--reject",
        nargs="?",
        help="Add topic pattern to reject list",
        action="append",
    )
    parser.add_argument("inbag", help="Input bag")
    parser.add_argument("outbag", help="Filtered output bag")

    args = parser.parse_args()
    topic_filter(args.inbag, args.accept, args.reject, args.outbag)
