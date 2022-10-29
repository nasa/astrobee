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
  rosbag_topic_filter.py in.bag -r "/gs/*" -r /hw/cam_sci/compressed out.bag
    Filters out rosjava messages produced by Guest Science and SciCamImage APKs
    that have incomplete message definitions that break many rosbag API operations,
    including the command-line utilities 'rosbag filter' and 'rosbag check'.
  rosbag_topic_filter.py in.bag -a /mgt/sys_monitor/time_sync out.bag
    Keep only a specific message of interest for data analysis so downstream script
    can run much faster on a smaller bag.
"""

from __future__ import print_function

import argparse
import collections
import fnmatch
import os
import re
import sys

import rosbag

# check if pattern contains a character that is special for fnmatch
FNMATCH_SPECIAL_REGEX = re.compile(r"[\*\?\[]")


def is_special(fnmatch_pattern):
    return FNMATCH_SPECIAL_REGEX.search(fnmatch_pattern) is not None


def topic_filter(verbose, inbag_path, accept_patterns, reject_patterns, outbag_path):
    with rosbag.Bag(inbag_path, "r") as inbag:
        if (
            accept_patterns
            and not reject_patterns
            and all([not is_special(p) for p in accept_patterns])
        ):
            # if the accept patterns are just a list of plain topics and there
            # are no reject patterns, there's no need to do an extra slow pass
            # through the whole bag just to extract what topics it contains
            accept_topics = accept_patterns
        else:
            all_topics = inbag.get_type_and_topic_info()[1].keys()
            accept_topics = []
            for topic in all_topics:
                do_accept = True
                if accept_patterns:
                    do_accept = any(
                        [fnmatch.fnmatch(topic, p) for p in accept_patterns]
                    )
                if reject_patterns:
                    do_accept = do_accept and not any(
                        [fnmatch.fnmatch(topic, p) for p in reject_patterns]
                    )
                if do_accept:
                    accept_topics.append(topic)

            for p in accept_patterns:
                if not fnmatch.filter(all_topics, p):
                    print("warning: no topics in bag match accept pattern '%s'" % p)
            for p in reject_patterns:
                if not fnmatch.filter(all_topics, p):
                    print("warning: no topics in bag match reject pattern '%s'" % p)

        if verbose:
            print("accept topics:\n  %s" % "\n  ".join(sorted(accept_topics)))

        msg_count = collections.Counter()
        with rosbag.Bag(outbag_path, "w") as outbag:
            if accept_topics:
                for topic, msg, t in inbag.read_messages(accept_topics):
                    outbag.write(topic, msg, t)
                    msg_count[topic] += 1
            else:
                # don't call read_messages() with an empty accept_topics list:
                # turns out, it would iterate through all messages instead of
                # no messages
                pass

        for topic in accept_topics:
            if msg_count[topic] == 0:
                print("warning: no messages on topic '%s' in bag" % topic)
        if sum([c for c in msg_count.values()]) == 0:
            print("warning: output bag does not contain any messages")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("inbag", nargs="+", help="Input bagfile with bayer images.")
    parser.add_argument(
        "-o",
        "--output",
        help="path for output bag",
        default="filtered_{inbag}",
    )
    parser.add_argument(
        "-a",
        "--accept",
        nargs="?",
        help="Add topic pattern to accept list",
        default=[],
        action="append",
    )
    parser.add_argument(
        "-r",
        "--reject",
        nargs="?",
        help="Add topic pattern to reject list",
        default=[],
        action="append",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="Print debug info",
        action="store_true",
    )

    args = parser.parse_args()

    for inbag_path in args.inbag:
        # Check if input bag exists
        if not os.path.isfile(inbag_path):
            print(("Bag file " + inbag_path + " does not exist."))
            sys.exit()

        output_bag_name = (
            os.path.dirname(inbag_path)
            + "/"
            + args.output.format(inbag=os.path.basename(inbag_path))
        )

        # Check if output bag already exists
        if os.path.exists(output_bag_name):
            parser.error("not replacing existing file %s" % output_bag_name)

        topic_filter(
            args.verbose, inbag_path, args.accept, args.reject, output_bag_name
        )
