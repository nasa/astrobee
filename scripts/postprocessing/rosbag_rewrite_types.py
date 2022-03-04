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
This script has the same basic goal as rosbag migration ("rosbag fix")
but takes an alternative approach of rewriting the message to have a
different type name without changing the raw message contents.

When and how to use it:

1. Determine if you have a message type whose definition has changed
   incompatibly such that it's not feasible to migrate legacy bags
   containing the message to use the new definition of the same message
   type. For example, maybe an important field of the old message has
   been deleted or has changed semantics, so that there's no easy and
   robust way to capture the old field's information in the new message.

2. Define a new message type with a different name that has exactly the
   same data structure as the old message type. Note: Take care! Because
   this script copies over the raw message contents verbatim, any change
   in the data structure could cause data corruption. By convention, if
   the old message type was pkg_msgs/Msg, the new message type should be
   something like pkg_legacy_msgs/MsgV1 (or V2, V3, ... if needed). If
   you are copying a parent type that contains subtypes that may change
   in the future, those subtypes should also be copied, and the
   reference to the subtype in the parent type should be updated to
   point to the copy.

3. Add the message type to a rules file. An example file is
   rosbag_rewrite_types_rules.json found in this folder. Message
   instances whose type matches old_type and whose message definition
   MD5 sum matches old_type_md5sum will have their type renamed to
   new_type and MD5 sum updated to new_type_md5sum. You can figure out
   both the old_type and old_type_md5sum to use by running 'rosbag info'
   on the legacy bag you want to migrate. You can figure out
   new_type_md5sum by running 'rosmsg md5 <new_type>'.

4. Run the script to rename the types. The legacy messages should now
   be compatible with the latest version of your package.

Note: It seems like 'rosbag fix' should support this use case, but we
tried hard and couldn't get it to work in practice. It always tried to
rewrite the message to the latest version of the same type instead of
renaming the type, regardless of what we put in the *.bmr update
rules. Also, an advantage of using this script instead of 'rosbag fix'
is that both the migration rules and the script itself are much simpler.
"""

from __future__ import print_function

import argparse
import collections
import json
import logging
import os

import genpy
import pprint
import rosbag
import rosmsg

DEFAULT_RULES_FILE = "rosbag_rewrite_types_rules.json"

# See the following source for many of the rosbag/rosmsg/genpy patterns used
# here:
#   http://docs.ros.org/en/melodic/api/rosbag/html/python/rosbag.migration-pysrc.html

def dosys(cmd):
    logging.info(cmd)
    ret = os.system(cmd)
    if ret != 0:
        logging.warning("Command failed with return value %s\n" % ret)
    return ret


def get_rule_entry(rule):
    key = (rule["old_type"], rule["old_type_md5sum"])

    new_type = rule["new_type"]
    new_pytype = genpy.message.get_message_class(new_type)
    latest_md5sum = rosmsg.rosmsg_md5(rosmsg.MODE_MSG, new_type)
    if rule["new_type_md5sum"] != latest_md5sum:
        logging.warning("warning: md5sum mismatch for: %s", new_type)
        logging.warning("  migration rule md5sum: %s", rule["new_type_md5sum"])
        logging.warning("  latest md5sum: %s", latest_md5sum)
        logging.warning("  [a changed message definition could cause data corruption]")

    val = {
        "type": new_type,
        "md5sum": rosmsg.rosmsg_md5(rosmsg.MODE_MSG, new_type),
        "message_definition": new_pytype._full_text,
        "pytype": new_pytype,
    }

    return (key, val)


def get_rule_lookup(rules_files):
    lookup = {}
    for rules_file in rules_files:
        rules = json.load(open(rules_file, "r"))
        lookup.update(dict((get_rule_entry(r) for r in rules)))

    return lookup


def rewrite_msg_types1(inbag_path, outbag_path, rule_lookup, verbose=False, no_reindex=False):
    if os.path.exists(outbag_path):
        logging.error("Not overwriting existing file %s" % outbag_path)
        return

    ctr = collections.Counter()
    with rosbag.Bag(inbag_path, "r") as inbag, rosbag.Bag(outbag_path, "w") as outbag:
        for topic, msg, t, conn in inbag.read_messages(raw=True, return_connection_header=True):
            old_type, msg_data, old_type_md5sum, msg_pos, msg_pytype = msg
            tgt = rule_lookup.get((old_type, old_type_md5sum))
            if tgt is None:
                # just copy the message to outbag, no rewrite needed
                outbag.write(topic, msg, t, raw=True)
            else:
                ctr[(old_type, old_type_md5sum)] += 1
                # rewrite the necessary fields of the message and connection header
                new_msg = (tgt["type"], msg_data, tgt["md5sum"], msg_pos, tgt["pytype"])
                for k in ("type", "md5sum", "message_definition"):
                    conn[k] = tgt[k]
                outbag.write(topic, new_msg, t, connection_header=conn, raw=True)

    if verbose:
        # summary of migrated messages
        migrated = sorted(ctr.keys())
        logging.info("Migrated message counts:")
        if not migrated:
            logging.info("  [no matching messages found]")
        for key in migrated:
            old_type, old_type_md5sum = key
            tgt = rule_lookup[key]
            logging.info(
                "  %5d %s %s -> %s %s",
                ctr[key],
                old_type,
                old_type_md5sum,
                tgt["type"],
                tgt["md5sum"]
            )

    cmd = "rosbag reindex %s" % outbag_path
    if no_reindex:
        logging.info("%s probably needs to be re-indexed. Run: %s\n", outbag_path, cmd)
    else:
        dosys(cmd)
        dosys("rm %s" % (os.path.splitext(outbag_path)[0] + ".orig.bag"))


def rewrite_msg_types(inbag_paths, outbag_path_pattern, rules_files, verbose=False, no_reindex=False):
    rule_lookup = get_rule_lookup(rules_files)

    for inbag_path in inbag_paths:
        inbag_name = os.path.splitext(inbag_path)[0]
        outbag_path = outbag_path_pattern.format(inbag=inbag_name)
        rewrite_msg_types1(inbag_path, outbag_path, rule_lookup, verbose)


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
    pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="print debug info",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="path for output bag",
        default="{inbag}.rewrite_types.bag",
    )
    parser.add_argument(
        "-r",
        "--rules",
        nargs="*",
        help="path to rewrite rules file (can specify multiple times) (default: %s)" % DEFAULT_RULES_FILE,
    )
    parser.add_argument(
        "-n",
        '--no-reindex',
        action='store_true',
        help="Suppress 'rosbag reindex' call",
        default=False,
    )
    parser.add_argument("inbag", nargs="+", help="input bag")

    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.WARN
    logging.basicConfig(level=level, format="%(message)s")

    if not args.rules:
        rules_file = DEFAULT_RULES_FILE
        if not os.path.exists(rules_file):
            # if default rules file not found in cwd, search in the same folder
            # as this script
            rules_file = os.path.join(
                os.path.dirname(os.path.realpath(__file__)), rules_file
            )
        args.rules = [rules_file]

    rewrite_msg_types(args.inbag, args.output, args.rules, args.verbose, args.no_reindex)

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)
