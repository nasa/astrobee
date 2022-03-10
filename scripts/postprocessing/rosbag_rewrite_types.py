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
import fnmatch
import json
import logging
import os

import genpy
import rosbag
import roslib

DEFAULT_RULES_FILE = "rosbag_rewrite_types_rules.json"

# See the following sources for many of the genpy/rosbag/roslib patterns used
# here:
#   http://docs.ros.org/en/melodic/api/rosbag/html/python/rosbag.migration-pysrc.html
#   https://github.com/gavanderhoorn/rosbag_fixer


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
    if rule["new_type_md5sum"] != new_pytype._md5sum:
        logging.warning("warning: md5sum mismatch for: %s", new_type)
        logging.warning("  migration rule md5sum: %s", rule["new_type_md5sum"])
        logging.warning("  latest md5sum: %s", new_pytype._md5sum)
        logging.warning("  [a changed message definition could cause data corruption]")

    val = {
        "type": new_type,
        "md5sum": rule["new_type_md5sum"],
        "message_definition": new_pytype._full_text,
        "pytype": new_pytype,
    }

    return (key, val)


def get_rules(rules_files):
    fix_topic_patterns = []
    rename_lookup = {}
    for rules_file in rules_files:
        with open(rules_file, "r") as rules_stream:
            rules_info = json.load(rules_stream)

        for k, rules in rules_info.iteritems():
            if k == "fix_message_definition_topic_patterns":
                fix_topic_patterns += rules
            elif k == "rename_types":
                rename_lookup.update(dict((get_rule_entry(r) for r in rules)))

    return (fix_topic_patterns, rename_lookup)


def topic_matcher(topic, topic_patterns):
    return any((fnmatch.fnmatch(topic, p) for p in topic_patterns))


def fix_message_definitions(inbag, fix_topic_patterns, verbose=False):
    """
    Modifies inbag metadata (in memory, not on disk). Changes will take
    effect when inbag's messages are subsequently written to outbag.
    """

    if not fix_topic_patterns:
        return

    fixed_topics = set()
    for conn in inbag._get_connections():
        if not topic_matcher(conn.topic, fix_topic_patterns):
            continue
        pytype = roslib.message.get_message_class(conn.datatype)
        if pytype is None:
            raise ValueError("Message class '%s' not found." % conn.datatype)
        conn.header["message_definition"] = pytype._full_text
        conn.msg_def = pytype._full_text
        fixed_topics.add(conn.topic)

    if verbose:
        logging.info("Fixed message definitions for topics:")
        if fixed_topics:
            for topic in sorted(fixed_topics):
                logging.info("  %s", topic)
        else:
            logging.info("  [none]")


def rename_types(inbag, outbag, rename_lookup, verbose=False):
    num_renamed = collections.Counter()
    for topic, msg, t, conn in inbag.read_messages(
        raw=True, return_connection_header=True
    ):
        old_type, msg_data, old_type_md5sum, msg_pos, msg_pytype = msg
        rename_key = (old_type, old_type_md5sum)
        tgt = rename_lookup.get(rename_key)
        if tgt is None:
            # just copy the message to outbag, no rewrite needed
            outbag.write(topic, msg, t, raw=True)
        else:
            num_renamed[rename_key] += 1
            # rewrite the necessary fields of the message and connection header
            new_msg = (tgt["type"], msg_data, tgt["md5sum"], msg_pos, tgt["pytype"])
            for k in ("type", "md5sum", "message_definition"):
                conn[k] = tgt[k]
            outbag.write(topic, new_msg, t, connection_header=conn, raw=True)

    if verbose:
        # summary of renamed messages
        migrated = sorted(num_renamed.keys())
        logging.info("Renamed message counts by type:")
        if not migrated:
            logging.info("  [no matching messages found]")
        for key in migrated:
            old_type, old_type_md5sum = key
            tgt = rename_lookup[key]
            logging.info(
                "  %5d %s %s -> %s %s",
                num_renamed[key],
                old_type,
                old_type_md5sum[:8],
                tgt["type"],
                tgt["md5sum"][:8],
            )


def rewrite_msg_types1(inbag_path, outbag_path, rules, verbose=False, no_reindex=False):
    if os.path.exists(outbag_path):
        logging.error("Not overwriting existing file %s" % outbag_path)
        return

    fix_topic_patterns, rename_lookup = rules
    with rosbag.Bag(inbag_path, "r") as inbag, rosbag.Bag(
        outbag_path, "w", options=inbag.options
    ) as outbag:
        fix_message_definitions(inbag, fix_topic_patterns, verbose)
        rename_types(inbag, outbag, rename_lookup, verbose)

    cmd = "rosbag reindex %s" % outbag_path
    if no_reindex:
        logging.info("%s probably needs to be re-indexed. Run: %s\n", outbag_path, cmd)
    else:
        dosys(cmd)
        dosys("rm %s" % (os.path.splitext(outbag_path)[0] + ".orig.bag"))


def rewrite_msg_types(
    inbag_paths, outbag_path_pattern, rules_files, verbose=False, no_reindex=False
):
    rules = get_rules(rules_files)

    for inbag_path in inbag_paths:
        inbag_name = os.path.splitext(inbag_path)[0]
        outbag_path = outbag_path_pattern.format(inbag=inbag_name)
        rewrite_msg_types1(
            inbag_path, outbag_path, rules, verbose=verbose, no_reindex=no_reindex
        )


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
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
        nargs="?",
        help="path to input rules file (specify multiple times for multiple files)",
        default=[],
        action="append",
    )
    parser.add_argument(
        "-n",
        "--no-reindex",
        action="store_true",
        help="suppress 'rosbag reindex' call",
        default=False,
    )
    parser.add_argument("inbag", nargs="+", help="input bag")

    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.WARN
    logging.basicConfig(level=level, format="%(message)s")

    if not args.rules:
        repo_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        )
        rel_paths = (
            "communication/ff_msgs/bmr/rosbag_rewrite_types_rules.json",
            "communication/ff_hw_msgs/bmr/rosbag_rewrite_types_rules.json",
        )
        rules_paths = [os.path.join(repo_root, p) for p in rel_paths]
        rules_paths = [p for p in rules_paths if os.path.isfile(p)]
        args.rules = rules_paths

    rewrite_msg_types(
        args.inbag,
        args.output,
        args.rules,
        verbose=args.verbose,
        no_reindex=args.no_reindex,
    )

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)
