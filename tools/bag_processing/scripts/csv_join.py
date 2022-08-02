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
Join two CSV files (call them A and B) by timestamp.

A typical use case would be to join sensor messages with robot pose
messages, so the sensor+pose output can be used to generate a
scatter plot showing how the sensor value varies with position.

Each input CSV file is assumed to be the output of a "rostopic echo -b
in.bag -p <topic>" command, where the message type for that topic
includes the standard ROS message header.

The joined output CSV file has the following properties:

- Each row contains columns from both inputs A+B and represents their
  joined values "at the same time", based on the ROS header.stamp field.

- The timestamp ('time' column) of each output row is drawn from the
  header.stamp field of input A. This is called a "left join" in database
  terminology.

- The 'time' column from each input, which is the time when the bag
  recorder received the message, is ignored in favor of the
  'header.stamp' field set by the publishing node.

- In general, inputs A and B are asynchronous, so there may be no input
  B row with the same timestamp as the output row. Instead:

  - The input B row with the nearest timestamp is used.

  - If there is no input B row within the time delta tolerance, the
    output row is dropped.

- The output header column names have the names of the input files
  prepended so you can tell which columns came from which input. You may
  want to rename the input files before running this script if you care
  about the column name prefix.
"""

from __future__ import print_function

import argparse
import logging
import os

import pandas as pd


def read_csv(in_path):
    df = pd.read_csv(in_path)

    in_name = os.path.splitext(os.path.basename(in_path))[0]

    # HACK: This specifically deals with the strange format of /hw/wifi message
    # by promoting the header embedded inside the signals0 field to the top
    # level. It should have no effect on other messages.
    df.columns = df.columns.str.replace(".signals0.header", ".header")

    df.columns = df.columns.str.replace("field.", "")
    df.columns = df.columns.str.replace(".", "_")
    # df["header_stamp"] = pd.to_datetime(df["header_stamp"], unit="ns")
    df.reindex(df["header_stamp"])
    df = df.drop(columns=["%time"])
    df.columns = [in_name + "__" + c for c in df.columns]
    return in_name, df


def csv_join(csv_a, csv_b, out_path, verbose=False):
    if os.path.exists(out_path):
        logging.critical("not overwriting existing file %s", out_path)
        return

    a_name, a = read_csv(csv_a)
    b_name, b = read_csv(csv_b)
    out = pd.merge_asof(
        a,
        b,
        left_index=True,
        right_index=True,
        direction="nearest",
        tolerance=int(1e9),  # ns
    )
    out.insert(0, "%time", out[a_name + "__header_stamp"])
    out.to_csv(out_path, index=False)
    logging.info("wrote output to %s", out_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="print more debug info",
        action="store_true",
    )

    parser.add_argument("csv_a", help="input A CSV filename")
    parser.add_argument("csv_b", help="input B CSV filename")
    parser.add_argument("csv_out", help="output joined CSV Filename")

    args = parser.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(message)s")

    csv_join(args.csv_a, args.csv_b, args.csv_out, args.verbose)
