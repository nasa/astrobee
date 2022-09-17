\page bag_processing Bag Processing

# Package Overview
The bag processing package provides several helper tools for handling bagfiles.

# Usage Instructions
For each script, run `rosrun bag_processing script_name.py -h` for further details and
usage instructions.

# Scripts
## `apply_histogram_equalization_to_images`
Applies either CLAHE or standard histogram equalization to images in a bagfile.

## `utilities/bmr_renumber_enum`
This is not a standalone script. It is a library that provides utility
functions to be used in *.bmr bag migration rules to help with migrating
legacy messages that contain an enumerated field where the label
numbering has changed.

## `check_bag_for_gaps`
Prints gaps above provided maximum time for a bagfile and topic using either message header times or bag receive times.
Also prints stats for the time differences for messages with the provided topic in the bagfile.

## `clock_skew`
Analyze clock skew LLP->MLP and HLP->MLP. Inputs an `ars_default.bag`
containing the `/mgt/sys_monitor/time_sync` topic.

## `convert_bayer`
Converts bayer encoded color images to grayscale and color image.

## `convert_all_bayer_bags`
Creates a new bagfile with grayscale and color images for the provided bags using their bayer encoded images.
If no bags are provided, runs conversion for each bag in the current directory.

## `csv_join`
Join two CSV files by timestamp. (After creating the CSV files by exporting a message topic from a bag.)

## `get_msg_stats`
Prints stats for a numeric field of a topic in a bagfile.

## `merge_bags`
Merges bagfiles with given prefix in the current directory.

## `merge_all_bags`
Creates merged bagfiles for each provided bag prefix or for each prefix in the current directory
if none are provided.

## `rosbag_detect_bad_topics`
Detect bad topics that the `rosbag` API can't handle. (The main problem so
far has been `rosjava` messages that have incomplete message definition
dependency information that causes `rosbag` to raise an exception when it
tries to deserialize the message.)

## `rosbag_fix_all`
Master script to apply all passes of processing needed to fix our legacy
bag files. The actual processing steps are found in
`Makefile.rosbag_fix_all`. The strategy is explained in [1].

[1] https://github.com/nasa/astrobee/blob/develop/doc/general_documentation/maintaining_telemetry.md

## `rosbag_rewrite_types`
Tool that applies rules to fix bag files, as specified in
`rosbag_rewrite_types_rules.json`. It performs two types of fix:

1. Fix incomplete message definition metadata, as written by `rosjava`
   nodes. Which topics to fix is specified in the
   `fix_message_definition_topic_patterns` field of the rules file.

2. For message types whose definition changed in a way that makes
   migration infeasible, changes the message type name to be the
   "frozen" type name, so that migration is not needed. Which message
   types to fix is specified in the `rename_types` field of the rules
   file.

This script is normally invoked by `rosbag_fix_all`.

## `rosbag_topic_filter`
Filter `rosbag` messages based on topic. Like a subset of `rosbag filter`
functionality, but more robust to incomplete message definitions because
it doesn't need to deserialize the messages to filter on topic.

## `splice_bag`
Interactively splices a bagfile at selected timestamps to create multiple smaller bagfiles, which when combined 
span the original bagfile. Iterates through the bagfile images to allow the user to select splice points.

## `trim_bag`
Creates a new bagfile with trimmed start and end times from a provided
bagfile.
