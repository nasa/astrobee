\page bagprocessing Bag Processing

# Package Overview
The bag processing package provides several helper tools for handling bagfiles.

# Usage Instructions
For each script, run `rosrun bag_processing script_name.py -h` for further details and 
usage instructions.

# Scripts
## `apply_histogram_equalization_to_images`
Applies either CLAHE or standard histogram equalization to images in a bagfile.  

## `check_bag_for_gaps`
Prints gaps above provided maximum time for a bagfile and topic using either message header times or bag receive times.
Also prints stats for the time differences for messages with the provided topic in the bagfile.

## `convert_bayer_to_grayscale`
Converts bayer encoded color images to grayscale.

## `convert_all_bayer_bags_to_grayscale`
Converts all bags in a directory with bayer encoded color images to grayscale.

## `get_msg_stats` 
Prints stats for a numeric field of a topic in a bagfile.

## `merge_bags`
Merges bagfiles with given prefix in the current directory.

## `merge_all_bags`
Finds all the different bag prefixes in the current directory and creates 
merged bagfiles for these.

## `trim_bag`
Creates a new bagfile with trimmed start and end times from a provided
bagfile.
