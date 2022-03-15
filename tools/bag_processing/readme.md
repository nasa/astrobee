\page bag_processing Bag Processing

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

## `convert_bayer`
Converts bayer encoded color images to grayscale and color image.

## `convert_all_bayer_bags`
Creates a new bagfile with grayscale and color images for the provided bags using their bayer encoded images.
If no bags are provided, runs conversion for each bag in the current directory.

## `get_msg_stats` 
Prints stats for a numeric field of a topic in a bagfile.

## `merge_bags`
Merges bagfiles with given prefix in the current directory.

## `merge_all_bags`
Creates merged bagfiles for each provided bag prefix or for each prefix in the current directory
if none are provided. 

## `trim_bag`
Creates a new bagfile with trimmed start and end times from a provided
bagfile.
