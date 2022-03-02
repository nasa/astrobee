\page bagprocessing Bag Processing

# Package Overview
The bag processing package provides several helper tools for handling bagfiles.

# Usage Instructions
For each script, run 'rosrun bag\_processing script\_name.py -h' for further details and 
usage instructions.

# Scripts
## apply\_histogram\_equalization\_to\_images
Applies either CLAHE or standard histogram equalization to images in a bagfile.  

## check\_bag\_for\_gaps
Prints gaps above provided maximum time for a bagfile and topic using either message header times or bag receive times.
Also prints stats for the time differences for messages with the provided topic in the bagfile.

## convert\_bayer\_to\_grayscale
Converts bayer encoded color images to grayscale.

## convert\_all\_bayer\_bags\_to\_grayscale
Converts all bags in a directory with bayer encoded color images to grayscale.

## get\_msg\_stats 
Prints stats for a numeric field of a topic in a bagfile.

## merge\_bags
Merges bagfiles with given prefix in the current directory.

## merge\_all\_bags
Finds all the different bag prefixes in the current directory and creates 
merged bagfiles for these.

## trim\_bag
Creates a new bagfile with trimmed start and end times from a provided
bagfile.
