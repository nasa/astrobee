\page bagprocessing Bag Processing

# Package Overview
The bag processing package provides several helper tools for handling bagfiles.

# Scripts
## apply\_histogram\_equalization\_to\_images
Applies either CLAHE or standard histogram equalization to images in a bagfile.  
See 'rosrun bag\_processing apply\_histogram\_equalization\_to\_images.py -h'
for further details and usage instructions.

## check\_bag\_for\_gaps
Prints gaps above provided maximum time for a bagfile and topic using either message header times or bag receive times.
Also prints stats for the time differences for messages with the provided topic in the bagfile.
See 'rosrun bag\_processing check\_bag\_for\_gaps.py -h'
for further details and usage instructions.

## merge\_bags
Merge bags looks for bag files in the current directory with a provided name prefix and merges these into a single bag file.  Bags are assumed to be numbered and are merged in numerical order.
