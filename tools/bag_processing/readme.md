\page bagprocessing Bag Processing

# Package Overview
The bag processing package provides several helper tools for handling bagfiles.

# Scripts
## check\_bags\_for\_gaps
This is a simple tool to check for large gaps in imu or image data in a bagfile.

## merge\_bags
Merge bags looks for bag files in the current directory with a provided name prefix and merges these into a single bag file.  Bags are assumed to be numbered and are merged in numerical order.
